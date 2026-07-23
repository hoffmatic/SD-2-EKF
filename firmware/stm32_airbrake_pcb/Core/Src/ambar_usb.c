/*
 * AMBAR USBX CDC TRANSPORT - IMPLEMENTATION
 *
 * Purpose
 *   Provides the direct STM32 USB CDC byte-stream transport for protocol-v2
 *   packets.  It owns USBX standalone initialization, cable attach/detach,
 *   COBS-frame accumulation, validated receive queues, framed transmit queues,
 *   and transport diagnostics.  It does not interpret or execute commands.
 *
 * Data/control flow
 *   AmbarUsb_Task() is called repeatedly by the cooperative application loop.
 *   CDC bytes are accumulated until the zero delimiter, decoded and CRC-checked
 *   by rocket_protocol, then queued for AmbarApp.  Outbound payloads are framed
 *   by rocket_protocol and drained through USBX's run-state API.  This is the
 *   transport portion of CODE_GUIDE.md [ARCH-3] and [ARCH-6].
 *
 * Section map
 *   1. Transport sizing and module-owned state
 *   2. USBX callbacks and stream/session helpers
 *   3. Receive framing and queue policy
 *   4. Initialization and cooperative service task
 *   5. Application-facing RX/TX/status API
 *   6. USBX standalone platform hooks
 *
 * Safety and assumptions
 *   Every frame is length-, COBS-, version-, and CRC-validated before delivery.
 *   ESTOP receives a one-slot priority path so normal queue saturation cannot
 *   hide it, but AmbarApp remains responsible for executing the stop.  Detach or
 *   CDC deactivation discards partial frames and stale queued output.  Calls are
 *   expected from the main cooperative context; no queue is a general ISR-safe
 *   synchronization primitive.  USB must be serviced frequently and no function
 *   here may introduce a blocking wait.
 */

#include "ambar_usb.h"

#include "ambar_features.h"
#include "main.h"
#include "ux_api.h"
#include "ux_dcd_stm32.h"
#include "ux_device_class_cdc_acm.h"
#include "ux_device_descriptors.h"

#include <string.h>

/* ===================== TRANSPORT SIZING AND MODULE STATE ===================== */

/* Fixed pools keep USB memory and queue latency bounded on the target. */
#define AMBAR_USB_MEMORY_SIZE 4096u
#define AMBAR_USB_RX_QUEUE_DEPTH 8u
#define AMBAR_USB_TX_QUEUE_DEPTH 8u
#define AMBAR_USB_CDC_CHUNK_SIZE 64u

typedef struct
{
    uint8_t bytes[ROCKET_PROTOCOL_MAX_FRAME];
    uint8_t length;
} AmbarUsbTxFrame_t;

extern PCD_HandleTypeDef hpcd_USB_DRD_FS;

__ALIGN_BEGIN static UCHAR s_usbx_memory[AMBAR_USB_MEMORY_SIZE] __ALIGN_END;
static UX_SLAVE_CLASS_CDC_ACM *s_cdc_acm;
static bool s_stack_ready;
static bool s_pcd_started;
static uint16_t s_tx_sequence;

static uint8_t s_cdc_rx_chunk[AMBAR_USB_CDC_CHUNK_SIZE];
static ULONG s_cdc_rx_actual;
static uint8_t s_encoded_rx[ROCKET_PROTOCOL_MAX_FRAME - 1u];
static uint8_t s_encoded_rx_length;
static bool s_drop_until_delimiter;

static RocketDecodedPacket s_rx_queue[AMBAR_USB_RX_QUEUE_DEPTH];
static uint8_t s_rx_head;
static uint8_t s_rx_tail;
static uint8_t s_rx_count;
static RocketDecodedPacket s_priority_estop;
static bool s_priority_estop_pending;

static AmbarUsbTxFrame_t s_tx_queue[AMBAR_USB_TX_QUEUE_DEPTH];
static uint8_t s_tx_head;
static uint8_t s_tx_tail;
static uint8_t s_tx_count;

static AmbarUsbStats_t s_stats;

static void ambar_usb_reset_streams(void);

/* ===================== USBX CALLBACKS AND SESSION HELPERS ===================== */

static VOID ambar_usb_cdc_activate(VOID *instance)
{
    /* Publish the class instance only after USBX has configured CDC ACM. */
    s_cdc_acm = (UX_SLAVE_CLASS_CDC_ACM *)instance;
}

static VOID ambar_usb_cdc_deactivate(VOID *instance)
{
    (void)instance;
    s_cdc_acm = UX_NULL;
    s_stats.configured = false;
    /*
     * A host reset can deactivate CDC while PA9/VBUS remains high.  Discard
     * partial COBS input and old queued output so a new USB session cannot
     * inherit bytes or ACKs from the session that was reset.
     */
    ambar_usb_reset_streams();
}

static bool ambar_usb_cable_attached(void)
{
    /* VBUS sense prevents presenting a device pull-up when the host is absent. */
#if AMBAR_FEATURE_USB_REQUIRE_VBUS_SENSE
    return HAL_GPIO_ReadPin(USB_SENSE_GPIO_Port, USB_SENSE_Pin) == GPIO_PIN_SET;
#else
    return true;
#endif
}

static void ambar_usb_configure_clock_recovery(void)
{
    /* Discipline HSI48 from USB SOF so full-speed signaling remains in tolerance. */
    RCC_CRSInitTypeDef crs = {0};

    __HAL_RCC_CRS_CLK_ENABLE();
    crs.Prescaler = RCC_CRS_SYNC_DIV1;
    crs.Source = RCC_CRS_SYNC_SOURCE_USB;
    crs.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
    crs.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000u, 1000u);
    crs.ErrorLimitValue = 34u;
    crs.HSI48CalibrationValue = 32u;
    HAL_RCCEx_CRSConfig(&crs);
}

static void ambar_usb_reset_streams(void)
{
    /* Begin a clean CDC session without carrying partial input or stale ACKs. */
    s_cdc_rx_actual = 0u;
    s_encoded_rx_length = 0u;
    s_drop_until_delimiter = false;
    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_rx_count = 0u;
    s_priority_estop_pending = false;
    s_tx_head = 0u;
    s_tx_tail = 0u;
    s_tx_count = 0u;
}

/* ===================== RECEIVE FRAMING AND QUEUE POLICY ===================== */

static void ambar_usb_queue_received(const RocketDecodedPacket *packet)
{
    /* Reserve independent delivery for ESTOP; all other packets use bounded FIFO. */
    const bool is_estop = packet->header.type == ROCKET_PKT_COMMAND
        && packet->payload_length == ROCKET_COMMAND_PREFIX_SIZE
        && packet->payload[0] == ROCKET_CMD_ESTOP
        && packet->payload[1] == 0u;

    if (is_estop)
    {
        s_priority_estop = *packet;
        s_priority_estop_pending = true;
        return;
    }

    if (s_rx_count >= AMBAR_USB_RX_QUEUE_DEPTH)
    {
        ++s_stats.receive_queue_drops;
        return;
    }

    s_rx_queue[s_rx_tail] = *packet;
    s_rx_tail = (uint8_t)((s_rx_tail + 1u) % AMBAR_USB_RX_QUEUE_DEPTH);
    ++s_rx_count;
}

static void ambar_usb_finish_encoded_frame(void)
{
    /* Decode only the bytes before the delimiter; malformed frames are counted. */
    RocketDecodedPacket packet;
    const RocketProtocolResult result = RocketProtocol_DecodeFrame(
        s_encoded_rx,
        s_encoded_rx_length,
        &packet);

    if (result == ROCKET_PROTOCOL_OK)
    {
        ++s_stats.received_frames;
        ambar_usb_queue_received(&packet);
    }
    else
    {
        ++s_stats.receive_errors;
    }

    s_encoded_rx_length = 0u;
}

static void ambar_usb_consume_bytes(const uint8_t *data, size_t length)
{
    /* Reassemble arbitrary CDC chunks and resynchronize after an oversized frame. */
    for (size_t i = 0u; i < length; ++i)
    {
        const uint8_t byte = data[i];

        if (byte == ROCKET_PROTOCOL_FRAME_DELIMITER)
        {
            if (s_drop_until_delimiter)
            {
                s_drop_until_delimiter = false;
                s_encoded_rx_length = 0u;
            }
            else if (s_encoded_rx_length != 0u)
            {
                ambar_usb_finish_encoded_frame();
            }
            continue;
        }

        if (s_drop_until_delimiter)
        {
            continue;
        }

        if (s_encoded_rx_length >= sizeof(s_encoded_rx))
        {
            ++s_stats.receive_errors;
            s_encoded_rx_length = 0u;
            s_drop_until_delimiter = true;
            continue;
        }

        s_encoded_rx[s_encoded_rx_length++] = byte;
    }
}

/* ===================== INITIALIZATION AND COOPERATIVE SERVICE ===================== */

bool AmbarUsb_Init(void)
{
#if !AMBAR_FEATURE_USB_PROTOCOL
    memset(&s_stats, 0, sizeof(s_stats));
    return false;
#else
    UCHAR *framework_hs;
    UCHAR *framework_fs;
    UCHAR *string_framework;
    UCHAR *language_framework;
    ULONG framework_hs_length;
    ULONG framework_fs_length;
    ULONG string_framework_length;
    ULONG language_framework_length;
    UX_SLAVE_CLASS_CDC_ACM_PARAMETER cdc_parameter;

    memset(&s_stats, 0, sizeof(s_stats));
    ambar_usb_reset_streams();
    s_cdc_acm = UX_NULL;
    s_stack_ready = false;
    s_pcd_started = false;
    s_tx_sequence = 0u;

    if (HAL_PCD_GetState(&hpcd_USB_DRD_FS) != HAL_PCD_STATE_READY)
    {
        ++s_stats.stack_errors;
        return false;
    }

    ambar_usb_configure_clock_recovery();

    if (ux_system_initialize(s_usbx_memory,
                             sizeof(s_usbx_memory),
                             UX_NULL,
                             0u) != UX_SUCCESS)
    {
        ++s_stats.stack_errors;
        return false;
    }

    framework_hs = USBD_Get_Device_Framework_Speed(USBD_HIGH_SPEED,
                                                    &framework_hs_length);
    framework_fs = USBD_Get_Device_Framework_Speed(USBD_FULL_SPEED,
                                                    &framework_fs_length);
    string_framework = USBD_Get_String_Framework(&string_framework_length);
    language_framework = USBD_Get_Language_Id_Framework(&language_framework_length);

    if (ux_device_stack_initialize(framework_hs,
                                   framework_hs_length,
                                   framework_fs,
                                   framework_fs_length,
                                   string_framework,
                                   string_framework_length,
                                   language_framework,
                                   language_framework_length,
                                   UX_NULL) != UX_SUCCESS)
    {
        ++s_stats.stack_errors;
        return false;
    }

    memset(&cdc_parameter, 0, sizeof(cdc_parameter));
    cdc_parameter.ux_slave_class_cdc_acm_instance_activate = ambar_usb_cdc_activate;
    cdc_parameter.ux_slave_class_cdc_acm_instance_deactivate = ambar_usb_cdc_deactivate;
    cdc_parameter.ux_slave_class_cdc_acm_parameter_change = UX_NULL;

    if (ux_device_stack_class_register(_ux_system_slave_class_cdc_acm_name,
                                       ux_device_class_cdc_acm_entry,
                                       USBD_Get_Configuration_Number(CLASS_TYPE_CDC_ACM, 0u),
                                       USBD_Get_Interface_Number(CLASS_TYPE_CDC_ACM, 0u),
                                       &cdc_parameter) != UX_SUCCESS)
    {
        ++s_stats.stack_errors;
        return false;
    }

    if (ux_dcd_stm32_initialize((ULONG)USB_DRD_FS,
                                (ULONG)&hpcd_USB_DRD_FS) != UX_SUCCESS)
    {
        ++s_stats.stack_errors;
        return false;
    }

    s_stack_ready = true;
    s_stats.stack_ready = true;
    return true;
#endif
}

void AmbarUsb_Task(void)
{
#if AMBAR_FEATURE_USB_PROTOCOL
    /* One nonblocking service pass: attach state, USBX, RX, then at most one TX. */
    if (!s_stack_ready)
    {
        return;
    }

    const bool attached = ambar_usb_cable_attached();
    s_stats.cable_attached = attached;

    if (attached && !s_pcd_started)
    {
        if (HAL_PCD_Start(&hpcd_USB_DRD_FS) == HAL_OK)
        {
            s_pcd_started = true;
        }
        else
        {
            ++s_stats.stack_errors;
            return;
        }
    }
    else if (!attached && s_pcd_started)
    {
        (void)ux_device_stack_disconnect();
        (void)HAL_PCD_Stop(&hpcd_USB_DRD_FS);
        s_pcd_started = false;
        s_cdc_acm = UX_NULL;
        ambar_usb_reset_streams();
        s_stats.configured = false;
        return;
    }

    if (!s_pcd_started)
    {
        return;
    }

    (void)ux_system_tasks_run();
    s_stats.configured = s_cdc_acm != UX_NULL;
    if (s_cdc_acm == UX_NULL)
    {
        return;
    }

    UINT state = ux_device_class_cdc_acm_read_run(s_cdc_acm,
                                                   s_cdc_rx_chunk,
                                                   sizeof(s_cdc_rx_chunk),
                                                   &s_cdc_rx_actual);
    if (state == UX_STATE_NEXT)
    {
        if (s_cdc_rx_actual != 0u)
        {
            ambar_usb_consume_bytes(s_cdc_rx_chunk, (size_t)s_cdc_rx_actual);
        }
        s_cdc_rx_actual = 0u;
    }
    else if (state == UX_STATE_ERROR)
    {
        ++s_stats.receive_errors;
        s_cdc_rx_actual = 0u;
    }

    if (s_tx_count != 0u)
    {
        AmbarUsbTxFrame_t *frame = &s_tx_queue[s_tx_head];
        ULONG actual = 0u;
        state = ux_device_class_cdc_acm_write_run(s_cdc_acm,
                                                   frame->bytes,
                                                   frame->length,
                                                   &actual);
        if (state == UX_STATE_NEXT)
        {
            s_tx_head = (uint8_t)((s_tx_head + 1u) % AMBAR_USB_TX_QUEUE_DEPTH);
            --s_tx_count;
            ++s_stats.transmitted_frames;
        }
        else if (state == UX_STATE_ERROR || state == UX_STATE_EXIT)
        {
            s_tx_head = (uint8_t)((s_tx_head + 1u) % AMBAR_USB_TX_QUEUE_DEPTH);
            --s_tx_count;
            ++s_stats.transmit_drops;
        }
    }
#endif
}

/* ===================== APPLICATION-FACING QUEUES AND STATUS ===================== */

bool AmbarUsb_TakePacket(RocketDecodedPacket *packet)
{
    /* Priority ESTOP is always observed before the normal arrival-order queue. */
    if (packet == NULL)
    {
        return false;
    }

    if (s_priority_estop_pending)
    {
        *packet = s_priority_estop;
        s_priority_estop_pending = false;
        return true;
    }

    if (s_rx_count == 0u)
    {
        return false;
    }

    *packet = s_rx_queue[s_rx_head];
    s_rx_head = (uint8_t)((s_rx_head + 1u) % AMBAR_USB_RX_QUEUE_DEPTH);
    --s_rx_count;
    return true;
}

static bool ambar_usb_queue_packet_with_sequence(uint8_t type,
                                                  uint16_t sequence,
                                                  const uint8_t *payload,
                                                  size_t payload_length,
                                                  uint32_t time_ms)
{
#if AMBAR_FEATURE_USB_PROTOCOL
    if (!AmbarUsb_IsConfigured())
    {
        return false;
    }
    if (s_tx_count >= AMBAR_USB_TX_QUEUE_DEPTH)
    {
        ++s_stats.transmit_drops;
        return false;
    }

    AmbarUsbTxFrame_t *frame = &s_tx_queue[s_tx_tail];
    const size_t length = RocketProtocol_EncodeFrame(frame->bytes,
                                                     sizeof(frame->bytes),
                                                     type,
                                                     sequence,
                                                     time_ms,
                                                     payload,
                                                     payload_length);
    if (length == 0u || length > UINT8_MAX)
    {
        ++s_stats.transmit_drops;
        return false;
    }

    frame->length = (uint8_t)length;
    s_tx_tail = (uint8_t)((s_tx_tail + 1u) % AMBAR_USB_TX_QUEUE_DEPTH);
    ++s_tx_count;
    return true;
#else
    (void)type;
    (void)sequence;
    (void)payload;
    (void)payload_length;
    (void)time_ms;
    return false;
#endif
}

bool AmbarUsb_QueuePacket(uint8_t type,
                          const uint8_t *payload,
                          size_t payload_length,
                          uint32_t time_ms)
{
    /* Frame immediately into owned storage; USB transmission happens in Task(). */
    if (!ambar_usb_queue_packet_with_sequence(type,
                                               s_tx_sequence,
                                               payload,
                                               payload_length,
                                               time_ms))
    {
        return false;
    }
    ++s_tx_sequence;
    return true;
}

bool AmbarUsb_QueueCorrelatedPacket(uint8_t type,
                                    uint16_t sequence,
                                    const uint8_t *payload,
                                    size_t payload_length,
                                    uint32_t time_ms)
{
    return ambar_usb_queue_packet_with_sequence(type,
                                                 sequence,
                                                 payload,
                                                 payload_length,
                                                 time_ms);
}

bool AmbarUsb_IsConfigured(void)
{
#if AMBAR_FEATURE_USB_PROTOCOL
    return s_stack_ready && s_pcd_started && s_cdc_acm != UX_NULL;
#else
    return false;
#endif
}

AmbarUsbStats_t AmbarUsb_GetStats(void)
{
    /* Refresh live readiness fields before returning a by-value diagnostic copy. */
    s_stats.stack_ready = s_stack_ready;
    s_stats.configured = AmbarUsb_IsConfigured();
    return s_stats;
}

/* ===================== USBX STANDALONE PLATFORM HOOKS ===================== */

ALIGN_TYPE _ux_utility_interrupt_disable(VOID)
{
    /* USBX uses these short critical sections while running without an RTOS. */
    const ALIGN_TYPE flags = (ALIGN_TYPE)__get_PRIMASK();
    __disable_irq();
    return flags;
}

VOID _ux_utility_interrupt_restore(ALIGN_TYPE flags)
{
    __set_PRIMASK((uint32_t)flags);
}

ULONG _ux_utility_time_get(VOID)
{
    /* USBX timebase shares the monotonic HAL millisecond tick. */
    return (ULONG)HAL_GetTick();
}
