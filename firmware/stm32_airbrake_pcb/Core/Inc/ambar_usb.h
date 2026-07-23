/*
 * AMBAR USBX CDC TRANSPORT - PUBLIC INTERFACE
 *
 * Purpose and ownership
 *   Exposes the nonblocking direct-USB transport used by AmbarApp.  The module
 *   owns USBX memory, CDC stream state, protocol frame queues, and sequence
 *   numbers; callers exchange already-decoded packets or unframed payloads.
 *
 * Call pattern
 *   Call AmbarUsb_Init() once, AmbarUsb_Task() on every cooperative main-loop
 *   pass, AmbarUsb_TakePacket() until RX is empty, and AmbarUsb_QueuePacket() for
 *   outbound telemetry/ACKs.  rocket_protocol performs framing and validation;
 *   AmbarApp interprets commands and simulation samples.  See CODE_GUIDE.md
 *   [ARCH-3] and [ARCH-6].
 *
 * Section map
 *   1. Transport diagnostic snapshot
 *   2. Initialization and cooperative service
 *   3. Application-facing packet queues
 *   4. Readiness and diagnostic accessors
 *
 * Safety and assumptions
 *   Initialization is fail-soft so sensor/flight logic may continue without
 *   USB.  Queue calls are nonblocking and may report drops under backpressure.
 *   A successful dequeue means framing and CRC passed, not that a command is
 *   authorized; physical-command gates remain in the application/actuator path.
 */
#ifndef AMBAR_USB_H
#define AMBAR_USB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rocket_protocol.h"

#include <stdbool.h>
#include <stdint.h>

/* ===================== TRANSPORT DIAGNOSTICS ===================== */

typedef struct
{
    /* Lifetime counters distinguish malformed input, backpressure, and stack faults. */
    uint32_t received_frames;
    uint32_t receive_errors;
    uint32_t receive_queue_drops;
    uint32_t transmitted_frames;
    uint32_t transmit_drops;
    uint32_t stack_errors;

    /* Live readiness progresses from stack -> cable -> configured CDC session. */
    bool stack_ready;
    bool cable_attached;
    bool configured;
} AmbarUsbStats_t;

/* ===================== INITIALIZATION AND COOPERATIVE SERVICE ===================== */

/* Fail-soft initialization: false leaves the rest of the flight app running. */
bool AmbarUsb_Init(void);

/* Run USBX, CDC read/write state machines, and attach/detach handling. */
void AmbarUsb_Task(void);

/* ===================== APPLICATION PACKET QUEUES ===================== */

/* Dequeue one validated protocol packet.  Physical commands are never run here. */
bool AmbarUsb_TakePacket(RocketDecodedPacket *packet);

/* Serialize and queue one framed packet for the host. */
bool AmbarUsb_QueuePacket(uint8_t type,
                          const uint8_t *payload,
                          size_t payload_length,
                          uint32_t time_ms);

/*
 * Queue a reply whose protocol header explicitly echoes an accepted host
 * packet sequence. Used only for VARIABLE_HIL request/state correlation; normal
 * telemetry should continue using the transport-owned sequence above.
 */
bool AmbarUsb_QueueCorrelatedPacket(uint8_t type,
                                    uint16_t sequence,
                                    const uint8_t *payload,
                                    size_t payload_length,
                                    uint32_t time_ms);

/* ===================== READINESS AND DIAGNOSTICS ===================== */

bool AmbarUsb_IsConfigured(void);
AmbarUsbStats_t AmbarUsb_GetStats(void);

#ifdef __cplusplus
}
#endif

#endif /* AMBAR_USB_H */
