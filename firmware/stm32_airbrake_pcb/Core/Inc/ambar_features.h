/*
 * AMBAR COMPILE-TIME FEATURE AND BUILD-PROFILE CONFIGURATION
 *
 * Purpose
 *   Centralizes project-owned compile-time switches, validates that each switch
 *   is Boolean, resolves dependency-aware effective features, and packs the
 *   resulting build identity into telemetry/config status.  This file describes
 *   what code exists in a binary; runtime state still decides what it may do.
 *
 * Build flow
 *   Raw 0/1 defaults select radio, USB/simulation, logging, motion, watchdog, and
 *   presentation behavior.  Effective macros combine dependent switches so an
 *   impossible child feature is never reported as active.  The packed feature
 *   word lets host preflight verify the flashed binary before a test.  See the
 *   "Build profiles" section of CODE_GUIDE.md, plus [ARCH-3], [ARCH-5], and
 *   [ARCH-7] for the affected paths.
 *
 * Section map
 *   1. Radio and direct-USB communication defaults
 *   2. Storage, actuator, and presentation defaults
 *   3. Watchdog, sensor, and telemetry-detail defaults
 *   4. Compile-time value validation
 *   5. Dependency-aware effective features
 *   6. Reported feature/status bit word
 *
 * Safety and assumptions
 *   Every change requires rebuild, reflash, and host feature-bit verification.
 *   The current defaults identify the motion-presentation profile, but actuator
 *   startup remains disabled/unhomed and runtime HOME, health, arming, phase, and
 *   inhibit gates still apply.  Setting presentation motion to zero creates the
 *   GUI-only profile.  A future flight profile must explicitly disable bench and
 *   simulation paths and validate the watchdog; it is not merely a renamed demo
 *   binary.  Do not bypass a runtime gate by adding another feature switch here.
 */

#ifndef AMBAR_FEATURES_H
#define AMBAR_FEATURES_H

#include <stdint.h>

/* ===================== RADIO AND USB COMMUNICATION DEFAULTS ===================== */

/* SX1280 initialization, command receive, ACK/NACK, and boot message. */
#ifndef AMBAR_FEATURE_RADIO
#define AMBAR_FEATURE_RADIO 1
#endif

/* One-hertz binary flight telemetry. Effective only when RADIO is also 1. */
#ifndef AMBAR_FEATURE_TELEMETRY
#define AMBAR_FEATURE_TELEMETRY 1
#endif

/* Five-second unsolicited radio heartbeat. Effective only with RADIO. */
#ifndef AMBAR_FEATURE_RADIO_HEARTBEAT
#define AMBAR_FEATURE_RADIO_HEARTBEAT 1
#endif

/* Direct STM32 USB CDC with rocket_protocol v2 framing.  This does not alter
 * the SX1280/Arduino version-1 packet format.
 */
#ifndef AMBAR_FEATURE_USB_PROTOCOL
#define AMBAR_FEATURE_USB_PROTOCOL 1
#endif

/* Accept deterministic altitude/acceleration samples from the USB GUI after
 * an explicit SIM_START command. Effective only when USB_PROTOCOL is enabled.
 */
#ifndef AMBAR_FEATURE_SIMULATION_INPUT
#define AMBAR_FEATURE_SIMULATION_INPUT 1
#endif

/* Use the board's PA9 USB_SENSE input to start/stop the device pull-up.  Set
 * this to 0 only for a bench board whose VBUS-sense circuit is unavailable.
 */
#ifndef AMBAR_FEATURE_USB_REQUIRE_VBUS_SENSE
#define AMBAR_FEATURE_USB_REQUIRE_VBUS_SENSE 1
#endif

/*
 * Allow blocking SAVE_CONFIG and ERASE_LOG operations to be requested over
 * standalone USBX. Keep 0 for flight and presentations: the current flash
 * driver busy-polls sector erases and cannot service USB while doing so.
 */
#ifndef AMBAR_FEATURE_USB_FLASH_MAINTENANCE
#define AMBAR_FEATURE_USB_FLASH_MAINTENANCE 0
#endif

/* ===================== STORAGE AND MOTION-PROFILE DEFAULTS ===================== */

/* Flash flight-log commands and periodic snapshot writes.
 * This does not disable W25Q64-backed configuration load/save.
 */
#ifndef AMBAR_FEATURE_FLASH_LOGGING
#define AMBAR_FEATURE_FLASH_LOGGING 1
#endif

/*
 * Presentation-only hardware-motion profile.
 *
 * This is the single switch for the USB simulation -> real actuator demo build.
 * Setting it to 0 restores the motion-inhibited defaults below.  Even when it is
 * 1, the runtime still requires an explicit HOME-at-retracted-position command,
 * a healthy driver/configuration, an armed simulation, and a non-inhibited
 * flight command before DRV_ENN can be released.
 */
#ifndef AMBAR_FEATURE_PRESENTATION_MOTION
#define AMBAR_FEATURE_PRESENTATION_MOTION 1
#endif

/* Set to 1 only if a low-travel direction check proves extension is negative. */
#ifndef AMBAR_PRESENTATION_ACTUATOR_DIRECTION_INVERTED
#define AMBAR_PRESENTATION_ACTUATOR_DIRECTION_INVERTED 0
#endif

/* Master hardware-motion lock. Defaults to the presentation profile switch. */
#ifndef AMBAR_FEATURE_ACTUATOR
#define AMBAR_FEATURE_ACTUATOR AMBAR_FEATURE_PRESENTATION_MOTION
#endif

/* HOME, RETRACT, and BENCH_MOVE radio commands. Requires ACTUATOR as well. */
#ifndef AMBAR_FEATURE_BENCH_ACTUATOR_COMMANDS
#define AMBAR_FEATURE_BENCH_ACTUATOR_COMMANDS AMBAR_FEATURE_PRESENTATION_MOTION
#endif

/* ===================== WATCHDOG, SENSOR, AND OUTPUT DEFAULTS ===================== */

/* Independent watchdog with an approximately sixteen-second timeout.
 * It starts only after application initialization and is refreshed once per
 * completed main-loop pass. ERASE_LOG is intentionally refused while enabled
 * because a full-chip log-area erase exceeds this window. Keep 0 during
 * debugger-heavy presentation work.
 */
#ifndef AMBAR_FEATURE_WATCHDOG
#define AMBAR_FEATURE_WATCHDOG 0
#endif

/* LIS2MDL initialization and raw magnetometer telemetry. The vertical EKF does
 * not depend on this sensor, so disabling it leaves IMU/barometer flight state.
 */
#ifndef AMBAR_FEATURE_MAGNETOMETER_TELEMETRY
#define AMBAR_FEATURE_MAGNETOMETER_TELEMETRY 1
#endif

/* Optional 0x50/0x51/0x52 text TLVs in the binary telemetry packet. Command
 * ACK/NACK packets remain enabled so the ground station still has feedback.
 */
#ifndef AMBAR_FEATURE_VERBOSE_STATUS_TEXT
#define AMBAR_FEATURE_VERBOSE_STATUS_TEXT 1
#endif

/* Showcase-only phase lock. Set 0 to keep estimating and telemetering while
 * holding PAD_IDLE, which also keeps flight deployment inhibited.
 */
#ifndef AMBAR_FEATURE_AUTO_FLIGHT_PHASES
#define AMBAR_FEATURE_AUTO_FLIGHT_PHASES 1
#endif

/* ===================== COMPILE-TIME VALUE VALIDATION ===================== */

/* Reject typos such as 2 or -1 instead of silently creating an unsafe build. */
#if ((AMBAR_FEATURE_RADIO != 0) && (AMBAR_FEATURE_RADIO != 1))
#error "AMBAR_FEATURE_RADIO must be 0 or 1"
#endif
#if ((AMBAR_FEATURE_TELEMETRY != 0) && (AMBAR_FEATURE_TELEMETRY != 1))
#error "AMBAR_FEATURE_TELEMETRY must be 0 or 1"
#endif
#if ((AMBAR_FEATURE_RADIO_HEARTBEAT != 0) && (AMBAR_FEATURE_RADIO_HEARTBEAT != 1))
#error "AMBAR_FEATURE_RADIO_HEARTBEAT must be 0 or 1"
#endif
#if ((AMBAR_FEATURE_USB_PROTOCOL != 0) && (AMBAR_FEATURE_USB_PROTOCOL != 1))
#error "AMBAR_FEATURE_USB_PROTOCOL must be 0 or 1"
#endif
#if ((AMBAR_FEATURE_SIMULATION_INPUT != 0) && (AMBAR_FEATURE_SIMULATION_INPUT != 1))
#error "AMBAR_FEATURE_SIMULATION_INPUT must be 0 or 1"
#endif
#if ((AMBAR_FEATURE_USB_REQUIRE_VBUS_SENSE != 0) && (AMBAR_FEATURE_USB_REQUIRE_VBUS_SENSE != 1))
#error "AMBAR_FEATURE_USB_REQUIRE_VBUS_SENSE must be 0 or 1"
#endif
#if ((AMBAR_FEATURE_USB_FLASH_MAINTENANCE != 0) && (AMBAR_FEATURE_USB_FLASH_MAINTENANCE != 1))
#error "AMBAR_FEATURE_USB_FLASH_MAINTENANCE must be 0 or 1"
#endif
#if ((AMBAR_FEATURE_FLASH_LOGGING != 0) && (AMBAR_FEATURE_FLASH_LOGGING != 1))
#error "AMBAR_FEATURE_FLASH_LOGGING must be 0 or 1"
#endif
#if ((AMBAR_FEATURE_PRESENTATION_MOTION != 0) && (AMBAR_FEATURE_PRESENTATION_MOTION != 1))
#error "AMBAR_FEATURE_PRESENTATION_MOTION must be 0 or 1"
#endif
#if ((AMBAR_PRESENTATION_ACTUATOR_DIRECTION_INVERTED != 0) && (AMBAR_PRESENTATION_ACTUATOR_DIRECTION_INVERTED != 1))
#error "AMBAR_PRESENTATION_ACTUATOR_DIRECTION_INVERTED must be 0 or 1"
#endif
#if ((AMBAR_FEATURE_ACTUATOR != 0) && (AMBAR_FEATURE_ACTUATOR != 1))
#error "AMBAR_FEATURE_ACTUATOR must be 0 or 1"
#endif
#if ((AMBAR_FEATURE_BENCH_ACTUATOR_COMMANDS != 0) && (AMBAR_FEATURE_BENCH_ACTUATOR_COMMANDS != 1))
#error "AMBAR_FEATURE_BENCH_ACTUATOR_COMMANDS must be 0 or 1"
#endif
#if ((AMBAR_FEATURE_WATCHDOG != 0) && (AMBAR_FEATURE_WATCHDOG != 1))
#error "AMBAR_FEATURE_WATCHDOG must be 0 or 1"
#endif
#if ((AMBAR_FEATURE_MAGNETOMETER_TELEMETRY != 0) && (AMBAR_FEATURE_MAGNETOMETER_TELEMETRY != 1))
#error "AMBAR_FEATURE_MAGNETOMETER_TELEMETRY must be 0 or 1"
#endif
#if ((AMBAR_FEATURE_VERBOSE_STATUS_TEXT != 0) && (AMBAR_FEATURE_VERBOSE_STATUS_TEXT != 1))
#error "AMBAR_FEATURE_VERBOSE_STATUS_TEXT must be 0 or 1"
#endif
#if ((AMBAR_FEATURE_AUTO_FLIGHT_PHASES != 0) && (AMBAR_FEATURE_AUTO_FLIGHT_PHASES != 1))
#error "AMBAR_FEATURE_AUTO_FLIGHT_PHASES must be 0 or 1"
#endif

/* ===================== DEPENDENCY-AWARE EFFECTIVE FEATURES ===================== */

/* Effective dependency-aware switches used by application code and status. */
#define AMBAR_FEATURE_EFFECTIVE_TELEMETRY \
    (AMBAR_FEATURE_RADIO && AMBAR_FEATURE_TELEMETRY)
#define AMBAR_FEATURE_EFFECTIVE_HEARTBEAT \
    (AMBAR_FEATURE_RADIO && AMBAR_FEATURE_RADIO_HEARTBEAT)
#define AMBAR_FEATURE_EFFECTIVE_BENCH_COMMANDS \
    (AMBAR_FEATURE_ACTUATOR && AMBAR_FEATURE_BENCH_ACTUATOR_COMMANDS)
#define AMBAR_FEATURE_EFFECTIVE_SIMULATION \
    (AMBAR_FEATURE_USB_PROTOCOL && AMBAR_FEATURE_SIMULATION_INPUT)
#define AMBAR_FEATURE_EFFECTIVE_USB_FLASH_MAINTENANCE \
    (AMBAR_FEATURE_USB_PROTOCOL && AMBAR_FEATURE_USB_FLASH_MAINTENANCE)

/* ===================== REPORTED BUILD IDENTITY ===================== */

/*
 * Feature word reported as STATUS F=xxx and in config_flags bits 16..31:
 *   bit 0 radio                 bit 7 verbose telemetry text
 *   bit 1 effective telemetry  bit 8 automatic flight phases
 *   bit 2 flash logging         bit 9 effective heartbeat
 *   bit 3 actuator motion       bit 10 direct USB protocol
 *   bit 4 effective bench cmds  bit 11 effective simulation input
 *   bit 5 watchdog              bit 12 require USB VBUS sense
 *   bit 13 USB flash maintenance  bit 14 presentation motion profile
 *   bit 15 presentation direction inverted
 *   bit 6 magnetometer telemetry
 * Existing AmbarConfig status remains untouched in config_flags bits 0..15.
 */
#define AMBAR_FEATURE_ENABLED_BITS ( \
    ((uint32_t)AMBAR_FEATURE_RADIO << 0U) | \
    ((uint32_t)AMBAR_FEATURE_EFFECTIVE_TELEMETRY << 1U) | \
    ((uint32_t)AMBAR_FEATURE_FLASH_LOGGING << 2U) | \
    ((uint32_t)AMBAR_FEATURE_ACTUATOR << 3U) | \
    ((uint32_t)AMBAR_FEATURE_EFFECTIVE_BENCH_COMMANDS << 4U) | \
    ((uint32_t)AMBAR_FEATURE_WATCHDOG << 5U) | \
    ((uint32_t)AMBAR_FEATURE_MAGNETOMETER_TELEMETRY << 6U) | \
    ((uint32_t)AMBAR_FEATURE_VERBOSE_STATUS_TEXT << 7U) | \
    ((uint32_t)AMBAR_FEATURE_AUTO_FLIGHT_PHASES << 8U) | \
    ((uint32_t)AMBAR_FEATURE_EFFECTIVE_HEARTBEAT << 9U) | \
    ((uint32_t)AMBAR_FEATURE_USB_PROTOCOL << 10U) | \
    ((uint32_t)AMBAR_FEATURE_EFFECTIVE_SIMULATION << 11U) | \
    ((uint32_t)AMBAR_FEATURE_USB_REQUIRE_VBUS_SENSE << 12U) | \
    ((uint32_t)AMBAR_FEATURE_EFFECTIVE_USB_FLASH_MAINTENANCE << 13U) | \
    ((uint32_t)AMBAR_FEATURE_PRESENTATION_MOTION << 14U) | \
    ((uint32_t)AMBAR_PRESENTATION_ACTUATOR_DIRECTION_INVERTED << 15U))

#define AMBAR_FEATURE_CONFIG_STATUS_FLAGS (AMBAR_FEATURE_ENABLED_BITS << 16U)

#endif /* AMBAR_FEATURES_H */
