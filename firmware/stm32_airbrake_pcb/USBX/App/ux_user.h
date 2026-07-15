/* USBX configuration for the AMBAR bare-metal CDC ACM device. */
#ifndef UX_USER_H
#define UX_USER_H

#define UX_DEVICE_SIDE_ONLY
#define UX_STANDALONE

/* CDC read_run/write_run are driven cooperatively from AmbarUsb_Task(). */
#define UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE
#define UX_DEVICE_CLASS_CDC_ACM_WRITE_AUTO_ZLP

/* Full-speed CDC endpoints use 64-byte packets. */
#define UX_SLAVE_REQUEST_DATA_MAX_LENGTH    64
/*
 * The CDC configuration descriptor is 75 bytes.  USBX stalls EP0 when the
 * requested descriptor is larger than this buffer, so keep the ST reference
 * value instead of limiting control transfers to one 64-byte packet.
 */
#define UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH 256
#define UX_PERIODIC_RATE                    1000

/* CDC uses endpoint number 1 in both directions. */
#define UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT

#endif /* UX_USER_H */
