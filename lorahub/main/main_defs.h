/*______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2024 Semtech

Description:
    LoRaHub Packet Forwarder Defs

License: Revised BSD License, see LICENSE.TXT file include in the project
*/

#ifndef _PKTFWD_DEFS_H
#define _PKTFWD_DEFS_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDENCIES --------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

typedef enum
{
    LRHB_ERROR_NONE,
    LRHB_ERROR_WIFI,
    LRHB_ERROR_LNS,
    LRHB_ERROR_OS,
    LRHB_ERROR_HAL,
    LRHB_ERROR_UNKNOWN,
} lorahub_error_t;

#endif  // _PKTFWD_DEFS_H

void set_user_led( bool on );
void wait_on_error( lorahub_error_t error, int line );

/* --- EOF ------------------------------------------------------------------ */