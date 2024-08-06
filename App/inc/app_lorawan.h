#ifndef __APP_LORAWAN_H
#define __APP_LORAWAN_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief LoRaWAN User credentials
 */
#define USER_LORAWAN_DEVICE_EUI                        \
    {                                                  \
        0x00, 0x80, 0xE1, 0x15, 0x00, 0x05, 0x33, 0xED \
    }
#define USER_LORAWAN_JOIN_EUI                          \
    {                                                  \
        0x75, 0x38, 0x90, 0x47, 0x70, 0x36, 0x66, 0x83 \
    }
#define USER_LORAWAN_GEN_APP_KEY                                                                       \
    {                                                                                                  \
        0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C \
    }
#define USER_LORAWAN_APP_KEY                                                                           \
    {                                                                                                  \
        0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C \
    }


/**
 * @brief Modem Region define
 */
#define MODEM_EXAMPLE_REGION SMTC_MODEM_REGION_EU_868

void main_porting_tests( void );

void main_periodical_uplink( void );

void main_lctt_certif(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_LORAWAN_H */
