
/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include <string.h>

#include "ral.h"
#include "ral_defs.h"
#include "smtc_modem_api.h"
#include "smtc_modem_utilities.h"

#include "smtc_modem_hal.h"
#include "smtc_hal_dbg_trace.h"

#include "app_lorawan.h"

#include "smtc_hal_mcu.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_watchdog.h"

#include "modem_pinout.h"


/**
 * @brief Returns the minimum value between a and b
 *
 * @param [in] a 1st value
 * @param [in] b 2nd value
 * @retval Minimum value
 */
#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

/*!
 * @brief Stringify constants
 */
#define xstr( a ) str( a )
#define str( a ) #a

/*!
 * @brief Helper macro that returned a human-friendly message if a command does not return SMTC_MODEM_RC_OK
 *
 * @remark The macro is implemented to be used with functions returning a @ref smtc_modem_return_code_t
 *
 * @param[in] rc  Return code
 */

#define ASSERT_SMTC_MODEM_RC( rc_func )                                                         \
    do                                                                                          \
    {                                                                                           \
        smtc_modem_return_code_t rc = rc_func;                                                  \
        if( rc == SMTC_MODEM_RC_NOT_INIT )                                                      \
        {                                                                                       \
            SMTC_HAL_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                  xstr( SMTC_MODEM_RC_NOT_INIT ) );                             \
        }                                                                                       \
        else if( rc == SMTC_MODEM_RC_INVALID )                                                  \
        {                                                                                       \
            SMTC_HAL_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                  xstr( SMTC_MODEM_RC_INVALID ) );                              \
        }                                                                                       \
        else if( rc == SMTC_MODEM_RC_BUSY )                                                     \
        {                                                                                       \
            SMTC_HAL_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                  xstr( SMTC_MODEM_RC_BUSY ) );                                 \
        }                                                                                       \
        else if( rc == SMTC_MODEM_RC_FAIL )                                                     \
        {                                                                                       \
            SMTC_HAL_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                  xstr( SMTC_MODEM_RC_FAIL ) );                                 \
        }                                                                                       \
        else if( rc == SMTC_MODEM_RC_NO_TIME )                                                  \
        {                                                                                       \
            SMTC_HAL_TRACE_WARNING( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__, \
                                    xstr( SMTC_MODEM_RC_NO_TIME ) );                            \
        }                                                                                       \
        else if( rc == SMTC_MODEM_RC_INVALID_STACK_ID )                                         \
        {                                                                                       \
            SMTC_HAL_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                  xstr( SMTC_MODEM_RC_INVALID_STACK_ID ) );                     \
        }                                                                                       \
        else if( rc == SMTC_MODEM_RC_NO_EVENT )                                                 \
        {                                                                                       \
            SMTC_HAL_TRACE_INFO( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,    \
                                 xstr( SMTC_MODEM_RC_NO_EVENT ) );                              \
        }                                                                                       \
    } while( 0 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * Stack id value (multistacks modem is not yet available)
 */
#define STACK_ID 0

/**
 * @brief Stack credentials
 */
#if !defined( USE_LR11XX_CREDENTIALS )
static const uint8_t user_dev_eui[8]      = USER_LORAWAN_DEVICE_EUI;
static const uint8_t user_join_eui[8]     = USER_LORAWAN_JOIN_EUI;
static const uint8_t user_gen_app_key[16] = USER_LORAWAN_GEN_APP_KEY;
static const uint8_t user_app_key[16]     = USER_LORAWAN_APP_KEY;
#endif

/**
 * @brief Watchdog counter reload value during sleep (The period must be lower than MCU watchdog period (here 32s))
 */
#define WATCHDOG_RELOAD_PERIOD_MS 20000

/**
 * @brief Periodical uplink alarm delay in seconds
 */
#define PERIODICAL_UPLINK_DELAY_S 10

#define DELAY_FIRST_MSG_AFTER_JOIN 10

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static uint8_t                  rx_payload[SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH] = { 0 };  // Buffer for rx payload
static uint8_t                  rx_payload_size = 0;      // Size of the payload in the rx_payload buffer
static smtc_modem_dl_metadata_t rx_metadata     = { 0 };  // Metadata of downlink
static uint8_t                  rx_remaining    = 0;      // Remaining downlink payload in modem

static volatile bool k1_button_is_press = false;  // Flag for button status
static volatile bool k2_button_is_press = false;  // Flag for button status
static uint32_t      uplink_counter       = 0;      // uplink raising counter

#if defined( USE_RELAY_TX )
static smtc_modem_relay_tx_config_t relay_config = { 0 };
#endif
/**
 * @brief Internal credentials
 */
#if defined( USE_LR11XX_CREDENTIALS )
static uint8_t chip_eui[SMTC_MODEM_EUI_LENGTH] = { 0 };
static uint8_t chip_pin[SMTC_MODEM_PIN_LENGTH] = { 0 };
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief User callback for modem event
 *
 *  This callback is called every time an event ( see smtc_modem_event_t ) appears in the modem.
 *  Several events may have to be read from the modem when this callback is called.
 */
static void modem_event_callback( void );


/**
 * @brief Send the 32bits uplink counter on chosen port
 */
static void send_uplink_counter_on_port( uint8_t port );

static void modem_event_callback( void )
{
    SMTC_HAL_TRACE_MSG_COLOR( "Modem event callback\n", HAL_DBG_TRACE_COLOR_BLUE );

    smtc_modem_event_t current_event;
    uint8_t            event_pending_count;
    uint8_t            stack_id = STACK_ID;

    // Continue to read modem event until all event has been processed
    do
    {
        // Read modem event
        ASSERT_SMTC_MODEM_RC( smtc_modem_get_event( &current_event, &event_pending_count ) );

        switch( current_event.event_type )
        {
        case SMTC_MODEM_EVENT_RESET:
            SMTC_HAL_TRACE_INFO( "Event received: RESET\n" );

#if !defined( USE_LR11XX_CREDENTIALS )
            // Set user credentials
            ASSERT_SMTC_MODEM_RC( smtc_modem_set_deveui( stack_id, user_dev_eui ) );
            ASSERT_SMTC_MODEM_RC( smtc_modem_set_joineui( stack_id, user_join_eui ) );
            ASSERT_SMTC_MODEM_RC( smtc_modem_set_appkey( stack_id, user_gen_app_key ) );
            ASSERT_SMTC_MODEM_RC( smtc_modem_set_nwkkey( stack_id, user_app_key ) );
#else
            // Get internal credentials
            ASSERT_SMTC_MODEM_RC( smtc_modem_get_chip_eui( stack_id, chip_eui ) );
            SMTC_HAL_TRACE_ARRAY( "CHIP_EUI", chip_eui, SMTC_MODEM_EUI_LENGTH );
            ASSERT_SMTC_MODEM_RC( smtc_modem_get_pin( stack_id, chip_pin ) );
            SMTC_HAL_TRACE_ARRAY( "CHIP_PIN", chip_pin, SMTC_MODEM_PIN_LENGTH );
#endif
            // Set user region
            ASSERT_SMTC_MODEM_RC( smtc_modem_set_region( stack_id, MODEM_EXAMPLE_REGION ) );
// Schedule a Join LoRaWAN network
#if defined( USE_RELAY_TX )
            // by default when relay mode is activated , CSMA is also activated by default to at least protect the WOR
            // transmission
            // if you want to disable the csma please uncomment the next line
            // ASSERT_SMTC_MODEM_RC(smtc_modem_csma_set_state (stack_id,false));

            relay_config.second_ch_enable = false;

            // The RelayModeActivation field indicates how the end-device SHOULD manage the relay mode.
            relay_config.activation = SMTC_MODEM_RELAY_TX_ACTIVATION_MODE_DYNAMIC;

            // number_of_miss_wor_ack_to_switch_in_nosync_mode  field indicates that the
            // relay mode SHALL be restart in no sync mode when it does not receive a WOR ACK frame after
            // number_of_miss_wor_ack_to_switch_in_nosync_mode consecutive uplinks.
            relay_config.number_of_miss_wor_ack_to_switch_in_nosync_mode = 3;

            // smart_level field indicates that the
            // relay mode SHALL be enabled if the end-device does not receive a valid downlink after smart_level
            // consecutive uplinks.
            relay_config.smart_level = 8;

            // The BackOff field indicates how the end-device SHALL behave when it does not receive
            // a WOR ACK frame.
            // BackOff Description
            // 0 Always send a LoRaWAN uplink
            // 1..63 Send a LoRaWAN uplink after X WOR frames without a WOR ACK
            relay_config.backoff = 4;
            ASSERT_SMTC_MODEM_RC( smtc_modem_relay_tx_enable( stack_id, &relay_config ) );
#endif
            ASSERT_SMTC_MODEM_RC( smtc_modem_join_network( stack_id ) );
            break;

        case SMTC_MODEM_EVENT_ALARM:
            SMTC_HAL_TRACE_INFO( "Event received: ALARM\n" );
            // Send periodical uplink on port 101
            send_uplink_counter_on_port( 101 );
            // Restart periodical uplink alarm
            ASSERT_SMTC_MODEM_RC( smtc_modem_alarm_start_timer( PERIODICAL_UPLINK_DELAY_S ) );
            break;

        case SMTC_MODEM_EVENT_JOINED:
            SMTC_HAL_TRACE_INFO( "Event received: JOINED\n" );
            SMTC_HAL_TRACE_INFO( "Modem is now joined \n" );

            // Send first periodical uplink on port 101
            send_uplink_counter_on_port( 101 );
            // start periodical uplink alarm
            ASSERT_SMTC_MODEM_RC( smtc_modem_alarm_start_timer( DELAY_FIRST_MSG_AFTER_JOIN ) );
            break;

        case SMTC_MODEM_EVENT_TXDONE:
            SMTC_HAL_TRACE_INFO( "Event received: TXDONE\n" );
            SMTC_HAL_TRACE_INFO( "Transmission done \n" );
            break;

        case SMTC_MODEM_EVENT_DOWNDATA:
            SMTC_HAL_TRACE_INFO( "Event received: DOWNDATA\n" );
            // Get downlink data
            ASSERT_SMTC_MODEM_RC(
                smtc_modem_get_downlink_data( rx_payload, &rx_payload_size, &rx_metadata, &rx_remaining ) );
            SMTC_HAL_TRACE_PRINTF( "Data received on port %u\n", rx_metadata.fport );
            SMTC_HAL_TRACE_ARRAY( "Received payload", rx_payload, rx_payload_size );
            break;

        case SMTC_MODEM_EVENT_JOINFAIL:
            SMTC_HAL_TRACE_INFO( "Event received: JOINFAIL\n" );
            break;

        case SMTC_MODEM_EVENT_ALCSYNC_TIME:
            SMTC_HAL_TRACE_INFO( "Event received: ALCSync service TIME\n" );
            break;

        case SMTC_MODEM_EVENT_LINK_CHECK:
            SMTC_HAL_TRACE_INFO( "Event received: LINK_CHECK\n" );
            break;

        case SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO:
            SMTC_HAL_TRACE_INFO( "Event received: CLASS_B_PING_SLOT_INFO\n" );
            break;

        case SMTC_MODEM_EVENT_CLASS_B_STATUS:
            SMTC_HAL_TRACE_INFO( "Event received: CLASS_B_STATUS\n" );
            break;

        case SMTC_MODEM_EVENT_LORAWAN_MAC_TIME:
            SMTC_HAL_TRACE_WARNING( "Event received: LORAWAN MAC TIME\n" );
            break;

        case SMTC_MODEM_EVENT_LORAWAN_FUOTA_DONE:
        {
            bool status = current_event.event_data.fuota_status.successful;
            if( status == true )
            {
                SMTC_HAL_TRACE_INFO( "Event received: FUOTA SUCCESSFUL\n" );
            }
            else
            {
                SMTC_HAL_TRACE_WARNING( "Event received: FUOTA FAIL\n" );
            }
            break;
        }

        case SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_C:
            SMTC_HAL_TRACE_INFO( "Event received: MULTICAST CLASS_C STOP\n" );
            break;

        case SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_B:
            SMTC_HAL_TRACE_INFO( "Event received: MULTICAST CLASS_B STOP\n" );
            break;

        case SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_C:
            SMTC_HAL_TRACE_INFO( "Event received: New MULTICAST CLASS_C \n" );
            break;

        case SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_B:
            SMTC_HAL_TRACE_INFO( "Event received: New MULTICAST CLASS_B\n" );
            break;

        case SMTC_MODEM_EVENT_FIRMWARE_MANAGEMENT:
            SMTC_HAL_TRACE_INFO( "Event received: FIRMWARE_MANAGEMENT\n" );
            if( current_event.event_data.fmp.status == SMTC_MODEM_EVENT_FMP_REBOOT_IMMEDIATELY )
            {
                smtc_modem_hal_reset_mcu( );
            }
            break;

        case SMTC_MODEM_EVENT_STREAM_DONE:
            SMTC_HAL_TRACE_INFO( "Event received: STREAM_DONE\n" );
            break;

        case SMTC_MODEM_EVENT_UPLOAD_DONE:
            SMTC_HAL_TRACE_INFO( "Event received: UPLOAD_DONE\n" );
            break;

        case SMTC_MODEM_EVENT_DM_SET_CONF:
            SMTC_HAL_TRACE_INFO( "Event received: DM_SET_CONF\n" );
            break;

        case SMTC_MODEM_EVENT_MUTE:
            SMTC_HAL_TRACE_INFO( "Event received: MUTE\n" );
            break;
        case SMTC_MODEM_EVENT_RELAY_TX_DYNAMIC:  //!< Relay TX dynamic mode has enable or disable the WOR protocol
            SMTC_HAL_TRACE_INFO( "Event received: RELAY_TX_DYNAMIC\n" );
            break;
        case SMTC_MODEM_EVENT_RELAY_TX_MODE:  //!< Relay TX activation has been updated
            SMTC_HAL_TRACE_INFO( "Event received: RELAY_TX_MODE\n" );
            break;
        case SMTC_MODEM_EVENT_RELAY_TX_SYNC:  //!< Relay TX synchronisation has changed
            SMTC_HAL_TRACE_INFO( "Event received: RELAY_TX_SYNC\n" );
            break;
        default:
            SMTC_HAL_TRACE_ERROR( "Unknown event %u\n", current_event.event_type );
            break;
        }
    } while( event_pending_count > 0 );
}

void k1_button_callback(void * context)
{
    SMTC_HAL_TRACE_INFO( "K1 Button pushed\n" );

    ( void ) context;  // Not used in the example - avoid warning

    static uint32_t last_press_timestamp_ms = 0;

    // Debounce the button press, avoid multiple triggers
    if( ( int32_t ) ( smtc_modem_hal_get_time_in_ms( ) - last_press_timestamp_ms ) > 500 )
    {
        last_press_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
        k1_button_is_press    = true;
    }
}

void k2_button_callback(void * context)
{
    SMTC_HAL_TRACE_INFO( "K2 Button pushed\n" );

    ( void ) context;  // Not used in the example - avoid warning

    static uint32_t last_press_timestamp_ms = 0;

    // Debounce the button press, avoid multiple triggers
    if( ( int32_t ) ( smtc_modem_hal_get_time_in_ms( ) - last_press_timestamp_ms ) > 500 )
    {
        last_press_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
        k2_button_is_press    = true;
    }
}

static void send_uplink_counter_on_port( uint8_t port )
{
    // Send uplink counter on port 102
    uint8_t buff[4] = { 0 };
    buff[0]         = ( uplink_counter >> 24 ) & 0xFF;
    buff[1]         = ( uplink_counter >> 16 ) & 0xFF;
    buff[2]         = ( uplink_counter >> 8 ) & 0xFF;
    buff[3]         = ( uplink_counter & 0xFF );
    ASSERT_SMTC_MODEM_RC( smtc_modem_request_uplink( STACK_ID, port, false, buff, 4 ) );
    // Increment uplink counter
    uplink_counter++;
}

#if defined( SX128X )
#include "ralf_sx128x.h"
#elif defined( SX126X )
#include "ralf_sx126x.h"
#include "sx126x.h"
#elif defined( LR11XX )
#include "ralf_lr11xx.h"
#include "lr11xx_system.h"
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

// !! SHOULD BE DEFINED BY USER !!
#define ENABLE_TEST_FLASH 0  // Enable flash porting test BUT disable other porting tests

// Delay introduced by HAL_LPTIM_TimeOut_Start_IT function of stm32l4xx_hal_lptim.c file
#define BOARD_COMPENSATION_IN_MS 6

#define NB_LOOP_TEST_SPI 2
#define NB_LOOP_TEST_CONFIG_RADIO 2

#if defined( LR1110 )
#define LR11XX_FW_VERSION 0x0401
#elif defined( LR1120 )
#define LR11XX_FW_VERSION 0x0201
#elif defined( LR1121 )
#define LR11XX_FW_VERSION 0x0103
#endif

#define FREQ_NO_RADIO 910300000
#define SYNC_WORD_NO_RADIO 0x21

#define MARGIN_GET_TIME_IN_MS 1
#define MARGIN_TIMER_IRQ_IN_MS 2
#define MARGIN_TIME_CONFIG_RADIO_IN_MS 8
#define MARGIN_SLEEP_IN_MS 2

#define PORTING_TEST_MSG_OK( )                                \
    do                                                        \
    {                                                         \
        SMTC_HAL_TRACE_PRINTF( HAL_DBG_TRACE_COLOR_GREEN );   \
        SMTC_HAL_TRACE_PRINTF( " OK \n" );                    \
        SMTC_HAL_TRACE_PRINTF( HAL_DBG_TRACE_COLOR_DEFAULT ); \
    } while( 0 );

#define PORTING_TEST_MSG_WARN( ... )                          \
    do                                                        \
    {                                                         \
        SMTC_HAL_TRACE_PRINTF( HAL_DBG_TRACE_COLOR_YELLOW );  \
        SMTC_HAL_TRACE_PRINTF( __VA_ARGS__ );                 \
        SMTC_HAL_TRACE_PRINTF( HAL_DBG_TRACE_COLOR_DEFAULT ); \
    } while( 0 );

#define PORTING_TEST_MSG_NOK( ... )                           \
    do                                                        \
    {                                                         \
        SMTC_HAL_TRACE_PRINTF( HAL_DBG_TRACE_COLOR_RED );     \
        SMTC_HAL_TRACE_PRINTF( "\n NOK:" );                   \
        SMTC_HAL_TRACE_PRINTF( __VA_ARGS__ );                 \
        SMTC_HAL_TRACE_PRINTF( HAL_DBG_TRACE_COLOR_DEFAULT ); \
    } while( 0 );

#if defined( SX128X )
const ralf_t modem_radio = RALF_SX128X_INSTANTIATE( NULL );
#elif defined( SX126X )
static const ralf_t modem_radio = RALF_SX126X_INSTANTIATE( NULL );
#elif defined( LR11XX )
const ralf_t modem_radio = RALF_LR11XX_INSTANTIATE( NULL );
#else
#error "Please select radio board.."
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */
/**
 * @brief Return test enumeration
 */
typedef enum return_code_test_e
{
    RC_PORTING_TEST_OK       = 0x00,  // Test OK
    RC_PORTING_TEST_NOK      = 0x01,  // Test NOK
    RC_PORTING_TEST_RELAUNCH = 0x02,  // Relaunch test
} return_code_test_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static volatile bool     radio_irq_raised      = false;
static volatile bool     irq_rx_timeout_raised = false;
static volatile bool     timer_irq_raised      = false;
static volatile uint32_t irq_time_ms           = 0;
static volatile uint32_t irq_time_s            = 0;

/**************************************************************************************************
################################## APP PORTING TEST ###############################################
***************************************************************************************************/

// LoRa configurations TO NOT receive or transmit
static ralf_params_lora_t rx_lora_param = { .sync_word                       = SYNC_WORD_NO_RADIO,
                                            .symb_nb_timeout                 = 0,
                                            .rf_freq_in_hz                   = FREQ_NO_RADIO,
                                            .mod_params.cr                   = RAL_LORA_CR_4_5,
                                            .mod_params.sf                   = RAL_LORA_SF10,
                                            .mod_params.bw                   = RAL_LORA_BW_250_KHZ,
                                            .mod_params.ldro                 = 0,
                                            .pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT,
                                            .pkt_params.pld_len_in_bytes     = 255,
                                            .pkt_params.crc_is_on            = false,
                                            .pkt_params.invert_iq_is_on      = true,
                                            .pkt_params.preamble_len_in_symb = 8 };

static ralf_params_lora_t tx_lora_param = { .sync_word                       = SYNC_WORD_NO_RADIO,
                                            .symb_nb_timeout                 = 0,
                                            .rf_freq_in_hz                   = FREQ_NO_RADIO,
                                            .output_pwr_in_dbm               = 14,
                                            .mod_params.cr                   = RAL_LORA_CR_4_5,
                                            .mod_params.sf                   = RAL_LORA_SF12,
                                            .mod_params.bw                   = RAL_LORA_BW_125_KHZ,
                                            .mod_params.ldro                 = 0,
                                            .pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT,
                                            .pkt_params.pld_len_in_bytes     = 50,
                                            .pkt_params.crc_is_on            = true,
                                            .pkt_params.invert_iq_is_on      = false,
                                            .pkt_params.preamble_len_in_symb = 8 };
#if( ENABLE_TEST_FLASH != 0 )
static const char* name_context_type[] = { "MODEM", "LR1MAC", "DEVNONCE", "SECURE_ELEMENT" };
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static void radio_tx_irq_callback( void* obj );
static void radio_rx_irq_callback( void* obj );
static void radio_irq_callback_get_time_in_s( void* obj );
static void timer_irq_callback( void* obj );

static bool               reset_init_radio( void );
static return_code_test_t test_get_time_in_s( void );
static return_code_test_t test_get_time_in_ms( void );

static bool porting_test_spi( void );
static bool porting_test_radio_irq( void );
static bool porting_test_get_time( void );
static bool porting_test_timer_irq( void );
static bool porting_test_stop_timer( void );
static bool porting_test_disable_enable_irq( void );
static bool porting_test_random( void );
static bool porting_test_config_rx_radio( void );
static bool porting_test_config_tx_radio( void );
static bool porting_test_sleep_ms( void );
static bool porting_test_timer_irq_low_power( void );
#if( ENABLE_TEST_FLASH != 0 )
static bool test_context_store_restore( modem_context_type_t context_type );
static bool porting_test_flash( void );
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Example to test functionality to be porting by the user according to the MCU hardware
 *
 */
void main_porting_tests( void )
{
    bool ret = true;

    // Configure all the µC periph (clock, gpio, timer, ...)
    hal_mcu_init( );

    // Tests
    SMTC_HAL_TRACE_MSG( "\n\n\nPORTING_TEST example is starting \n\n" );

#if( ENABLE_TEST_FLASH == 0 )

    ret = porting_test_spi( );
    if( ret == false )
        return;

    ret = porting_test_radio_irq( );
    if( ret == false )
        return;

    ret = porting_test_get_time( );
    if( ret == false )
        return;

    ret = porting_test_timer_irq( );
    if( ret == false )
        return;

    porting_test_stop_timer( );

    porting_test_disable_enable_irq( );

    porting_test_random( );

    porting_test_config_rx_radio( );

    porting_test_config_tx_radio( );

    porting_test_sleep_ms( );

    porting_test_timer_irq_low_power( );

#else

    ret = porting_test_flash( );
    if( ret == false )
        return ret;

    SMTC_HAL_TRACE_MSG_COLOR( "\n MCU RESET => relaunch tests and check if read after reset = write before reset \n\n",
                              HAL_DBG_TRACE_COLOR_BLUE );

    hal_mcu_set_sleep_for_ms( 2000 );

    hal_mcu_reset( );

#endif

    SMTC_HAL_TRACE_MSG( "----------------------------------------\nEND \n\n" );

    while( 1 )
    {
        hal_watchdog_reload( );
    }

    return;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/**
 * @brief Test SPI
 *
 * @remark
 * Prerequisite:
 * Radio reset should be implemented:
 * - drive of gpio (hal_gpio_set_value)
 * - mcu wait us (hal_mcu_wait_us)
 *
 * Test processing:
 * - Reset radio
 * - Read data through SPI
 * - Check if data is coherent
 *
 * Ported functions:
 * lr11xx_hal_read
 *      lr11xx_hal_check_device_ready
 *          lr11xx_hal_wait_on_busy
 *              hal_gpio_get_value
 *          hal_gpio_set_value
 *          hal_spi_in_out
 *
 * @return bool True if test is successful
 */
static bool porting_test_spi( void )
{
    SMTC_HAL_TRACE_MSG( "----------------------------------------\n porting_test_spi : " );

    uint16_t counter_nok = 0;

    // Reset radio (prerequisite)
    ral_reset( &( modem_radio.ral ) );

    for( uint16_t i = 0; i < NB_LOOP_TEST_SPI; i++ )
    {
#if defined( LR11XX )
        lr11xx_system_version_t version;
        lr11xx_status_t         status;

        status = lr11xx_system_get_version( NULL, &version );

        if( status == LR11XX_STATUS_OK )
        {
            if( version.fw == LR11XX_FW_VERSION )
            {
                SMTC_HAL_TRACE_PRINTF( " LR11XX firmware version is 0x%04X \n", version.fw );
                break;
            }
            else
            {
                PORTING_TEST_MSG_NOK( " Wrong LR11XX firmware version: expected 0x%04X / get 0x%04X \n",
                                      LR11XX_FW_VERSION, version.fw );
                counter_nok++;
            }
        }
        else
        {
            PORTING_TEST_MSG_NOK( " Failed to get LR11XX firmware version \n" );
            counter_nok++;
        }

#elif defined( SX126X )
        sx126x_chip_status_t chip_status;
        sx126x_status_t      status;

        status = sx126x_get_status( NULL, &chip_status );

        if( status == SX126X_STATUS_OK )
        {
            if( chip_status.chip_mode == SX126X_CHIP_MODE_UNUSED )
            {
                PORTING_TEST_MSG_NOK( " Wrong SX126X chip mode, get SX126X_CHIP_MODE_UNUSED \n" );
                counter_nok++;
            }
        }
        else
        {
            PORTING_TEST_MSG_NOK( " Failed to get SX126X status \n" );
            counter_nok++;
        }
#else
        PORTING_TEST_MSG_NOK( " Radio is not supported \n" );
        return false;
#endif
    }

    if( counter_nok == 0 )
    {
        PORTING_TEST_MSG_OK( );
    }
    else
    {
        PORTING_TEST_MSG_WARN( " Failed test = %u / %u \n", counter_nok, NB_LOOP_TEST_SPI );
        return false;
    }

    return true;
}

/**
 * @brief Reset and init radio
 *
 * @remark
 * Test processing:
 * - Reset radio
 * - Init radio
 * - Set radio in sleep mode
 *
 * Ported functions:
 * ral_reset:
 * lr11xx_hal_reset
 *     hal_gpio_set_value
 *     hal_mcu_wait_us
 * ral_init:
 * ral_lr11xx_init
 *     ral_lr11xx_bsp_get_crc_state
 *     ral_lr11xx_bsp_get_reg_mode
 *     ral_lr11xx_bsp_get_rf_switch_cfg
 *     ral_lr11xx_bsp_get_xosc_cfg
 *     lr11xx_system_set_tcxo_mode
 *         lr11xx_hal_write
 *     lr11xx_system_calibrate
 *         lr11xx_hal_write
 * ral_set_sleep:
 * ral_lr11xx_set_sleep
 *     lr11xx_system_set_sleep
 *         lr11xx_hal_write
 *
 * @return bool True if test is successful
 */
static bool reset_init_radio( void )
{
    ral_status_t status = RAL_STATUS_ERROR;

    // Reset, init radio and put it in sleep mode
    ral_reset( &( modem_radio.ral ) );

    status = ral_init( &( modem_radio.ral ) );
    if( status != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_init() function failed \n" );
        return false;
    }

    status = ral_set_sleep( &( modem_radio.ral ), true );
    smtc_modem_hal_set_ant_switch( true );
    if( status != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_set_sleep() function failed \n" );
        return false;
    }

    return true;
}

/**
 * @brief Test radio irq
 *
 * @remark
 * Test processing:
 * - Reset and init radio
 * - Configure radio irq
 * - Configure radio with bad parameters to receive a rx timeout irq
 * - Configure radio in reception mode with a timeout
 * - Wait
 * - Check if rx timeout irq is raised
 *
 * Ported functions:
 * smtc_modem_hal_irq_config_radio_irq:
 *     hal_gpio_irq_attach
 *
 * lr11xx_hal_write
 *     lr11xx_hal_check_device_ready
 *         lr11xx_hal_wait_on_busy
 *         hal_gpio_get_value
 *     hal_gpio_set_value
 *     hal_spi_in_out
 *     hal_gpio_set_value
 *     hal_mcu_wait_us
 *
 * @return bool True if test is successful
 */
static bool porting_test_radio_irq( void )
{
    SMTC_HAL_TRACE_MSG( "----------------------------------------\n porting_test_radio_irq : " );

    bool     ret              = true;
    uint32_t rx_timeout_in_ms = 500;
    radio_irq_raised          = false;

    // Reset, init radio and put it in sleep mode
    ret = reset_init_radio( );
    if( ret == false )
        return ret;

    // Setup radio and relative irq
    smtc_modem_hal_irq_config_radio_irq( radio_rx_irq_callback, NULL );

    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_set_ant_switch( false );
    if( ral_set_dio_irq_params( &( modem_radio.ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
                                                          RAL_IRQ_RX_CRC_ERROR ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_set_dio_irq_params() function failed \n" );
        return false;
    }

    if( ralf_setup_lora( &modem_radio, &rx_lora_param ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ralf_setup_lora() function failed \n" );
        return false;
    }

    // Configure radio in reception mode
    if( ral_set_rx( &( modem_radio.ral ), rx_timeout_in_ms ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_set_rx() function failed \n" );
        return false;
    }

    // Wait 2 * timeout
    hal_mcu_wait_us( ( rx_timeout_in_ms * 4 ) * 1000 );

    if( radio_irq_raised == true )
    {
        PORTING_TEST_MSG_OK( );
    }
    else
    {
        PORTING_TEST_MSG_NOK( " Timeout, radio irq not received \n" );
        return false;
    }
    return true;
}

/**
 * @brief Test get time in s
 *
 *
 * @remark
 *  Test processing:
 * - Reset, init and configure radio
 * - Configure radio in reception mode with a timeout
 * - Get start time
 * - Wait radio irq (get stop time in irq callback)
 * - Check if time is coherent with the configured timeout radio irq
 * Note: if radio irq received different of rx timeout irq -> relaunch test
 *
 * Ported functions:
 * smtc_modem_hal_get_time_in_s
 *      hal_rtc_get_time_s
 *
 * @return return_code_test_t   RC_PORTING_TEST_OK
 *                              RC_PORTING_TEST_NOK
 *                              RC_PORTING_TEST_RELAUNCH
 */
static return_code_test_t test_get_time_in_s( void )
{
    SMTC_HAL_TRACE_MSG( " * Get time in second: " );

    bool     ret              = true;
    uint32_t rx_timeout_in_ms = 5000;

    radio_irq_raised      = false;
    irq_rx_timeout_raised = false;

    rx_lora_param.symb_nb_timeout = 0;

    // Reset, init radio and put it in sleep mode
    ret = reset_init_radio( );
    if( ret == false )
        return RC_PORTING_TEST_NOK;

    // Setup radio and relative irq
    smtc_modem_hal_irq_config_radio_irq( radio_irq_callback_get_time_in_s, NULL );

    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_set_ant_switch( true );
    if( ralf_setup_lora( &modem_radio, &rx_lora_param ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ralf_setup_lora() function failed \n" );
        return RC_PORTING_TEST_NOK;
    }
    if( ral_set_dio_irq_params( &( modem_radio.ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
                                                          RAL_IRQ_RX_CRC_ERROR ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_set_dio_irq_params() function failed \n" );
        return RC_PORTING_TEST_NOK;
    }

    // Configure radio in reception mode
    if( ral_set_rx( &( modem_radio.ral ), rx_timeout_in_ms ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_set_rx() function failed \n" );
        return RC_PORTING_TEST_NOK;
    }
    uint32_t start_time_s = smtc_modem_hal_get_time_in_s( );

    while( radio_irq_raised == false )
    {
        // Do nothing
    }

    if( irq_rx_timeout_raised == false )
    {
        PORTING_TEST_MSG_WARN( "\n Radio irq received but not RAL_IRQ_RX_TIMEOUT -> relaunched test \n " );
        return RC_PORTING_TEST_RELAUNCH;
    }

    uint32_t time = irq_time_s - start_time_s;
    if( time == ( rx_timeout_in_ms / 1000 ) )
    {
        PORTING_TEST_MSG_OK( );
        SMTC_HAL_TRACE_PRINTF( " Time expected %us / get %us (no margin) \n", ( rx_timeout_in_ms / 1000 ), time );
    }
    else
    {
        PORTING_TEST_MSG_NOK( " Time is not coherent: expected %us / get %us (no margin) \n",
                              ( rx_timeout_in_ms / 1000 ), time );
        return RC_PORTING_TEST_NOK;
    }

    return RC_PORTING_TEST_OK;
}

/**
 * @brief Test get time in ms
 *
 *
 * @remark
 *  Test processing:
 * - Reset, init and configure radio (with a timeout symbol number)
 * - Get start time
 * - Configure radio in reception mode
 * - Wait radio irq (get stop time in irq callback)
 * - Check if time is coherent with the configured timeout symbol number
 * Note: if radio irq received different of rx timeout irq -> relaunch test
 *
 * Ported functions:
 * smtc_modem_hal_get_time_in_ms
 *      hal_rtc_get_time_ms
 *
 * @return return_code_test_t   RC_PORTING_TEST_OK
 *                              RC_PORTING_TEST_NOK
 *                              RC_PORTING_TEST_RELAUNCH
 */
static return_code_test_t test_get_time_in_ms( void )
{
    SMTC_HAL_TRACE_MSG( " * Get time in millisecond: " );

    bool ret              = true;
    radio_irq_raised      = false;
    irq_rx_timeout_raised = false;
    uint8_t wait_start_ms = 5;

    // To avoid misalignment between symb timeout and real timeout for all radio, a number of symbols smaller than 63 is
    // to be used.
    rx_lora_param.symb_nb_timeout = 62;
    rx_lora_param.mod_params.sf   = RAL_LORA_SF12;
    rx_lora_param.mod_params.bw   = RAL_LORA_BW_125_KHZ;

    // Warning: to be updated if previous parameters (SF and BW) are changed
    uint32_t symb_time_ms =
        ( uint32_t ) ( rx_lora_param.symb_nb_timeout * ( ( 1 << 12 ) / 125.0 ) );  // 2^(SF) / BW * symb_nb_timeout

    // Reset, init radio and put it in sleep mode
    ret = reset_init_radio( );
    if( ret == false )
        return RC_PORTING_TEST_NOK;

    // Setup radio and relative irq
    smtc_modem_hal_irq_config_radio_irq( radio_rx_irq_callback, NULL );

    smtc_modem_hal_start_radio_tcxo( );
    smtc_modem_hal_set_ant_switch( false );
    if( ralf_setup_lora( &modem_radio, &rx_lora_param ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ralf_setup_lora() function failed \n" );
        return RC_PORTING_TEST_NOK;
    }
    if( ral_set_dio_irq_params( &( modem_radio.ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
                                                          RAL_IRQ_RX_CRC_ERROR ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_set_dio_irq_params() function failed \n" );
        return RC_PORTING_TEST_NOK;
    }

    // Wait 5ms to start
    uint32_t start_time_ms = smtc_modem_hal_get_time_in_ms( ) + wait_start_ms;
    while( smtc_modem_hal_get_time_in_ms( ) < start_time_ms )
    {
        // Do nothing
    }

    // Configure radio in reception mode
    if( ral_set_rx( &( modem_radio.ral ), 0 ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_set_rx() function failed \n" );
        return RC_PORTING_TEST_NOK;
    }

    while( radio_irq_raised == false )
    {
        // Do nothing
    }

    if( irq_rx_timeout_raised == false )
    {
        PORTING_TEST_MSG_WARN( "\n Radio irq received but not RAL_IRQ_RX_TIMEOUT -> relaunched test \n" );
        return RC_PORTING_TEST_RELAUNCH;
    }

    uint32_t time = irq_time_ms - start_time_ms - smtc_modem_hal_get_radio_tcxo_startup_delay_ms( );
    if( abs( time - symb_time_ms ) <= MARGIN_GET_TIME_IN_MS )
    {
        PORTING_TEST_MSG_OK( );
        SMTC_HAL_TRACE_PRINTF( " Time expected %ums / get %ums (margin +/-%ums) \n", ( uint32_t ) symb_time_ms, time,
                               MARGIN_GET_TIME_IN_MS );
    }
    else
    {
        PORTING_TEST_MSG_NOK( " Time is not coherent with radio irq : expected %ums / get %ums (margin +/-%ums) \n",
                              ( uint32_t ) symb_time_ms, time, MARGIN_GET_TIME_IN_MS );
        return RC_PORTING_TEST_NOK;
    }

    return RC_PORTING_TEST_OK;
}

/**
 * @brief Test time (Get time in s and in ms)
 *
 * @remark See test_get_time_in_s() and test_get_time_in_ms() functions
 *
 * @return bool True if test is successful
 */
static bool porting_test_get_time( void )
{
    SMTC_HAL_TRACE_MSG( "----------------------------------------\n porting_test_get_time : \n" );

    return_code_test_t ret = RC_PORTING_TEST_OK;

    do
    {
        ret = test_get_time_in_s( );
        if( ret == RC_PORTING_TEST_NOK )
            return false;
    } while( ret == RC_PORTING_TEST_RELAUNCH );

    do
    {
        ret = test_get_time_in_ms( );
        if( ret == RC_PORTING_TEST_NOK )
            return false;
    } while( ret == RC_PORTING_TEST_RELAUNCH );

    return true;
}

/**
 * @brief Test timer IRQ
 *
 * @warning smtc_modem_hal_start_timer() function takes ~4ms for STM32L4 (see HAL_LPTIM_TimeOut_Start_IT())
 *
 * @remark
 * Test processing:
 * - Get start time
 * - Configure and start timer
 * - Wait timer irq (get stop time in irq callback)
 * - Check the time elapsed between timer start and timer IRQ reception
 *
 * Ported functions:
 * smtc_modem_hal_start_timer
 *      hal_lp_timer_start
 *
 * @return bool True if test is successful
 */
static bool porting_test_timer_irq( void )
{
    SMTC_HAL_TRACE_MSG( "----------------------------------------\n porting_test_timer_irq : " );

    uint32_t timer_ms      = 3000;
    uint8_t  wait_start_ms = 5;
    uint16_t timeout_ms    = 2000;
    timer_irq_raised       = false;

    smtc_modem_hal_stop_timer( );

    // Wait 5ms to start
    uint32_t start_time_ms = smtc_modem_hal_get_time_in_ms( ) + wait_start_ms;

    while( smtc_modem_hal_get_time_in_ms( ) < start_time_ms )
    {
        // Do nothing
    }

    // FIXME: lptime inaccuracy
    smtc_modem_hal_start_timer( timer_ms, timer_irq_callback,
                                NULL );  // Warning this function takes ~3,69 ms for STM32L4

    // Timeout if irq not raised
    while( ( timer_irq_raised == false ) &&
           ( ( smtc_modem_hal_get_time_in_ms( ) - start_time_ms ) < ( timer_ms + timeout_ms ) ) )
    {
        // Do nothing
    }

    if( timer_irq_raised == false )
    {
        PORTING_TEST_MSG_NOK( " Timeout: timer irq not received \n" );
        return false;
    }

    uint32_t time = irq_time_ms - start_time_ms - BOARD_COMPENSATION_IN_MS;

    if( ( time >= timer_ms ) && ( time <= timer_ms + MARGIN_TIMER_IRQ_IN_MS ) )
    {
        PORTING_TEST_MSG_OK( );
        SMTC_HAL_TRACE_PRINTF( " Timer irq configured with %ums / get %ums (margin +%ums) \n", timer_ms, time,
                               MARGIN_TIMER_IRQ_IN_MS );
    }
    else
    {
        PORTING_TEST_MSG_NOK( " Timer irq delay is not coherent: expected %ums / get %ums (margin +%ums) \n", timer_ms,
                              time, MARGIN_TIMER_IRQ_IN_MS );
        return false;
    }
    return true;
}

/**
 * @brief Test stop timer
 *
 * @remark
 * Test processing:
 * - Configure and start timer
 * - Wait
 * - Stop timer
 * - Wait the end of timer
 * - Check if timer IRQ is not received
 *
 * Ported functions:
 * smtc_modem_hal_stop_timer
 *      hal_lp_timer_stop
 *
 * @return bool True if test is successful
 */
static bool porting_test_stop_timer( void )
{
    SMTC_HAL_TRACE_MSG( "----------------------------------------\n porting_test_stop_timer : " );

    uint32_t timer_ms = 1000;
    timer_irq_raised  = false;

    smtc_modem_hal_start_timer( timer_ms, timer_irq_callback, NULL );

    // Wait half of timer
    uint32_t time = smtc_modem_hal_get_time_in_ms( );
    while( ( smtc_modem_hal_get_time_in_ms( ) - time ) < ( timer_ms / 2 ) )
    {
        // Do nothing
    }

    smtc_modem_hal_stop_timer( );

    // Wait a little more than the end of timer
    time = smtc_modem_hal_get_time_in_ms( );
    while( ( smtc_modem_hal_get_time_in_ms( ) - time ) < ( timer_ms + 500 ) )
    {
        // Do nothing
    }

    if( timer_irq_raised == false )
    {
        PORTING_TEST_MSG_OK( );
    }
    else
    {
        PORTING_TEST_MSG_NOK( " Timer irq raised while timer is stopped \n" );
        return false;
    }
    return true;
}

/**
 * @brief Test enable/disable irq
 *
 * @remark
 * Test processing:
 * - Disable irq
 * - Start timer with irq
 * - Wait the end of timer
 * - Check if timer irq is not raised
 * - Enable irq
 * - Check if timer irq is raised
 *
 * Ported functions:
 * smtc_modem_hal_disable_modem_irq
 * smtc_modem_hal_enable_modem_irq
 *
 * @return bool True if test is successful
 */
static bool porting_test_disable_enable_irq( void )
{
    SMTC_HAL_TRACE_MSG( "----------------------------------------\n porting_test_disable_enable_irq : " );

    uint32_t timer_ms = 3000;
    timer_irq_raised  = false;

    smtc_modem_hal_disable_modem_irq( );

    smtc_modem_hal_start_timer( timer_ms, timer_irq_callback, NULL );

    // Wait a little more than the end of timer
    uint32_t time = smtc_modem_hal_get_time_in_ms( );
    while( ( smtc_modem_hal_get_time_in_ms( ) - time ) < ( timer_ms + 1000 ) )
    {
        // Do nothing
    }

    if( timer_irq_raised == true )
    {
        PORTING_TEST_MSG_NOK( " Timer irq raised while irq is disabled\n" );
        return false;
    }

    smtc_modem_hal_enable_modem_irq( );

    if( timer_irq_raised == true )
    {
        PORTING_TEST_MSG_OK( );
    }
    else
    {
        PORTING_TEST_MSG_NOK( " Timer irq not received while irq is reenabled \n" );
        return false;
    }

    return true;
}

/**
 * @brief Test get random numbers
 *
 * @remark
 * Test processing:
 * 1) - Get 2 random numbers in full range
 * - Check if numbers are not equals to 0 and are different
 * 2) - Get 2 random numbers in a defined range
 * - Check if numbers are different and in the defined range
 * 3) - Get random draw of numbers between in a defined range
 * - Check if draw of each value is equivalent
 *
 * Ported functions:
 * smtc_modem_hal_get_random_nb_in_range
 *      hal_rng_get_random_in_range
 *
 * @return bool True if test is successful
 */
static bool porting_test_random( void )
{
    bool ret = true;

    SMTC_HAL_TRACE_MSG( "----------------------------------------\n porting_test_random : \n" );

    SMTC_HAL_TRACE_MSG( " * Get random nb : " );
    uint32_t rdom1 = smtc_modem_hal_get_random_nb_in_range( 0, 0xFFFFFFFF );
    uint32_t rdom2 = smtc_modem_hal_get_random_nb_in_range( 0, 0xFFFFFFFF );

    if( ( rdom1 != 0 ) && ( rdom2 != 0 ) && ( rdom1 != rdom2 ) )
    {
        PORTING_TEST_MSG_OK( );
        SMTC_HAL_TRACE_PRINTF( " random1 = %u, random2 = %u\n", rdom1, rdom2 );
    }
    else
    {
        PORTING_TEST_MSG_WARN( "\n => random1 = %u, random2 = %u\n", rdom1, rdom2 );
        ret = false;
    }

    SMTC_HAL_TRACE_MSG( " * Get random nb in range : " );
    uint32_t range_min = 1;
    uint32_t range_max = 42;

    rdom1 = smtc_modem_hal_get_random_nb_in_range( range_min, range_max );
    rdom2 = smtc_modem_hal_get_random_nb_in_range( range_min, range_max );

    if( ( rdom1 >= range_min ) && ( rdom1 <= range_max ) && ( rdom2 >= range_min ) && ( rdom2 <= range_max ) &&
        ( rdom1 != rdom2 ) )
    {
        PORTING_TEST_MSG_OK( );
        SMTC_HAL_TRACE_PRINTF( " random1 = %u, random2 = %u in range [%u;%u]\n", rdom1, rdom2, range_min, range_max );
    }
    else
    {
        PORTING_TEST_MSG_WARN( "\n => random1 = %u, random2 = %u, expected range [%u;%u]\n", rdom1, rdom2, range_min,
                               range_max );
        ret = false;
    }

    SMTC_HAL_TRACE_MSG( " * Get random draw : " );
    range_min                       = 1;
    range_max                       = 10;
    uint32_t tab_counter_random[10] = { 0 };
    uint32_t nb_draw                = 100000;
    uint32_t probability_draw       = nb_draw / ( range_max - range_min + 1 );
    // Warning to be update if probability_draw is changed
    int16_t margin = ( probability_draw * 5 ) / 100;  // error margin = 5% of probability_draw

    for( uint32_t i = 0; i < nb_draw; i++ )
    {
        rdom1 = smtc_modem_hal_get_random_nb_in_range( range_min, range_max );
        tab_counter_random[rdom1 - 1]++;
    }

    uint8_t tab_size = sizeof( tab_counter_random ) / sizeof( uint32_t );
    for( uint16_t i = 0; i < tab_size; i++ )
    {
        if( abs( probability_draw - tab_counter_random[i] ) > margin )
        {
            PORTING_TEST_MSG_WARN( "\n => The number %u has been drawned %u times, Expected [%u;%u] times \n",
                                   ( i + 1 ), tab_counter_random[i], ( probability_draw - margin ),
                                   ( probability_draw + margin ) );
            ret = false;
        }
    }

    if( ret == true )
    {
        PORTING_TEST_MSG_OK( );
    }
    else
    {
        PORTING_TEST_MSG_WARN( " TODO Warning smtc_modem_hal_get_random_nb_in_range error margin > 5%% \n" );
    }

    SMTC_HAL_TRACE_PRINTF( " Random draw of %u numbers between [%u;%u] range \n", nb_draw, range_min, range_max );

    return ret;
}

/**
 * @brief Test time to configure rx radio
 *
 * @remark
 * Test processing:
 * - Init radio
 * - Configure radio irq
 * - Get start time
 * - Configure rx radio
 * - Get stop time
 * - Check configuration time
 *
 * @return bool True if test is successful
 */
static bool porting_test_config_rx_radio( void )
{
    SMTC_HAL_TRACE_MSG( "----------------------------------------\n porting_test_config_rx_radio :" );

    bool ret = true;
    // uint32_t rx_timeout_in_ms = 500;
    uint16_t counter_nok = 0;
    radio_irq_raised     = false;

    // Reset, init and put it in sleep mode radio
    // Setup radio and relative irq
    ret = reset_init_radio( );
    if( ret == false )
        return ret;

    smtc_modem_hal_irq_config_radio_irq( radio_rx_irq_callback, NULL );

    for( uint16_t i = 0; i < NB_LOOP_TEST_CONFIG_RADIO; i++ )
    {
        radio_irq_raised = false;

        uint32_t start_time_ms = smtc_modem_hal_get_time_in_ms( );
        // Setup radio and relative irq
        smtc_modem_hal_start_radio_tcxo( );
        smtc_modem_hal_set_ant_switch( false );
        if( ralf_setup_lora( &modem_radio, &rx_lora_param ) != RAL_STATUS_OK )
        {
            PORTING_TEST_MSG_NOK( " ralf_setup_lora() function failed \n" );
            return false;
        }
        if( ral_set_dio_irq_params( &( modem_radio.ral ), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
                                                              RAL_IRQ_RX_CRC_ERROR ) != RAL_STATUS_OK )
        {
            PORTING_TEST_MSG_NOK( " ral_set_dio_irq_params() function failed \n" );
            return false;
        }

        // Configure radio in reception mode
        // if( ral_set_rx( &( modem_radio.ral ), rx_timeout_in_ms ) !=
        //     RAL_STATUS_OK )
        // {
        //     PORTING_TEST_MSG_NOK( " ral_set_rx() function failed \n" );
        //     return false;
        // }

        uint32_t time = smtc_modem_hal_get_time_in_ms( ) - start_time_ms;
        if( time >= MARGIN_TIME_CONFIG_RADIO_IN_MS )
        {
            PORTING_TEST_MSG_NOK( " Configuration of rx radio is too long: %ums (margin +%ums) \n", time,
                                  MARGIN_TIME_CONFIG_RADIO_IN_MS );
            counter_nok++;
        }
        // else
        // {
        //     SMTC_HAL_TRACE_PRINTF( " Configuration of rx radio is: %ums  \n", time );
        // }

        smtc_modem_hal_stop_radio_tcxo( );
    }

    if( counter_nok == 0 )
    {
        PORTING_TEST_MSG_OK( );
    }
    else
    {
        PORTING_TEST_MSG_WARN( " => Failed test = %u / %u \n", counter_nok, NB_LOOP_TEST_CONFIG_RADIO );
    }

    return true;
}

/**
 * @brief Test time to configure tx radio
 *
 * @remark
 * Test processing:
 * - Init radio
 * - Configure radio irq
 * - Get start time
 * - Configure tx radio
 * - Get stop time
 * - Check configuration time
 *
 * @return bool True if test is successful
 */
static bool porting_test_config_tx_radio( void )
{
    SMTC_HAL_TRACE_MSG( "----------------------------------------\n porting_test_config_tx_radio :" );

    uint16_t payload_size = 50;
    uint8_t  payload[50]  = { 0 };
    uint16_t counter_nok  = 0;
    radio_irq_raised      = false;

    // Reset, init and put it in sleep mode radio
    bool ret = reset_init_radio( );
    if( ret == false )
        return ret;

    // Setup radio and relative irq
    smtc_modem_hal_irq_config_radio_irq( radio_tx_irq_callback, NULL );

    for( uint16_t i = 0; i < NB_LOOP_TEST_CONFIG_RADIO; i++ )
    {
        radio_irq_raised = false;

        uint32_t start_time_ms = smtc_modem_hal_get_time_in_ms( );

        smtc_modem_hal_start_radio_tcxo( );
        smtc_modem_hal_set_ant_switch( true );
        if( ralf_setup_lora( &modem_radio, &tx_lora_param ) != RAL_STATUS_OK )
        {
            PORTING_TEST_MSG_NOK( " ralf_setup_lora() function failed \n" );
            return false;
        }
        if( ral_set_dio_irq_params( &( modem_radio.ral ), RAL_IRQ_TX_DONE ) != RAL_STATUS_OK )
        {
            PORTING_TEST_MSG_NOK( " ral_set_dio_irq_params() function failed \n" );
            return false;
        }

        if( ral_set_pkt_payload( &( modem_radio.ral ), payload, payload_size ) != RAL_STATUS_OK )
        {
            PORTING_TEST_MSG_NOK( " ral_set_pkt_payload() function failed \n" );
            return false;
        }

        // if( ral_set_tx( &( modem_radio.ral ) ) != RAL_STATUS_OK )
        // {
        //     PORTING_TEST_MSG_NOK( " ral_set_tx() function failed \n" );
        //     return false;
        // }

        uint32_t time = smtc_modem_hal_get_time_in_ms( ) - start_time_ms;
        if( time >= MARGIN_TIME_CONFIG_RADIO_IN_MS )
        {
            PORTING_TEST_MSG_NOK( " Configuration of tx radio is too long: %ums (margin +%ums) \n", time,
                                  MARGIN_TIME_CONFIG_RADIO_IN_MS );
            counter_nok++;
        }
        // else
        // {
        //     SMTC_HAL_TRACE_PRINTF( " Configuration of tx radio is: %ums  \n", time );
        // }

        smtc_modem_hal_stop_radio_tcxo( );
    }

    if( counter_nok == 0 )
    {
        PORTING_TEST_MSG_OK( );
    }
    else
    {
        PORTING_TEST_MSG_WARN( " => Failed test = %u / %u \n", counter_nok, NB_LOOP_TEST_CONFIG_RADIO );
    }

    return true;
}

/**
 * @brief Test sleep time
 *
 * @remark
 * Test processing:
 * - Get start time
 * - Set sleep for ms
 * - Get stop time
 * - Check sleep time
 *
 * Ported functions:
 * hal_mcu_set_sleep_for_ms
 *      hal_watchdog_reload
 *      hal_rtc_wakeup_timer_set_ms
 *      lpm_handler
 *      hal_rtc_wakeup_timer_stop
 *
 * @return bool True if test is successful
 */
static bool porting_test_sleep_ms( void )
{
    SMTC_HAL_TRACE_MSG( "----------------------------------------\n porting_test_sleep_ms :" );

    bool    ret           = true;
    int32_t sleep_ms      = 2000;
    uint8_t wait_start_ms = 5;

    // Wait 5ms to start
    uint32_t start_time_ms = smtc_modem_hal_get_time_in_ms( ) + wait_start_ms;
    while( smtc_modem_hal_get_time_in_ms( ) < start_time_ms )
    {
        // Do nothing
    }

    hal_mcu_set_sleep_for_ms( sleep_ms );

    uint32_t stop_time_ms = smtc_modem_hal_get_time_in_ms( );
    uint32_t time         = stop_time_ms - start_time_ms;

    if( abs( time - sleep_ms ) <= MARGIN_SLEEP_IN_MS )
    {
        PORTING_TEST_MSG_OK( );
        SMTC_HAL_TRACE_PRINTF( " Sleep time expected %ums / get %ums (margin +/-%ums) \n", sleep_ms, time,
                               MARGIN_SLEEP_IN_MS );
    }
    else
    {
        PORTING_TEST_MSG_WARN( "\n => Sleep time is not coherent: expected %ums / get %ums (margin +/-%ums) \n",
                               sleep_ms, time, MARGIN_SLEEP_IN_MS );
    }
    return ret;
}

/**
 * @brief Test timer IRQ in low power
 *
 * @warning smtc_modem_hal_start_timer() function takes ~4ms for STM32L4 (see HAL_LPTIM_TimeOut_Start_IT())
 *
 * @remark
 * Test processing:
 * - Get start time
 * - Configure and start timer
 * - Wait timer irq
 * - Get stop time
 * - Check the time elapsed between timer start and timer IRQ reception
 *
 * Ported functions:
 * smtc_modem_hal_start_timer
 *      hal_lp_timer_start
 *
 * @return bool True if test is successful
 */
static bool porting_test_timer_irq_low_power( void )
{
    SMTC_HAL_TRACE_MSG( "----------------------------------------\n porting_test_timer_irq_low_power : " );

    uint32_t timer_ms      = 3000;
    int32_t  sleep_ms      = timer_ms + 5000;
    uint8_t  wait_start_ms = 5;
    timer_irq_raised       = false;

    smtc_modem_hal_stop_timer( );

    // Wait 5ms to start
    uint32_t start_time_ms = smtc_modem_hal_get_time_in_ms( ) + wait_start_ms;
    while( smtc_modem_hal_get_time_in_ms( ) < start_time_ms )
    {
        // Do nothing
    }

    smtc_modem_hal_start_timer( timer_ms, timer_irq_callback,
                                NULL );  // Warning this function takes ~3,69 ms for STM32L4

    hal_mcu_set_sleep_for_ms( sleep_ms );

    if( timer_irq_raised == false )
    {
        PORTING_TEST_MSG_NOK( " Timeout: timer irq not received \n" );
        return false;
    }

    uint32_t time =
        irq_time_ms - start_time_ms - BOARD_COMPENSATION_IN_MS;  // TODO Warning to compensate delay introduced by
                                                                 // smtc_modem_hal_start_timer for STM32L4
    if( ( time >= timer_ms ) && ( time <= timer_ms + MARGIN_TIMER_IRQ_IN_MS ) )
    {
        PORTING_TEST_MSG_OK( );
        SMTC_HAL_TRACE_PRINTF( " Timer irq configured with %ums / get %ums (margin +%ums) \n", timer_ms, time,
                               MARGIN_TIMER_IRQ_IN_MS );
    }
    else
    {
        PORTING_TEST_MSG_NOK( " Timer irq delay is not coherent: expected %ums / get %ums (margin +%ums) \n", timer_ms,
                              time, MARGIN_TIMER_IRQ_IN_MS );
        return false;
    }
    return true;
}

/*
 * -----------------------------------------------------------------------------
 * --- FLASH PORTING TESTS -----------------------------------------------------
 */
#if( ENABLE_TEST_FLASH != 0 )

/**
 * @brief Test read/write context in flash
 *
 * @remark
 * Test processing:
 * - Read context in flash
 * - Write a different context in flash
 * - Read context in flash
 * - Check if read context is equal to written context
 *
 * Ported functions:
 * smtc_modem_hal_context_restore
 *     hal_flash_read_buffer
 * smtc_modem_hal_context_store
 *     hal_flash_write_buffer
 *
 * @param [in]  context_type   The context type
 *
 * @return bool True if test is successful
 */
static bool test_context_store_restore( modem_context_type_t context_type )
{
    bool    ret             = true;
    uint8_t read_buffer[8]  = { 0 };
    uint8_t write_buffer[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    bool    cmp             = true;

    SMTC_HAL_TRACE_PRINTF( "\n * Context %s : \n", name_context_type[context_type] );

    smtc_modem_hal_context_restore( context_type, 0, read_buffer, sizeof( read_buffer ) );

    SMTC_HAL_TRACE_MSG( " Read:  { " );
    for( uint8_t i = 0; i < sizeof( read_buffer ); i++ )
    {
        SMTC_HAL_TRACE_PRINTF( "%u", read_buffer[i] );
        if( i != ( sizeof( read_buffer ) - 1 ) )
            SMTC_HAL_TRACE_MSG( ", " );
    }
    SMTC_HAL_TRACE_MSG( " }\n" );

    for( uint8_t i = 0; i < sizeof( write_buffer ); i++ )
    {
        if( read_buffer[i] == write_buffer[i] )
        {
            write_buffer[i] = ( read_buffer[i] + 1 ) % 256;
        }
    }

    SMTC_HAL_TRACE_MSG( " Write: { " );
    for( uint8_t i = 0; i < sizeof( write_buffer ); i++ )
    {
        SMTC_HAL_TRACE_PRINTF( "%u", write_buffer[i] );
        if( i != ( sizeof( write_buffer ) - 1 ) )
            SMTC_HAL_TRACE_MSG( ", " );
    }
    SMTC_HAL_TRACE_MSG( " }\n" );

    smtc_modem_hal_context_store( context_type, 0, write_buffer, sizeof( write_buffer ) );

    memset( read_buffer, 0, sizeof( read_buffer ) );
    smtc_modem_hal_context_restore( context_type, 0, read_buffer, sizeof( read_buffer ) );

    SMTC_HAL_TRACE_MSG( " Read:  { " );
    for( uint8_t i = 0; i < sizeof( read_buffer ); i++ )
    {
        SMTC_HAL_TRACE_PRINTF( "%u", read_buffer[i] );
        if( i != ( sizeof( read_buffer ) - 1 ) )
            SMTC_HAL_TRACE_MSG( ", " );
    }
    SMTC_HAL_TRACE_MSG( " }\n" );

    for( uint8_t i = 0; i < sizeof( write_buffer ); i++ )
    {
        if( read_buffer[i] != write_buffer[i] )
        {
            cmp = false;
        }
    }
    if( cmp == true )
    {
        SMTC_HAL_TRACE_MSG( " Store/restore without MCU reset :" );
        PORTING_TEST_MSG_OK( );
    }
    else
    {
        PORTING_TEST_MSG_NOK( " Store or restore context failed (without MCU reset) \n\n" );
        return false;
    }

    return ret;
}

/**
 * @brief Test read/write context in flash
 *
 * @remark
 * Test processing:
 * - See test_context_store_restore() function
 * - Reset MCU
 * - RELAUNCH this test after mcu reset
 * - Check if read after reset = write before reset
 *
 * @return bool True if test is successful
 */
static bool porting_test_flash( void )
{
    bool ret = true;
    SMTC_HAL_TRACE_MSG( "----------------------------------------\n porting_test_flash : \n" );
    SMTC_HAL_TRACE_MSG_COLOR( " !! TEST TO BE LAUNCH TWICE !! To check writing after MCU reset \n",
                              HAL_DBG_TRACE_COLOR_BLUE );

    /* LORAWAN */
    ret = test_context_store_restore( CONTEXT_LORAWAN_STACK );
    if( ret == false )
        return ret;

    /* MODEM */
    ret = test_context_store_restore( CONTEXT_MODEM );
    if( ret == false )
        return ret;

    /* MODEM KEY */
    ret = test_context_store_restore( CONTEXT_KEY_MODEM );
    if( ret == false )
        return ret;

    /* SECURE ELEMENT */
    ret = test_context_store_restore( CONTEXT_SECURE_ELEMENT );
    if( ret == false )
        return ret;

    return ret;
}
#endif
/*
 * -----------------------------------------------------------------------------
 * --- IRQ CALLBACK DEFINITIONS ---------------------------------------------------------
 */

/**
 * @brief Radio tx irq callback
 */
static void radio_tx_irq_callback( void* obj )
{
    UNUSED( obj );
    // ral_irq_t radio_irq = 0;

    irq_time_ms = smtc_modem_hal_get_time_in_ms( );

    radio_irq_raised = true;

    // if( ral_get_irq_status( &( modem_radio.ral ), &radio_irq ) != RAL_STATUS_OK )
    // {
    //     SMTC_HAL_TRACE_MSG_COLOR( "NOK\n ral_get_irq_status() function failed \n", HAL_DBG_TRACE_COLOR_RED );
    // }
    // SMTC_HAL_TRACE_INFO( " RP: IRQ source - 0x%04x\n", radio_irq );
    if( ral_clear_irq_status( &( modem_radio.ral ), RAL_IRQ_ALL ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_clear_irq_status() function failed \n" );
    }
}

/**
 * @brief Radio rx irq callback (get time in ms)
 */
static void radio_rx_irq_callback( void* obj )
{
    UNUSED( obj );

    ral_irq_t radio_irq = 0;
    irq_time_ms         = smtc_modem_hal_get_time_in_ms( );
    radio_irq_raised    = true;

    if( ral_get_irq_status( &( modem_radio.ral ), &radio_irq ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( "ral_get_irq_status() function failed \n" );
    }
    // SMTC_HAL_TRACE_INFO( " RP: IRQ source - 0x%04x\n", radio_irq );

    if( ( radio_irq & RAL_IRQ_RX_TIMEOUT ) == RAL_IRQ_RX_TIMEOUT )
    {
        irq_rx_timeout_raised = true;
    }

    if( ral_clear_irq_status( &( modem_radio.ral ), RAL_IRQ_ALL ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_clear_irq_status() function failed \n" );
    }

    // Shut Down the TCXO
    smtc_modem_hal_stop_radio_tcxo( );
}

/**
 * @brief Radio irq callback (get time in s)
 */
static void radio_irq_callback_get_time_in_s( void* obj )
{
    UNUSED( obj );
    ral_irq_t radio_irq = 0;
    irq_time_s          = smtc_modem_hal_get_time_in_s( );
    radio_irq_raised    = true;

    if( ral_get_irq_status( &( modem_radio.ral ), &radio_irq ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_get_irq_status() function failed \n" );
    }

    if( ( radio_irq & RAL_IRQ_RX_TIMEOUT ) == RAL_IRQ_RX_TIMEOUT )
    {
        irq_rx_timeout_raised = true;
    }

    if( ral_clear_irq_status( &( modem_radio.ral ), RAL_IRQ_ALL ) != RAL_STATUS_OK )
    {
        PORTING_TEST_MSG_NOK( " ral_clear_irq_status() function failed \n" );
    }

    // Shut Down the TCXO
    smtc_modem_hal_stop_radio_tcxo( );
}

/**
 * @brief Timer irq callback
 */
static void timer_irq_callback( void* obj )
{
    UNUSED( obj );
    irq_time_ms      = smtc_modem_hal_get_time_in_ms( );
    timer_irq_raised = true;
}


/**************************************************************************************************
##################### APP PERIODICAL_UPLINK ########################################################
***************************************************************************************************/

/**
 * @brief Example to send a user payload on an external event
 *
 */
void main_periodical_uplink( void )
{
    uint32_t sleep_time_ms = 0;
    smtc_modem_status_mask_t status_mask = 0;

    // Disable IRQ to avoid unwanted behavior during init
    hal_mcu_disable_irq( );

    // Configure all the µC periph (clock, gpio, timer, ...)
    hal_mcu_init( );

    // Init the modem and use modem_event_callback as event callback, please note that the callback will be
    // called immediately after the first call to smtc_modem_run_engine because of the reset detection
    smtc_modem_init( &modem_event_callback );

    // Configure Nucleo blue button as EXTI
    hal_gpio_irq_t qb20_evk_k1 = {
        .pin      = QB20_EVK_K1,
        .context  = NULL,                  // context pass to the callback - not used in this example
        .callback = k1_button_callback,  // callback called when K1 is pressed
    };
    hal_gpio_irq_t qb20_evk_k2 = {
        .pin      = QB20_EVK_K2,
        .context  = NULL,                  // context pass to the callback - not used in this example
        .callback = k2_button_callback,  // callback called when K2 is pressed
    };

    hal_gpio_init_in( QB20_EVK_K1, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_FALLING, &qb20_evk_k1 );
    hal_gpio_init_in( QB20_EVK_K2, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_FALLING, &qb20_evk_k2 );

    // Init done: enable interruption
    hal_mcu_enable_irq( );

    SMTC_HAL_TRACE_INFO( "Periodical uplink (%d sec) example is starting \n", PERIODICAL_UPLINK_DELAY_S );

    while( 1 )
    {
        smtc_modem_get_status( STACK_ID, &status_mask );
        // Check button
        if( k1_button_is_press == true )
        {
            k1_button_is_press = false;

            // Check if the device has already joined a network
            if( ( status_mask & SMTC_MODEM_STATUS_JOINED ) == SMTC_MODEM_STATUS_JOINED )
            {
                // Send the uplink counter on port 102
                send_uplink_counter_on_port( 102 );
            }
        }

        // if ( k2_button_is_press == false && ( status_mask & SMTC_MODEM_STATUS_JOINING ) == SMTC_MODEM_STATUS_JOINING ) {
        //     goto sleep;
        // }

        // Modem process launch
        sleep_time_ms = smtc_modem_run_engine( );

sleep:
        // Atomically check sleep conditions (button was not pressed and no modem flags pending)
        hal_mcu_disable_irq( );
        if( ( k1_button_is_press == false ) && ( smtc_modem_is_irq_flag_pending( ) == false ) )
        {
            hal_watchdog_reload( );
            hal_mcu_set_sleep_for_ms( MIN( sleep_time_ms, WATCHDOG_RELOAD_PERIOD_MS ) );
        }
        hal_watchdog_reload( );
        hal_mcu_enable_irq( );
    }
}

/**************************************************************************************************
################################## APP LCTT TEST ##################################################
***************************************************************************************************/

static bool certif_running = false;

static void main_handle_push_button( void )
{
    if( certif_running == true )
    {
        smtc_modem_set_certification_mode( STACK_ID, false );
        ASSERT_SMTC_MODEM_RC( smtc_modem_leave_network( STACK_ID ) );
        ASSERT_SMTC_MODEM_RC( smtc_modem_join_network( STACK_ID ) );
        certif_running = false;
    }
    else
    {
        smtc_modem_set_certification_mode( STACK_ID, true );
        certif_running = true;
    }
}

void main_lctt_certif(void)
{
    uint32_t sleep_time_ms = 0;

    // Disable IRQ to avoid unwanted behavior during init
    hal_mcu_disable_irq( );

    // Configure all the µC periph (clock, gpio, timer, ...)
    hal_mcu_init( );

    // Init the modem and use modem_event_callback as event callback, please note that the callback will be
    // called immediately after the first call to smtc_modem_run_engine because of the reset detection
    smtc_modem_init( &modem_event_callback );

    // Configure Nucleo blue button as EXTI
    hal_gpio_irq_t qb20_evk_k1 = {
        .pin      = QB20_EVK_K1,
        .context  = NULL,                  // context pass to the callback - not used in this example
        .callback = k1_button_callback,  // callback called when EXTI is triggered
    };
    hal_gpio_init_in( QB20_EVK_K1, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_FALLING, &qb20_evk_k1 );

    // Init done: enable interruption
    hal_mcu_enable_irq( );

    SMTC_HAL_TRACE_INFO( "Certification example is starting\n" );
    SMTC_HAL_TRACE_INFO( "Push button to enable/disable certification\n" );

    while( 1 )
    {
        // Check button
        if( k1_button_is_press == true )
        {
            k1_button_is_press = false;

            main_handle_push_button( );
        }

        // Modem process launch
        sleep_time_ms = smtc_modem_run_engine( );

        // Atomically check sleep conditions (button was not pressed)
        hal_mcu_disable_irq( );
        if( ( k1_button_is_press == false ) && ( smtc_modem_is_irq_flag_pending( ) == false ) )
        {
            hal_watchdog_reload( );
            hal_mcu_set_sleep_for_ms( MIN( sleep_time_ms, WATCHDOG_RELOAD_PERIOD_MS ) );
        }
        hal_watchdog_reload( );
        hal_mcu_enable_irq( );
    }
}


/* --- EOF ------------------------------------------------------------------ */