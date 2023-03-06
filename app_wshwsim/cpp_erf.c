/**
 * @file efr32_rf_driver.cpp
 * @author Yann Charbon <yann.charbon@heig-vd.ch>
 * @brief PHY driver for EFR32 RAIL radio supporting IEEE 802.15.4-2015+ frames (Wi-SUN ready).
 * @version 1.1
 * @date 2022-06-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "efr32_rf_driver.h"
#include "nanostack/platform/arm_hal_phy.h"
#include "common_functions.h"
#include <Timer.h>
#include "Timeout.h"
#include "Thread.h"
#include "platform/arm_hal_interrupt.h"
#include "mbed_toolchain.h"
#include "mbed.h"
#include "randLIB.h"

using namespace std::chrono;

/* Driver specific headers */
extern "C" {
#include "rail_config.h"
#include "ieee_802154_frame.h"
#include "efr32_rf_wisun_util.h"
}

/* Silicon Labs headers */
extern "C" {
#include "rail/common/rail.h"
}

#include "mbed-trace/mbed_trace.h"
#define  TRACE_GROUP  "EFR32"

//#define EFR32_DEBUG
#ifdef EFR32_DEBUG
#ifndef EFR32_TR_DEBUG
#define EFR32_TR_DEBUG(...) tr_debug(__VA_ARGS__)
#endif
#else
#define EFR32_TR_DEBUG(...)
#endif

/* Driver instance handle */
static Efr32RfDriver *efr32RfDriver = NULL;

/* Nanostack related variables configuration */
static uint8_t mac_address[8];
phy_device_driver_s device_driver;
int8_t rf_radio_driver_id = -1;
uint8_t mac_tx_handle;


/* ********************* Driver enumerations ************************** */

/**
 * @enum driver_state
 * @brief Represents the state of the driver
 * 
 * @note IMPORTANT : this is not the state of the radio, but of the driver
 */
enum driver_state {
    /// Driver is not initialized
    DRVSTATE_RADIO_UNINIT,
    /// Driver is beeing initialized but is not ready to be used
    DRVSTATE_RADIO_INITING,
    /// Driver is in a state where any operation/maintenance can be performed
    DRVSTATE_IDLE,
    /// Driver is listening channel for incoming packets
    DRVSTATE_IDLE_WAITING_RX,
    /// Driver is transmitting
    DRVSTATE_TX,
    /// Driver is receiving
    DRVSTATE_RX,
    /// Driver is inside the CSMA process
    DRVSTATE_CSMA_STARTED,
};


/**
 * @enum idle_state
 * @brief Represent the different idle states depths that can be reached * 
 */
enum idle_state {
    /// Puts the radio in idle state after current TRX operation is finished
    RADIO_IDLE,
    /// Puts the radio in idle state immediately by cancelling the current TRX operation
    RADIO_IDLE_ABORT,
    /// Puts the radio in shutdown/sleep mode immediately by cancelling the current TRX operation and clearing any pending flags
    RADIO_SHUTDOWN,
};

/* ********************* Prototypes ************************** */

/* *******************************
 * RF driver "public" functions *
 * *******************************
 */
static int8_t rf_device_register(void);
static void rf_device_unregister(void);
static int8_t rf_interface_state_control(phy_interface_state_e new_state, uint8_t rf_channel);
static int8_t rf_extension(phy_extension_type_e extension_type, uint8_t *data_ptr);
static int8_t rf_address_write(phy_address_type_e address_type, uint8_t *address_ptr);
static uint32_t rf_get_timestamp();
static int8_t rf_start_cca(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol);

/* *******************************
 * RF driver private functions *
 * *******************************
 */

/* Initialization functions */
static void _rf_init_efr32_RAIL();
static void _rf_ready_cb(RAIL_Handle_t handle);

/* Interface control */
static int _rf_interface_up(int8_t new_channel);
static void _rf_reset_interface(enum idle_state state);

/* Channel frequency, channel, etc., configuration functions  */
static int _rf_check_and_change_channel(int8_t new_channel);
static int _rf_check_and_apply_rf_config(phy_rf_channel_configuration_s *config, bool only_check);
static int _rf_update_phy_config(phy_rf_channel_configuration_s *new_config);
static void _rf_apply_phy_config();
static void _rf_update_symbol_rate(uint32_t baudrate, phy_modulation_e modulation);

/* Radio state related functions */
static bool _rf_is_busy();
static void _rf_set_idle(enum idle_state mode);
static void _rf_poll_state_change(enum driver_state expected_drv_state);

/* Transmit, receive, CCA related functions */
static uint16_t _rf_write_tx_fifo(uint8_t *dataPtr, uint16_t length, bool reset);
static int _rf_execute_tx();
static bool _rf_check_cca();
static void _rf_handle_rx_end();

/* Radio events handling IRQ and thread functions */
static void _rf_rail_events_irq_handler(RAIL_Handle_t railHandle, RAIL_Events_t events);
static void _rf_irq_thread_task();

/* CSMA timer realted functions */
static void _rf_start_csma_timeout_timer(uint32_t duration);
static void _rf_stop_csma_timeout_timer();
static void _rf_rearm_csma_timeout_timer();
static void _rf_csma_timeout_timer_cb();
static void _rf_csma_timeout_handler();

/* Backup timer related functions */
static void _rf_start_backup_timer(uint32_t duration);
static void _rf_stop_backup_timer();
static void _rf_backup_timer_cb();
static void _rf_backup_timeout_handler();

/* *****************************************************/

/* ********************* RF driver variables and macros ************************** */

// Default config is Wi-SUN EU option 3
static phy_rf_channel_configuration_s cur_phy_config = {863100000, 200000, 150000, 35, M_2FSK, MODULATION_INDEX_0_5};

static phy_device_channel_page_s phy_channel_pages[] = {
    {CHANNEL_PAGE_0, &cur_phy_config},
    {CHANNEL_PAGE_0, NULL},
}; 

#define FIFO_SIZE 4096
#define PACKET_SENDING_EXTRA_TIME   5000
uint32_t MAX_PACKET_SENDING_TIME = (uint32_t)(8000000/cur_phy_config.datarate)*FIFO_SIZE + PACKET_SENDING_EXTRA_TIME;


#define SIG_ALL             (SIG_RADIO_ERROR|SIG_CSMA_TIMEOUT|SIG_TIMER_BACKUP|SIG_PACKET_RECEIVED|SIG_PACKET_SENT|SIG_RX_FIFO_EVENTS)
#define SIG_RADIO_ERROR     1
#define SIG_CSMA_TIMEOUT    2
#define SIG_TIMER_BACKUP    4
#define SIG_PACKET_RECEIVED 8
#define SIG_PACKET_SENT     16
#define SIG_RX_FIFO_EVENTS  32

static RAIL_Handle_t rail_handle;
static int8_t channel = -1;
static int8_t new_channel = -1;
static uint8_t tx_fifo[FIFO_SIZE];
static uint8_t rxFifo[FIFO_SIZE];
static Timer rf_timer; // For gloabl time stamp
static Timeout csma_timeout_timer;
static Timeout backup_timeout_timer;
static Thread irq_thread(osPriorityRealtime, 1024);
static enum driver_state driver_state = DRVSTATE_RADIO_UNINIT;
static bool sleep_blocked = false;
static volatile RAIL_RxPacketStatus_t radio_error_packet_status;
static uint32_t rf_symbol_rate;
static bool cca_enabled;
static uint32_t backoff_time;
static int16_t rssi_cca_threshold = RSSI_THRESHOLD;
static bool phy_config_update_requested = false;
static uint32_t cur_pkt_rx_sync_word_time = 0;
static uint32_t last_pkt_rx_sync_word_time = 0;

/* Queue for passing packets from interrupt to adaptor thread */

/**
 * @enum rx_queue_event
 * @brief Represents the different events that can be triggered for the RX queue
 * 
 */
enum rx_queue_event {
    RX_QUEUE_FULL,
    RX_QUEUE_ALMOST_FULL,
    RX_QUEUE_OVERFLOW,
};

/**
 * @struct rx_queue_slot
 * @brief Represents a slot in the RX queue
 * 
 */
struct rx_queue_slot {
    uint8_t buffer[FIFO_SIZE];
    uint16_t length;
    uint8_t lqi;
    int8_t rssi;
    uint32_t rx_time;
};

#define RX_QUEUE_SLOTS 6
static struct rx_queue_slot rx_queue[RX_QUEUE_SLOTS];
static volatile size_t rx_queue_head = 0;
static volatile size_t rx_queue_tail = 0;
static volatile enum rx_queue_event rx_fifo_irq_event;

/* Debugging variables */
/*static uint32_t DBG_ack_req_tx_time = 0;
static uint32_t DBG_ack_rx_time = 0;
static uint16_t DBG_cur_tx_data_length = 0;
static uint32_t DBG_start_cca_time = 0;
static uint32_t DBG_restart_csma_time = 0;
static uint32_t DBG_exec_tx_time = 0;
static uint32_t DBG_backoff_time = 0;*/

#ifdef EFR32_ENABLE_TEST_GPIOS
DigitalOut fhss_uc_switch_test_pin(EFR32_FHSS_UC_PIN);
DigitalOut fhss_bc_switch_test_pin(EFR32_FHSS_BC_PIN);

static void efr32_fhss_uc_switch_debug(void)
{
    fhss_uc_switch_test_pin = !fhss_uc_switch_test_pin;
}

static void efr32_fhss_bc_switch_debug(void)
{
    fhss_bc_switch_test_pin = !fhss_bc_switch_test_pin;
}
#endif


/* RAIL configuration structure */
static RAIL_Config_t rail_config = { // Must never be const
    .eventsCallback = &_rf_rail_events_irq_handler,
    .protocol = NULL, // For BLE, pointer to a RAIL_BLE_State_t. For IEEE802.15.4 this must be NULL.
    .scheduler = NULL, // For MultiProtocol, pointer to a RAIL_SchedConfig_t
};

/* RAIL power amplifier configuration */
static const RAIL_TxPowerConfig_t tx_power_config = { 
    .mode = RAIL_TX_POWER_MODE_SUBGIG, 
    .voltage = 1800, 
    .rampTime = 10 
};


/* ********************* RF driver "public" functions implementation ************************** */

/**
 * @brief Configures the RAIL API and the Nanostack PHY device driver
 * 
 * @return The driver ID or -1 if the driver is already initialized 
 */
static int8_t rf_device_register(void){
    /* Avoid registering if already registered */
    if (driver_state != DRVSTATE_RADIO_UNINIT){
        return -1;
    }

    /* Init RAIL for EFR32 */
    _rf_init_efr32_RAIL();

    /* Update the actual symbol rate */
    _rf_update_symbol_rate(cur_phy_config.datarate, cur_phy_config.modulation);

    /* Set pointer to MAC address */
    device_driver.PHY_MAC = mac_address;
    /* Set driver Name */
    device_driver.driver_description = (char*)"EFR32 Wi-SUN compatible driver";

    /* Configuration for Sub GHz Radio */
    device_driver.link_type = PHY_LINK_15_4_SUBGHZ_TYPE;
    device_driver.phy_channel_pages = phy_channel_pages;

    /*Maximum size of payload is 2047*/
    device_driver.phy_MTU = 2047;
    /*2 bytes PHY header (PHR)*/
    device_driver.phy_header_length = 2;   // This makes Nanostack add two empty byte at beginning of the data to transmit
    /*No tail in PHY*/
    device_driver.phy_tail_length = 0;

    /*Set up driver functions*/
    device_driver.address_write = &rf_address_write;
    device_driver.extension = &rf_extension;
    device_driver.state_control = &rf_interface_state_control;
    device_driver.tx = &rf_start_cca;
    //Nullify rx/tx callbacks
    device_driver.phy_rx_cb = NULL;
    device_driver.phy_tx_done_cb = NULL;
    device_driver.arm_net_virtual_rx_cb = NULL;
    device_driver.arm_net_virtual_tx_cb = NULL;

    driver_state = DRVSTATE_RADIO_INITING;

    /*Register device driver*/
    rf_radio_driver_id = arm_net_phy_register(&device_driver);

#ifdef EFR32_ENABLE_TEST_GPIOS
    fhss_bc_switch = efr32_fhss_bc_switch_debug;
    fhss_uc_switch = efr32_fhss_uc_switch_debug;
#endif //TEST_GPIOS_ENABLED

    return rf_radio_driver_id;
}

/**
 * @brief Unregisters the PHY device driver from Nanostack
 * 
 */
static void rf_device_unregister(void){
    arm_net_phy_unregister(rf_radio_driver_id);
    if (sleep_blocked) {
        sleep_manager_unlock_deep_sleep();
        sleep_blocked = false;
    }
}

/**
 * @brief This function is called by Nanostack to activate/deactivate the driver
 * 
 * @param new_state The state to which the driver should be changed
 * @param rf_channel The channel on which the driver should listen when it is started
 * @return 0 on success, -1 if channel could not be set 
 */
static int8_t rf_interface_state_control(phy_interface_state_e new_state, uint8_t rf_channel)
{
    int8_t ret_val = 0;
    switch (new_state)
    {
        /*Reset PHY driver and set to idle*/
        case PHY_INTERFACE_RESET:
            EFR32_TR_DEBUG("PHY_INTERFACE_RESET");
            _rf_reset_interface(RADIO_IDLE_ABORT);
            break;
        /*Disable PHY Interface driver*/
        case PHY_INTERFACE_DOWN:
            EFR32_TR_DEBUG("PHY_INTERFACE_DOWN");
            _rf_reset_interface(RADIO_SHUTDOWN);
            break;
        /*Enable PHY Interface driver
         * -> set channel
         * -> set RX mode */
        case PHY_INTERFACE_UP:
            EFR32_TR_DEBUG("PHY_INTERFACE_UP");
            ret_val = _rf_interface_up(rf_channel);
            break;
        /*Enable wireless interface ED scan mode*/
        case PHY_INTERFACE_RX_ENERGY_STATE:
            // Not required
            break;
        /*Enable Sniffer state*/
        case PHY_INTERFACE_SNIFFER_STATE:
            EFR32_TR_DEBUG("PHY_INTERFACE_SNIFFER_STATE");
            ret_val = _rf_interface_up(rf_channel);
            break;
    }
    return ret_val;
}

/**
 * @brief This function is called by Nanostack to configure/retrieve information from the driver during runtime
 * 
 * @param extension_type Parameter to get/set
 * @param data_ptr A pointer from which the parameter value can be read or to which data can be written
 * @return 0 on success, -1 on failure
 */
static int8_t rf_extension(phy_extension_type_e extension_type, uint8_t *data_ptr)
{
    uint32_t *data_ptr_32bits;
    phy_csma_params_t *new_csma_params;
    phy_rf_channel_configuration_s *new_channel_params;

    int ret = 0;

    switch (extension_type)
    {
        case PHY_EXTENSION_SET_CHANNEL:
            EFR32_TR_DEBUG("PHY_EXTENSION_SET_CHANNEL %u\n", *data_ptr);
            if (driver_state == DRVSTATE_IDLE ||
                driver_state == DRVSTATE_IDLE_WAITING_RX ||
                 driver_state == DRVSTATE_CSMA_STARTED){
                new_channel = *data_ptr;
                _rf_set_rx();
            } else {
                // Store for later : channel could not be set yet
                new_channel = *data_ptr;
                ret = -1;
            }            
            break;
        case PHY_EXTENSION_DYNAMIC_RF_SUPPORTED:
            *data_ptr = true;
            break;
        case PHY_EXTENSION_GET_TIMESTAMP:
            data_ptr_32bits = (uint32_t *)data_ptr;
            *data_ptr_32bits = rf_get_timestamp();
            break;
        case PHY_EXTENSION_READ_RX_TIME:
            common_write_32_bit(cur_pkt_rx_sync_word_time, data_ptr);
            break;
        case PHY_EXTENSION_GET_SYMBOLS_PER_SECOND:
            data_ptr_32bits = (uint32_t *)data_ptr;
            *data_ptr_32bits = rf_symbol_rate;
            break;
        case PHY_EXTENSION_SET_CSMA_PARAMETERS:
            new_csma_params = (phy_csma_params_t*)data_ptr;
            if (new_csma_params->backoff_time == 0){
                // Cancel any on-going transmission and set to RX state on current channel
                _rf_stop_csma_timeout_timer();
                _rf_set_idle(RADIO_IDLE_ABORT);
                driver_state = DRVSTATE_IDLE;
                _rf_set_rx();             
                backoff_time = 0;                
            } else {
                // Store backoff_time and cca_enabled
                cca_enabled = new_csma_params->cca_enabled;
                backoff_time = new_csma_params->backoff_time;
            }
            
            EFR32_TR_DEBUG("PHY_EXTENSION_SET_CSMA_PARAMETERS : cca_enabled=%u backoff_time=%u", cca_enabled, backoff_time);
            break;
        case PHY_EXTENSION_SET_RF_CONFIGURATION:
            EFR32_TR_DEBUG("PHY_EXTENSION_SET_RF_CONFIGURATION");
            new_channel_params = (phy_rf_channel_configuration_s*)data_ptr;            
            ret = _rf_update_phy_config(new_channel_params);
            break;
        case PHY_EXTENSION_SET_CHANNEL_CCA_THRESHOLD:
            EFR32_TR_DEBUG("PHY_EXTENSION_SET_CHANNEL_CCA_THRESHOLD : old=%d new=%d", rssi_cca_threshold, (int16_t)*data_ptr);
            rssi_cca_threshold = (int8_t)*data_ptr;
            break;
        default:
            EFR32_TR_DEBUG("Unhandled PHY_EXTENSION type (%d)", extension_type);
            break;
    }
    return ret;
}

/**
 * @brief Used by Nanostack to write MAC long and short addresses, PAN address, etc.
 * 
 * @param address_type Type of address)
 * @param address_ptr Buffer containing the address
 * @return 0 on success, -1 on failure 
 */
static int8_t rf_address_write(phy_address_type_e address_type, uint8_t *address_ptr)
{
    switch (address_type)
    {
        /*Set 48-bit address*/
        case PHY_MAC_48BIT:
            // 15.4 does not support 48-bit addressing
            break;
        /*Set 64-bit address*/
        case PHY_MAC_64BIT:
            /* Store MAC in MSB order */
            memcpy(mac_address, address_ptr, 8);
#ifdef EFR32_DEBUG
            tr_debug("rf_address_write: MAC");
            for (unsigned int i = 0; i < sizeof(mac_address); i ++) {
                tr_debug("%02x:", mac_address[i]);
            }
            tr_debug("\n");
#endif
            break;
        /*Set 16-bit address*/
        case PHY_MAC_16BIT:
            // Not required for IEEE 802.15.4-2015+ frames
            break;
        /*Set PAN Id*/
        case PHY_MAC_PANID:
            // Not required for IEEE 802.15.4-2015+ frames
            break;
    }

    return 0;
}

/**
 * @brief Gets the monotonic timestamp of the driver
 * 
 * @return Timestamp in microseconds
 *
 * The timestamp corresponds to the time since the driver was initialized. 
 */
static uint32_t rf_get_timestamp(){
    return (uint32_t)rf_timer.elapsed_time().count();
}

/**
 * @brief Called by Nanostack to transmit a packet
 * 
 * @param data_ptr A pointer to the buffer containing the packet to be transmitted
 * @param data_length Packet length
 * @param tx_handle An ID that is used by Nanostack to track the packet inside the driver during the CSMA/CA process
 * @param data_protocol Not used here.
 * @return 0 on success, -1 if the driver is busy (already transmitting)
 */
static int8_t rf_start_cca(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol){
    /*Check if transmitter is busy*/
    if(_rf_is_busy()) {
        return -1;
    }

    platform_enter_critical();

    driver_state = DRVSTATE_CSMA_STARTED;

    /*Save tx_handle to be able to use it inside IRQ handler*/
    mac_tx_handle = tx_handle;

    /*Create PHY header for Wi-SUN and insert it at beginning of TX buffer*/
    uint16_t phr = wisun_util_create_phy_header(data_length, WISUN_FAN_PROFILE);
    uint8_t* phr_0 = (uint8_t*)&phr;
    uint8_t* phr_1 = phr_0 + 1;
    data_ptr[0] = *phr_0;
    data_ptr[1] = *phr_1;

#ifdef EFR32_DEBUG
    printf("===== TX Packet (sending on channel %u) :\n", channel);
    for(int i=0; i < (data_length + device_driver.phy_header_length); i++){
        printf(" 0x%02X", data_ptr[i]);
    }
    printf("=====\n");
#endif    

    /*Load data to TX FIFO*/
    _rf_write_tx_fifo(data_ptr, data_length + device_driver.phy_header_length, true);
    
#ifdef EFR32_DEBUG
    if(txFifo[IEEE_FRAME_CONTROL_BYTE_0 + device_driver.phy_header_length] & IEEE_TYPE_ACK){
        tr_warn("ACK length = %u\n", data_length + device_driver.phy_header_length);
    }
#endif

    /*Calculate CSMA-CA period and activate timeout timer*/
    if (backoff_time > 0) {
        _rf_rearm_csma_timeout_timer();
    } else {
        // If there is no backoff time, we can start transmitting immediately
        _rf_start_csma_timeout_timer(1);
    }

    platform_exit_critical();

    /*Return success*/
    return 0;
}

/* ********************* RF driver private functions implementation ************************** */

/**
 * @brief Initializes the RAIL API
 * 
 * @return 0 on success, -1 on failure
 */
static void _rf_init_efr32_RAIL(){
    RAIL_Status_t status;

    irq_thread.start(mbed::callback(_rf_irq_thread_task));

    rf_timer.start();

    // Initialize the RAIL library and any internal state it requires
    rail_handle = RAIL_Init(&rail_config, &_rf_ready_cb);
    

    // Configure the TRX mode to packet mode
    RAIL_DataConfig_t data_config = {
        .txSource = TX_PACKET_DATA,
        .rxSource = RX_PACKET_DATA,
        .txMethod = PACKET_MODE,
        .rxMethod = PACKET_MODE,
    };

    status = RAIL_ConfigData(rail_handle, &data_config);

    if ( status != RAIL_STATUS_NO_ERROR ){
        tr_error("RAIL_ConfigData failed: %d\n", status);
    }

    // Set the channel RF configuration structure
    RAIL_ConfigChannels(rail_handle, &Protocol_Configuration_channelConfig, NULL);

    // Configure calibration settings
    RAIL_ConfigCal(rail_handle, RAIL_CAL_ALL);

    // Tell RAIL the internal state transitions for TRX events
    RAIL_StateTransitions_t tx_transitions = {
        .success = RAIL_RF_STATE_RX,
        .error = RAIL_RF_STATE_RX
    };
    RAIL_StateTransitions_t rx_transitions = {
        .success = RAIL_RF_STATE_RX,
        .error = RAIL_RF_STATE_RX
    };
    status = RAIL_SetTxTransitions(rail_handle, &tx_transitions);

    if ( status != RAIL_STATUS_NO_ERROR ){
        tr_error("RAIL_SetTxTransitions failed: %d\n", status);
    }

    status = RAIL_SetRxTransitions(rail_handle, &rx_transitions);

    if ( status != RAIL_STATUS_NO_ERROR ){
        tr_error("RAIL_SetRxTransitions failed: %d\n", status);
    }

    // Assign the buffers for TRX fifos
    uint16_t allocated_fifo_size = 0;
    allocated_fifo_size = RAIL_SetTxFifo(rail_handle, tx_fifo, 0, FIFO_SIZE);
    tr_info("Allocated TX FIFO size: %d", allocated_fifo_size);
    RAIL_SetRxFifo(rail_handle, rxFifo, &allocated_fifo_size);
    tr_info("Allocated RX FIFO size: %d", allocated_fifo_size);

    // Configure the events we want to catch in IRQ
    RAIL_ConfigEvents(rail_handle, RAIL_EVENTS_ALL,
                     RAIL_EVENTS_TX_COMPLETION | RAIL_EVENTS_RX_COMPLETION |
                     RAIL_EVENT_RX_FIFO_ALMOST_FULL | RAIL_EVENT_RX_FIFO_OVERFLOW |
                     RAIL_EVENT_RX_SYNC1_DETECT);

    // Set frame length to variable length mode
    RAIL_SetFixedLength(rail_handle, RAIL_SETFIXEDLENGTH_INVALID);

    // Set the PA configuration
    if(RAIL_ConfigTxPower(rail_handle, &tx_power_config) != RAIL_STATUS_NO_ERROR){
        // Error: The PA could not be initialized due to an improper configuration.
        // Please ensure your configuration is valid for the selected part.
        tr_error("PA init failed -> stopping execution\n");
        while (1) ;
    }

    RAIL_EnablePaCal(true);
    RAIL_SetTxPower(rail_handle, RAIL_TX_POWER_LEVEL_SUBGIG_HP_MAX);
    

    /* Setup Packet Trace Interface
     * PTI 0 : PTI.CLK (DCLK) = PB11, PTI.DATA (DOUT) = PB12, PTI.SYNC (FRAME) = PB13
     */
    RAIL_PtiConfig_t ptiConfig = {
        RAIL_PTI_MODE_UART,
        1615384,
        6,  // Alternate function location (see datasheet)
        gpioPortB,
        12,
        6,
        gpioPortB,
        11,
        6,
        gpioPortB,
        13
    };

    // Initialize the Packet Trace Interface (PTI) for the EFR32
    RAIL_ConfigPti(rail_handle, &ptiConfig);
    RAIL_SetPtiProtocol	(rail_handle, RAIL_PTI_PROTOCOL_CUSTOM);
    RAIL_EnablePti(rail_handle, true);

}

/**
 * @brief Callback for RAIL initialization completion
 * 
 * @param[in] handle Handle of the current RAIL instance
 */
static void _rf_ready_cb(RAIL_Handle_t handle){
    (void) handle;
    driver_state = DRVSTATE_IDLE;
    EFR32_TR_DEBUG("Radio Ready");
}

/**
 * @brief Activates the interface.
 * 
 * @param[in] new_channel The channel on which the interface should listen when it is started
 * @param[in] filtering Not use for IEEE 802.15.4-2015 frames because filtering is done by Nanostack
 * @return 0 on success, -1 if channel could not be set
 * 
 * When the interface is started, it will listen on the given channel.
 */
static int _rf_interface_up(int8_t new_channel){
    if (_rf_check_and_change_channel(new_channel) == -1){
        return -1;
    }
    _rf_set_idle(RADIO_SHUTDOWN);
    driver_state = DRVSTATE_IDLE;
    if (!sleep_blocked) {
        /* RX can only happen in EM0/1*/
        sleep_manager_lock_deep_sleep();
        sleep_blocked = true;
    }
    return 0;
}

/**
 * @brief Shuts interface down.
 * 
 * The radio is deactivated and the interface is not listening on any channel.
 */
static void _rf_reset_interface(enum idle_state state){
    // Put RAIL to idle state
    _rf_set_idle(state);
    driver_state = DRVSTATE_IDLE;
    if (sleep_blocked) {
        sleep_manager_unlock_deep_sleep();
        sleep_blocked = false;
    }    
}

/**
 * @brief Changes the active channel of the radio with state conflict avoidance
 * 
 * @param[in] new_channel New channel to switch to
 * @return 0 on success, -1 on failure (channel not changed because radio is
 * emitting or receiving a packet)
 */
static int _rf_check_and_change_channel(int8_t new_channel){
    if (driver_state == DRVSTATE_TX || driver_state == DRVSTATE_RX){
        return -1;
    }
    if (new_channel != channel && new_channel != -1) {
        channel = new_channel;
    }
    return 0;
}

/**
 * @brief Changes or checks a new RF channel configuration (no state verification).
 * 
 * @param[in] config The new RF configuration to use
 * @param[in] only_check If true, only checks if the new configuration is valid regarding the configuration supported by the driver
 * @return 0 on success, -1 on failure (config not valid)
 * 
 * @note IMPORTANT : this function does not check the state of the radio before applying the new configuration (use @ref _rf_update_phy_config if required).
 * 
 * The configuration that are supported by the driver are:
 * - Wi-SUN option EU-868-1a
 * - Wi-SUN option EU-868-2a
 * - Wi-SUN option EU-868-3
 * - Wi-SUN option EU-873-1a
 * - Wi-SUN option EU-873-2a
 * - Wi-SUN option EU-873-3
 * - Wi-SUN option IN-866-1a
 * - Wi-SUN option IN-866-2a
 * - Wi-SUN option IN-866-3
 * - Wi-SUN option NA-915-1b
 * - Wi-SUN option NA-915-2a
 * - Wi-SUN option NA-915-3
 * - Wi-SUN option NA-915-4a
 * - Wi-SUN option NA-915-5
 * - Wi-SUN option JP-920-2b
 * - Wi-SUN option JP-920-4b
 * - Wi-SUN option ECHONET JP-920-1b
 * - Wi-SUN option ECHONET JP-920-2b
 * - Wi-SUN option CN-470-1b
 */
static int _rf_check_and_apply_rf_config(phy_rf_channel_configuration_s *config, bool only_check){

    if (!(config->modulation == M_2FSK || config->modulation == M_GFSK)){
        return -1;
    }

    int i = 0;
    while(wisun_rail_configs[i].name != NULL){
        if (wisun_rail_configs[i].channel_config.channel_0_center_frequency == config->channel_0_center_frequency
            && wisun_rail_configs[i].channel_config.channel_spacing == config->channel_spacing
            && wisun_rail_configs[i].channel_config.datarate == config->datarate
            && wisun_rail_configs[i].channel_config.number_of_channels == config->number_of_channels
            && wisun_rail_configs[i].channel_config.modulation == config->modulation
            && wisun_rail_configs[i].channel_config.modulation_index == config->modulation_index){
            if (!only_check){
                wisun_rail_configs[i].set_config_func();
                RAIL_ConfigChannels(rail_handle, &Protocol_Configuration_channelConfig, NULL);
                EFR32_TR_DEBUG("Found and applied configuration: %s", wisun_rail_configs[i].name);
            }
            return 0;
        }
        i++;
    }
    
    tr_warn("No RF configuration has been found for the given parameters. Leaving the current configuration.");
    return -1;
}

/**
 * @brief Configure the radio for the given configuration (with state verification).
 * 
 * @param[in] new_config The new RF configuration to use.
 * @return 0 on success, -1 on failure (configuration is not valid). 
 */
static int _rf_update_phy_config(phy_rf_channel_configuration_s *new_config){
    if (_rf_check_and_apply_rf_config(new_config, true) == 0){
        EFR32_TR_DEBUG("Requested PHY configuration is valid");

        cur_phy_config.datarate = new_config->datarate;
        cur_phy_config.channel_spacing = new_config->channel_spacing;
        cur_phy_config.channel_0_center_frequency = new_config->channel_0_center_frequency;
        cur_phy_config.number_of_channels = new_config->number_of_channels;
        cur_phy_config.modulation = new_config->modulation;
        cur_phy_config.modulation_index = new_config->modulation_index;
        _rf_update_symbol_rate(cur_phy_config.datarate, cur_phy_config.modulation);

        if (driver_state == DRVSTATE_IDLE){
            EFR32_TR_DEBUG("Applying new PHY configuration");
            _rf_apply_phy_config();
        } else {
            EFR32_TR_DEBUG("Requesting PHY configuration update");
            phy_config_update_requested = true;
        }

        return 0;

    } else {
        tr_error("Requested PHY configuration is INVALID. Leaving previous configuration unchanged");
        return -1;
    }
    
}

/**
 * @brief Creates and applies a new RF configuration based on the PHY configuration.
 */
static void _rf_apply_phy_config(){
    _rf_set_idle(RADIO_IDLE_ABORT);    

    _rf_check_and_apply_rf_config(&cur_phy_config, false);

    phy_config_update_requested = false;

    EFR32_TR_DEBUG("PHY config has been applied");
}

/**
 * @brief Calculates and updates the symbol rate based on the datarate and the modulation.
 * 
 * @param[in] datarate The current dataratee.
 * @param[in] modulation The modulation that is used (not used here because only 2-FSK is supported).
 */
static void _rf_update_symbol_rate(uint32_t baudrate, phy_modulation_e modulation)
{
    (void) modulation;
    uint8_t bits_in_symbols = 1;
    rf_symbol_rate = baudrate / bits_in_symbols;
    MAX_PACKET_SENDING_TIME = (uint32_t)(8000000/cur_phy_config.datarate)*FIFO_SIZE + PACKET_SENDING_EXTRA_TIME;
}

/**
 * @brief Checks if the radio is busy (i.e. transmitting or receiving a packet).
 * 
 * @return true if the radio is busy, false otherwise.
 */
static bool _rf_is_busy(){
    if (driver_state == DRVSTATE_IDLE || driver_state == DRVSTATE_IDLE_WAITING_RX){
        return false;
    } else {
        return true;
    }
}

/**
 * @brief Sets the radio to idle state.
 * 
 * @param[in] mode Specifies if the radio must abort ongoing operation (TRX) or must shutdown (see @ref idle_state).
 */
static void _rf_set_idle(enum idle_state mode){
    if (mode == RADIO_IDLE){
        RAIL_Idle(rail_handle, RAIL_IDLE, true);
    } else if (mode == RADIO_IDLE_ABORT){
        RAIL_Idle(rail_handle, RAIL_IDLE_ABORT, true);
    } else if (mode == RADIO_SHUTDOWN){
        RAIL_Idle(rail_handle, RAIL_IDLE_FORCE_SHUTDOWN_CLEAR_FLAGS, true);
    } else {
        RAIL_Idle(rail_handle, RAIL_IDLE, true);
    } 
}

/**
 * @brief Sets the to receive mode (with state verification).
 * 
 * @return 0 on success, > 0 on failure. 
 * 
 * @note The return value can be casted to @ref RAIL_Status_t to get the exact error code.
 */
static int _rf_set_rx(){ //
    RAIL_Status_t status;
    if (driver_state == DRVSTATE_TX) {
        return (int)RAIL_STATUS_INVALID_STATE;
    }

    /*Set channel*/
    _rf_check_and_change_channel(new_channel); 

    if(phy_config_update_requested){
        phy_config_update_requested = false;
        _rf_apply_phy_config();
    }

    status = RAIL_StartRx(rail_handle, channel, NULL);
    if (driver_state != DRVSTATE_CSMA_STARTED) {
        driver_state = DRVSTATE_IDLE_WAITING_RX;
    }

    _rf_poll_state_change(DRVSTATE_IDLE_WAITING_RX);
    return (int)status;
}

/**
 * @brief Waits for the radio to enter the given state.
 *
 * @param[in] expected_drv_state The state that the radio must be in.
 */
static void _rf_poll_state_change(enum driver_state expected_drv_state)
{
    RAIL_RadioState_t new_radio_state = RAIL_RF_STATE_INACTIVE;
    uint16_t break_counter = 0;

    switch(expected_drv_state){
        case DRVSTATE_IDLE:
            new_radio_state = RAIL_RF_STATE_IDLE;
            break;
        case DRVSTATE_IDLE_WAITING_RX:
            new_radio_state = RAIL_RF_STATE_RX;
            break;
        case DRVSTATE_RX:
            new_radio_state = RAIL_RF_STATE_RX_ACTIVE;
            break;
        case DRVSTATE_TX:
            new_radio_state = RAIL_RF_STATE_TX_ACTIVE;
            break;
        default:
            break;
    }

    while (new_radio_state != RAIL_GetRadioState(rail_handle)) {
        if (break_counter++ == 0xffff) {
            tr_err("Failed to change state from %x to: %x", RAIL_GetRadioState(rail_handle), new_radio_state);
            break;
        }
    }
}

/**
 * @brief Writes a packet to the radio TX fifo.
 * 
 * @param dataPtr A pointer to the buffer containing the packet to be sent.
 * @param length Length of the packet to be sent (including PHY header (PHR) length).
 * @param reset A flag to tell if the TX fifo must be reset before writing the packet.
 * @return The number of bytes written to the TX fifo (should normally be equal to the length of the packet). 
 */
static uint16_t _rf_write_tx_fifo(uint8_t *dataPtr, uint16_t length, bool reset){
    uint16_t nb_bytes_written;
    return nb_bytes_written = RAIL_WriteTxFifo(rail_handle, dataPtr, length, reset);
}

/**
 * @brief Sends the packet stored inside the TX fifo.
 * 
 * @return 0 on success, > 0 on failure.
 * 
 * This function is non-blocking. It only sets the radio to TX mode. The end of transmission is handled
 * by the radio IRQ handler @ref _rf_rail_events_irq_handler with the RAIL_EVENTS_TX_COMPLETION flag.
 * 
 * @note The return value can be casted to @ref RAIL_Status_t to get the exact error code.
 */
static int _rf_execute_tx(){
    EFR32_TR_DEBUG("Execute TX on channel %d", channel);

    // TX FIFO must be pre-filled with TX data
    platform_enter_critical();

    driver_state = DRVSTATE_TX;
    RAIL_TxOptions_t txOpt = RAIL_TX_OPTIONS_DEFAULT;
    RAIL_Status_t status;

    _rf_set_idle(RADIO_IDLE_ABORT);

    status = RAIL_StartTx(rail_handle, channel, txOpt, NULL);

    if (status != RAIL_STATUS_NO_ERROR){
        // TX error
        _rf_set_idle(RADIO_IDLE_ABORT);
        driver_state = DRVSTATE_IDLE;
        platform_exit_critical();
        return (int)status;
    }   

    _rf_start_backup_timer(MAX_PACKET_SENDING_TIME);

    platform_exit_critical(); 
    return (int)RAIL_STATUS_NO_ERROR;
}

/**
 * @brief Checks if the current channel is clear to send.
 * 
 * @return true if the channel is clear to send, false otherwise.
 */
static bool _rf_check_cca(){   
    if(!cca_enabled){
        return true;
    }

    int16_t rssi; 
    unsigned int tries = 0;
    EFR32_TR_DEBUG("check_cca()");
    if(!_rf_set_rx()){ //
        EFR32_TR_DEBUG("Could not start RX mode");
    }

    do {
        rssi = RAIL_GetRssi(rail_handle, true);
        tries++;
    } while(rssi == RAIL_RSSI_INVALID && tries < 100);

    if(tries >= 100){
        EFR32_TR_DEBUG("Could get RSSI value for CCA");
        return false;
    }

    rssi /= 4;  // RAIL_GetRssi returns dbm*4
    EFR32_TR_DEBUG("tries=%u rssi=%d rssi_cca_threshold=%d", tries, rssi, rssi_cca_threshold);
    if(rssi > rssi_cca_threshold){
        return false;
    } else {
        return true;
    }
}


/**
 * @brief Handles the end of packet reception by sending the received packets to the MAC layer.
 * 
 * This function is called by the IRQ thread (see @ref _rf_irq_thread_task) when the radio IRQ handler (see @ref _rf_rail_events_irq_handler)
 * has processed a new packet and added it to the RX queue. Because multiple packets can be received before the IRQ thread is able to call
 * this function, a queue is required to store the received packets before handling them.
 * 
 * @note This function sends only the IEEE 802.15.4-2015 compliant packets. All packets that have an older version are discarded.
 */
static void _rf_handle_rx_end(){
    while (rx_queue_tail != rx_queue_head) {
        // Handle every packet that are available in the queue

        struct rx_queue_slot *rx_slot = &rx_queue[rx_queue_tail];

        rx_queue_tail = (rx_queue_tail + 1) % RX_QUEUE_SLOTS;

        cur_pkt_rx_sync_word_time = rx_slot->rx_time;

        if (((rx_slot->buffer[IEEE_FRAME_CONTROL_BYTE_1 + device_driver.phy_header_length] & IEEE_FRAME_VERSION_MASK) >> IEEE_FRAME_VERSION_SHIFT) == IEEE_VERSION_2015){
            if (device_driver.phy_rx_cb)
                device_driver.phy_rx_cb(&rx_slot->buffer[device_driver.phy_header_length],
                                        rx_slot->length - device_driver.phy_header_length,
                                        rx_slot->lqi,
                                        rx_slot->rssi,
                                        rf_radio_driver_id);
        } else {
                // Driver does not support IEEE 802.15.4-2006/2003 frames
                printf("Driver does not support IEEE 802.15.4-2006/2003 frames\n");
        }        
    }
}

/**
 * @brief IRQ handler for the RAIL events generated by the radio.
 * 
 * @param[in] railHandle Handle to the current RAIL instance.
 * @param[in] events 64-bits mask of the events that have been generated (see @ref RAIL_Events_t).
 */
static void _rf_rail_events_irq_handler(RAIL_Handle_t railHandle,
                            RAIL_Events_t events){
    if (railHandle != rail_handle) {
        // It is not our RAIL instance : ignore the event
        return;
    }

    if (events & RAIL_EVENT_RX_SYNC1_DETECT){
        driver_state = DRVSTATE_RX;
        last_pkt_rx_sync_word_time = rf_get_timestamp();
        _rf_start_backup_timer(MAX_PACKET_SENDING_TIME);
    }

    if (events & RAIL_EVENTS_RX_COMPLETION){
        _rf_stop_backup_timer();

        if (events & RAIL_EVENT_RX_PACKET_RECEIVED){
            RAIL_RxPacketInfo_t packet_info;
            RAIL_RxPacketHandle_t packet_handle = RAIL_GetRxPacketInfo(rail_handle,
                                                                    RAIL_RX_PACKET_HANDLE_NEWEST,
                                                                    &packet_info
                                                                    );

            /* Only process the packet if it had a correct CRC */
            if (packet_info.packetStatus == RAIL_RX_PACKET_READY_SUCCESS) {
                /* Get RSSI and LQI information about this packet */
                RAIL_RxPacketDetails_t packet_details;
                packet_details.timeReceived.timePosition = RAIL_PACKET_TIME_DEFAULT;
                packet_details.timeReceived.totalPacketBytes = 0;
                RAIL_GetRxPacketDetails(rail_handle, packet_handle, &packet_details);

                /* Drop this packet if the RX queue is out of space */
                if (((rx_queue_head + 1) % RX_QUEUE_SLOTS) == rx_queue_tail) {
                    rx_fifo_irq_event = RX_QUEUE_FULL;
                    irq_thread.flags_set(SIG_RX_FIFO_EVENTS); 
                    RAIL_ReleaseRxPacket(rail_handle, packet_handle);
                }

                /* Get the next available slot in the RX queue */
                struct rx_queue_slot *rx_slot = &rx_queue[rx_queue_tail];

                /* Copy packet payload from circular FIFO into the RX queue slot */
                RAIL_CopyRxPacket(rx_slot->buffer, &packet_info);

                /* Release RAIL resources early */
                RAIL_ReleaseRxPacket(rail_handle, packet_handle);

                /* Store the packet parameters along with the packet payload in the queue slot */
                rx_slot->rssi = packet_details.rssi;
                rx_slot->lqi = packet_details.lqi;
                rx_slot->length = packet_info.packetBytes;
                rx_slot->rx_time = last_pkt_rx_sync_word_time;

                rx_queue_head = (rx_queue_head + 1) % RX_QUEUE_SLOTS;
                irq_thread.flags_set(SIG_PACKET_RECEIVED);
            } else {
                irq_thread.flags_set(SIG_RADIO_ERROR);
                radio_error_packet_status = packet_info.packetStatus;
            }
        }
        driver_state = DRVSTATE_IDLE_WAITING_RX;
    }

    if (events & RAIL_EVENTS_TX_COMPLETION){
        _rf_stop_backup_timer();
        _rf_set_idle(RADIO_IDLE_ABORT);
        driver_state = DRVSTATE_IDLE_WAITING_RX;
        _rf_set_rx(); //
        irq_thread.flags_set(SIG_PACKET_SENT);
    }

    if (events & RAIL_EVENT_RX_FIFO_ALMOST_FULL){
        rx_fifo_irq_event = RX_QUEUE_ALMOST_FULL;
        irq_thread.flags_set(SIG_RX_FIFO_EVENTS);
    }

    if (events & RAIL_EVENT_RX_FIFO_OVERFLOW){
        rx_fifo_irq_event = RX_QUEUE_OVERFLOW;
        irq_thread.flags_set(SIG_RX_FIFO_EVENTS);
    }    
}

/**
 * @brief This function is the task assigned to the IRQ thread.
 * 
 * This task is responsible for taking information out of the IRQ context.
 * 
 */
static void _rf_irq_thread_task(){
    while (1) {
    uint32_t flags = ThisThread::flags_wait_any(SIG_ALL);
        if (flags & SIG_RADIO_ERROR) {
            const char * const error_str[] = {
                [RAIL_RX_PACKET_NONE] = "RAIL_RX_PACKET_NONE",
                [RAIL_RX_PACKET_ABORT_FORMAT] = "RAIL_RX_PACKET_ABORT_FORMAT",
                [RAIL_RX_PACKET_ABORT_FILTERED]  = "RAIL_RX_PACKET_ABORT_FILTERED",
                [RAIL_RX_PACKET_ABORT_ABORTED]  = "RAIL_RX_PACKET_ABORT_ABORTED",
                [RAIL_RX_PACKET_ABORT_OVERFLOW]  = "RAIL_RX_PACKET_ABORT_OVERFLOW",
                [RAIL_RX_PACKET_ABORT_CRC_ERROR]  = "RAIL_RX_PACKET_ABORT_CRC_ERROR",
                [RAIL_RX_PACKET_READY_CRC_ERROR]  = "RAIL_RX_PACKET_READY_CRC_ERROR",
                [RAIL_RX_PACKET_READY_SUCCESS] = "RAIL_RX_PACKET_READY_SUCCESS",
                [RAIL_RX_PACKET_RECEIVING] = "RAIL_RX_PACKET_RECEIVING",
            };
            tr_error("RADIO_ERROR: %s", error_str[radio_error_packet_status]);
        }
        if (flags & SIG_CSMA_TIMEOUT) {
            _rf_csma_timeout_handler();
        }
        if (flags & SIG_TIMER_BACKUP) {
            _rf_backup_timeout_handler();
        }
        if (flags & SIG_RX_FIFO_EVENTS) {
            switch(rx_fifo_irq_event){
                case RX_QUEUE_ALMOST_FULL:
                    tr_warn("/!\\ RX queue almost full");
                    break;
                case RX_QUEUE_FULL:
                    tr_error("/!\\ RX queue is full (packet dropped)");
                    break;
                case RX_QUEUE_OVERFLOW:
                    tr_error("/!\\ RX queue overflow (packet dropped)");
                    break;
            }            
        }
        if (flags & SIG_PACKET_RECEIVED) {
            _rf_handle_rx_end();
        }
        if (flags & SIG_PACKET_SENT) {
            /*
            const char * const error_str[] = {
                [DRVSTATE_RADIO_UNINIT] = "DRVSTATE_RADIO_UNINIT",
                [DRVSTATE_RADIO_INITING] = "DRVSTATE_RADIO_INITING",
                [DRVSTATE_IDLE]  = "DRVSTATE_IDLE",
                [DRVSTATE_TX]  = "DRVSTATE_TX",
                [DRVSTATE_RX]  = "DRVSTATE_RX",
                [DRVSTATE_CSMA_STARTED]  = "DRVSTATE_CSMA_STARTED",
            };*/

            // TX success
            if (device_driver.phy_tx_done_cb)
                device_driver.phy_tx_done_cb(rf_radio_driver_id, 
                                                        mac_tx_handle, PHY_LINK_TX_SUCCESS, 0, 0);   
        }
    }
}

/**
 * @brief Starts the CSMA-CA timer
 * 
 * @param[in] duration before triggering the CSMA-CA timeout callback
 */
static void _rf_start_csma_timeout_timer(uint32_t duration){
    csma_timeout_timer.attach(_rf_csma_timeout_timer_cb, microseconds(duration));
}

/**
 * @brief Stops the CSMA-CA timer 
 */
static void _rf_stop_csma_timeout_timer(){
    csma_timeout_timer.detach();
}

/**
 * @brief CSMA-CA timeout callback 
 */
static void _rf_csma_timeout_timer_cb(){
    irq_thread.flags_set(SIG_CSMA_TIMEOUT);
}

/**
 * @brief Starts/restarts the CSMA-CA timer based on the requested backoff time * 
 */
static void _rf_rearm_csma_timeout_timer(){
    uint32_t csma_ca_period = backoff_time - rf_get_timestamp();

    if(csma_ca_period > 65000){
        // Time has already passed, trigger the IRQ immediately for immediate TX
        csma_ca_period = 1;
    } 

    _rf_start_csma_timeout_timer(csma_ca_period);
}


/**
 * @brief CSMA-CA timeout handler called by the IRQ thread task
 * 
 * This function tells to the MAC layer that the CSMA-CA timeout has occurred.
 * The MAC layer is responsible to tell what to do next. Depending on the status
 * that the MAC layer returns, this function will either cancel the transmission,
 * start the transmission or start the CSMA-CA procedure again.
 */
static void _rf_csma_timeout_handler(){
    int8_t status = -1;

    // Request CSMA status from Nanostack
    if (device_driver.phy_tx_done_cb)
        status = device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_PREPARE, 0, 0);
    
    switch (status){
    case PHY_TX_NOT_ALLOWED:
        if (driver_state == DRVSTATE_CSMA_STARTED) {
            _rf_set_idle(RADIO_IDLE_ABORT);
            driver_state = DRVSTATE_IDLE;
            _rf_set_rx();
        }
        break;   
    case PHY_RESTART_CSMA:
        if (driver_state == DRVSTATE_RX){
            if (device_driver.phy_tx_done_cb)
                device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_FAIL_RX, 0, 0);
        } else if (!_rf_check_cca()) {
            // Channel is busy
            if (device_driver.phy_tx_done_cb)
                device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_FAIL, 0, 0);
        } else {
            // Channel is clear
            if (device_driver.phy_tx_done_cb)
                device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_OK, 0, 0);

            if (backoff_time > 0) {
                _rf_rearm_csma_timeout_timer();
            }
        }
        break;   
    case PHY_TX_ALLOWED:
        if (driver_state == DRVSTATE_RX){
            if (device_driver.phy_tx_done_cb)
                device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_FAIL_RX, 0, 0);
        } else if(!_rf_check_cca()){
            EFR32_TR_DEBUG("Channel is busy");
            // Channel is busy
            if (device_driver.phy_tx_done_cb)
                device_driver.phy_tx_done_cb(rf_radio_driver_id, 
                                                                mac_tx_handle, PHY_LINK_CCA_FAIL, 0, 0);
        } else {
            EFR32_TR_DEBUG("Channel is clear");
            // Channel is clear -> start transmission immediately
            if (_rf_execute_tx() != 0){
                EFR32_TR_DEBUG("TX fail");
                if (device_driver.phy_tx_done_cb)
                    device_driver.phy_tx_done_cb(rf_radio_driver_id, 
                                                                mac_tx_handle, PHY_LINK_TX_FAIL, 0, 0);
            }
            // TX success is called in _rf_rail_events_irq_handler()
        }
        break;  
    default:
        break;
    }
}

/**
 * @brief Starts the backup timer
 * 
 * @param[in] duration before triggering the backup timeout callback
 * 
 * The backup timer is responsible to put the radio in a known state in case of unexpected radio failure.
 */
void _rf_start_backup_timer(uint32_t duration){
    backup_timeout_timer.attach(_rf_backup_timer_cb, microseconds(duration));
}

/**
 * @brief Stops the backup timer 
 */
void _rf_stop_backup_timer(){
    backup_timeout_timer.detach();
}

/**
 * @brief Backup timeout callback 
 */
void _rf_backup_timer_cb(){
    irq_thread.flags_set(SIG_TIMER_BACKUP);
}

/**
 * @brief Backup timeout handler called by the IRQ thread task
 */
static void _rf_backup_timeout_handler(){
    if (driver_state == DRVSTATE_TX) {
        // Radio is stuck in TX state, reset it
        if (device_driver.phy_tx_done_cb) {
            device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_TX_FAIL, 0, 0);
        }
        driver_state = DRVSTATE_IDLE;
        _rf_set_idle(RADIO_IDLE_ABORT);
        _rf_set_rx();
    } else if (driver_state == DRVSTATE_RX){
        driver_state = DRVSTATE_IDLE_WAITING_RX;
    }
}



/* *******************************
 * Nanostack backend related functions *
 * *******************************
 */

/**
 * @brief Gets the MAC address of the PHY
 * 
 * @param[out] mac A pointer to an 8 bytes buffer where the MAC address will be stored 
 */
void Efr32RfDriver::get_mac_address(uint8_t *mac)
{
    platform_enter_critical();

    memcpy(mac, mac_address, sizeof(mac_address));

    platform_exit_critical();
}

/**
 * @brief Sets the MAC address of the PHY
 * 
 * @param[out] mac A pointer to an 8 bytes buffer containing the MAC address to be set  
 */
void Efr32RfDriver::set_mac_address(uint8_t *mac)
{
    EFR32_TR_DEBUG("set_mac_address: MAC");
    platform_enter_critical();

    if (NULL != efr32RfDriver) {
        error("Efr32RfDriver cannot change mac address when running");
        platform_exit_critical();
        return;
    }

    memcpy(mac_address, mac, sizeof(mac_address)); 
    _mac_is_set = true;   

    platform_exit_critical();

#ifdef EFR32_DEBUG
    tr_debug("set_mac_address: MAC\n");
    for (int i = 0; i < sizeof(mac_address); i++) {
        tr_debug("%02x:", mac_address[i]);
    }
    tr_debug("\n");
#endif
}

/**
 * @brief Gets the PHY radio version
 * 
 * @return The PHY radio version
 */
uint32_t Efr32RfDriver::get_driver_version()
{
    RAIL_Version_t railversion;
    RAIL_GetVersion(&railversion, false);

    return (railversion.major << 24) |
           (railversion.minor << 16) |
           (railversion.rev   << 8)  |
           (railversion.build);
}

/**
 * @brief Register and initializes the PHY driver into Nanostack
 * 
 * @return The radio driver ID on success, -1 if the PHY driver is already registered 
 */
int8_t Efr32RfDriver::rf_register()
{
    platform_enter_critical();

    if (efr32RfDriver != NULL) {
        platform_exit_critical();
        error("Multiple registrations of Efr32RfDriver not supported");
        return -1;
    }

    if (!_mac_is_set) {
        uint8_t new_mac[8];

        tr_warn("No MAC has been assigned yet -> generating random MAC");
        randLIB_seed_random();
        randLIB_get_n_bytes_random(new_mac, 8);
        new_mac[0] |= 2; //Set Local Bit
        new_mac[0] &= ~1; //Clear multicast bit

        set_mac_address(new_mac);
    }

    int8_t radio_id = rf_device_register();
    if (radio_id < 0) {
        efr32RfDriver = NULL;
    } else {
        efr32RfDriver = this;
    }

    platform_exit_critical();
    return radio_id;
}

/**
 * @brief Removes the PHY driver from Nanostack
 */
void Efr32RfDriver::rf_unregister()
{
    platform_enter_critical();

    if (efr32RfDriver != this) {
        platform_exit_critical();
        return;
    }

    rf_device_unregister();
    efr32RfDriver = NULL;

    platform_exit_critical();
}

/**
 * @brief Constructs a new Efr32RfDriver object 
 */
Efr32RfDriver::Efr32RfDriver() : NanostackRfPhy(), _mac_is_set(false)
{
    // Do nothing
}

/**
 * @brief Destroys the Efr32Driver object
 * 
 */
Efr32RfDriver::~Efr32RfDriver()
{
    rf_unregister();
}

/**
 * @brief Retrieves the instance of the Efr32 PHY driver
 * 
 * @return Reference to the Efr32RfDriver instance
 */
NanostackRfPhy &NanostackRfPhy::get_default_instance()
{
    static Efr32RfDriver rf_phy;
    return rf_phy;
}