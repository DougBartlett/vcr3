/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <stdbool.h>
#include <em_common.h>
#include <em_cmu.h>
#include <em_prs.h>
#include <em_device.h>
#include <em_rtcc.h>
#include <em_gpio.h>
#include <nvm3_default.h>
#include <sl_status.h>
#include <sl_sleeptimer.h>
#include <sl_i2cspm_instances.h>
#include <sl_bluetooth.h>
#include <stdio.h>
#include <app_log.h>
#include <app_assert.h>
#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif // SL_COMPONENT_CATALOG_PRESENT
#include <sl_bt_api.h>
#include <sl_simple_button.h>
#include <sl_simple_button_instances.h>
#ifdef SL_CATALOG_CLI_PRESENT
#include "sl_cli.h"
#endif // SL_CATALOG_CLI_PRESENT
#include "app.h"
#include "vcr.h"
#include "gatt_db.h"



#define SCAN_DURATION   10000
#define ADV_DURATION    30000
#define MAXCONNECTIONS 4
#define NVM3_VCR_GLOVE_BONDING_KEY    74559
#define FIXEDLATENCYADDER     7                           // Additional remote glove latency (in ticks) due to s/w processing overhead.  Benchtop characterization was used to determine this value.
                                                          // Note that any printf or app_log_XXXX calls will add significantly to this additional latency, so AVOID these...

struct connection
  { uint8_t  isOpen;
    uint8_t  isGlove;
    bd_addr  btAddress;
    uint8_t  btAddressType;
    char     btAddressString[24];
    uint8_t  bonding;
  } connectionList[MAXCONNECTIONS];

bool gloveConnected, localVCRStarted, buttonReleasedNoHold, buttonReleasedAfter2Sec, buttonReleasedAfter5Sec, buttonReleasedAfter10Sec;
uint32_t connIntervalInTicks;
static sl_sleeptimer_timer_handle_t advertiserStopTimer, scannerStopTimer;
// static sl_status_t send_notification(uint16_t attribute);

static const char *securityModeNames[] = { "No Security", "Unauthenticated pairing with encryption", "Authenticated pairing with encryption", "Authenticated pairing with encryption using 128bit key" };
// static const char *addressTypeStrings[2] = { "public device", "static random" };
static char obuffer[512];
static uint8_t advertising_set_handle = 0xff;                                           // The advertising set handle allocated from Bluetooth stack
// static bool notifyCommandResult;
static bool allowNonBonded = false;
static uint8array vcrServiceUUID =                     { 16, { 0xD8, 0xC8, 0x6A, 0xC7, 0xBE, 0xDC, 0x23, 0xAA, 0x9A, 0x41, 0x8C, 0xA8, 0xC2, 0x5B, 0xD2, 0x63 } };
static uint8array vcrVibrationSetCharacteristicUUID  = { 16, { 0x57, 0x65, 0xDA, 0xB2, 0x93, 0xB3, 0x59, 0x9B, 0xA1, 0x43, 0x9B, 0x1D, 0xE3, 0x8F, 0xE3, 0x09 } };
static uint16_t vcrVibrationSetCharacteristic;
//static uint8array vcrCommandCharacteristicUUID       = { 16, { 0x32, 0xae, 0xa6, 0x1b, 0x01, 0xa7, 0x90, 0x8a, 0x72, 0x49, 0x1f, 0xdf, 0x5c, 0x34, 0x75, 0x44 } };
//static uint8array vcrCommandResultCharacteristicUUID = { 16, { 0x7c, 0x6d, 0x21, 0xb8, 0x6f, 0xc3, 0xaa, 0x96, 0x0e, 0x4e, 0x60, 0xee, 0x8f, 0xa9, 0x0b, 0xed } };

int16_t findGloveConnection(void)
  { for(uint16_t c = 0; c < MAXCONNECTIONS; c++)
        if(connectionList[c].isGlove) return(c);
    return(-1);
  }

int16_t uuid128cmp(uint8_t *a1, uint8_t *a2)
  { for(uint16_t i = 0; i < 16; i++)
        if(a1[i] != a2[i]) return(1);
    return(0);
  }

int16_t bd_addr_cmp(const bd_addr a1, const bd_addr a2)
  { for(uint16_t i = 0; i < 6; i++)
        if(a1.addr[i] != a2.addr[i]) return(1);
    return(0);
  }

void bd_addr_to_string(bd_addr address, char *str, uint16_t maxlen)
  { snprintf(str, maxlen, "%.2X:%.2X:%.2X:%.2X:%.2X:%.2X", address.addr[5], address.addr[4],
                            address.addr[3], address.addr[2], address.addr[1], address.addr[0]);
  }

uint16_t string_to_bd_addr(char *str, bd_addr *address)
  { uint16_t nf = sscanf(str, "%2hhX:%2hhX:%2hhX:%2hhX:%2hhX:%2hhX", &address->addr[5], &address->addr[4],
                         &address->addr[3], &address->addr[2], &address->addr[1], &address->addr[0]);
    return(nf == 6 ? SL_STATUS_OK : SL_STATUS_FAIL);
  }

uint16_t uuid_cmp(const uint8array *uuid1, const uint8array *uuid2)
  { if(uuid1->len != uuid2->len) return(1);
    for(uint16_t i = 0; i < uuid1->len; i++)
        if(uuid1->data[i] != uuid2->data[i]) return(1);
    return(0);
  }

void uuid_to_string(uint8array *uuid, char *str, uint16_t maxlen)
  { const char hexchars[] = "0123456789ABCDEF";
    uint16_t len = 0;
    uint16_t i = uuid->len;
    char *sp = str;
    while(i--)
      { if(len > maxlen - 3) break;
        *sp++ = hexchars[uuid->data[i] >> 4];
        *sp++ = hexchars[uuid->data[i] & 0x0F];
        len += 2;
      }
    *sp = '\0';
  }

int16_t hex2dec(char c)
  { if((c >= '0') && (c <= '9')) return(c - '0');
    else if((c >= 'a') && (c <= 'f')) return(c + 10 - 'a');
    else if((c >= 'A') && (c <= 'F')) return(c + 10 - 'A');
    else return(-1);
  }

uint16_t string_to_uuid(char *str, uint8array *uuid)
  { int16_t v1, v2;
    uint16_t len = strlen(str);
    if((len < 4) || (len > 32) || (len & 0x01)) return(SL_STATUS_FAIL);
    char *sp = str;
    uint16_t nbytes = len >> 1;
    uuid->len = nbytes;
    while(len)
      { v1 = hex2dec(*sp++);
        v2 = hex2dec(*sp++);
        if((v1 < 0) || (v2 < 0)) return(SL_STATUS_FAIL);
        uuid->data[--nbytes] = v1 * 16 + v2;
        len -= 2;
      }
    return(SL_STATUS_OK);
  }

void buttonTimerHandler(sl_sleeptimer_timer_handle_t *handle, void* p)
  { (void) handle;
    (*(uint16_t *)(p))++;
  }

void sl_button_on_change(const sl_button_t *handle)
  { static sl_sleeptimer_timer_handle_t buttonTimerHandle;
    static uint16_t downCount = 0;
    if(handle == &sl_button_btn0)
      { if(handle->get_state(handle) == 1)
          { downCount = 1;
            sl_sleeptimer_start_periodic_timer_ms(&buttonTimerHandle, 100, buttonTimerHandler, &downCount, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
          }
        else
          { sl_sleeptimer_stop_timer(&buttonTimerHandle);
            if(downCount > 100) buttonReleasedAfter10Sec = true;
            else if(downCount > 50) buttonReleasedAfter5Sec = true;
            else if(downCount > 20) buttonReleasedAfter2Sec = true;
            else buttonReleasedNoHold = true;
            downCount = 0;
          }
      }
  }

void app_init(void)
  { setvbuf(stdout, obuffer, _IOLBF, sizeof(obuffer));                                  // Line buffering mode on stdout with buffer
    setvbuf(stdin, NULL, _IONBF, 0);                                                    // Unbuffered mode on stdin

    for(uint8_t pin = 1; pin < 5; pin++)                                                // Configure the port B GPIO pins used to gate the finger multiplexer FETS
      { GPIO_PinModeSet(gpioPortB, pin, gpioModePushPull, 0);
        GPIO_PinOutClear(gpioPortB, pin);
      }

    if(nvm3_repackNeeded(nvm3_defaultHandle))                                           // If needed, Do NVM re-packing
        nvm3_repack(nvm3_defaultHandle);

//    CMU_ClockEnable(cmuClock_PRS, true);                                                // Turn on the PRS, GPIO & RTCC clocks
//    CMU_ClockEnable(cmuClock_GPIO, true);
//    CMU_ClockEnable(cmuClock_RTCC, true);

    // RTCC Capture Channel 0 will capture RTCC counter value at beginning of local TX events
    // RTCC Capture Channel 2 will capture RCCC counter value at beginning of local RX events
    // NOTE:  DO NOT use Capture Channel 1 because the sl_sleeptimer functions use CC1 for interrupt scheduling!!!
    // Do not call RTCC_Init() here.  Let sl_sleeptimer handle this!
    // const RTCC_Init_TypeDef rtccInit = { .enable = true, .debugRun = true, .precntWrapOnCCV0 = false, .cntWrapOnCCV1 = false, .presc = rtccCntPresc_1, .prescMode = rtccCntTickPresc };
    // RTCC_Init(&rtccInit);

    int prsChannelTx = PRS_GetFreeChannel(prsTypeAsync);                                // Allocate Peripheral Reflex System channels for Radio Tx active signal
    PRS_SourceAsyncSignalSet(prsChannelTx, PRS_ASYNC_CH_CTRL_SOURCESEL_RACL, PRS_ASYNC_CH_CTRL_SIGSEL_RACLTX);
    const RTCC_CCChConf_TypeDef ccConfTx = { .chMode = rtccCapComChModeCapture, .compMatchOutAction = rtccCompMatchOutActionPulse, .prsSel = prsChannelTx, .inputEdgeSel = rtccInEdgeRising, .compBase = rtccCompBaseCnt };
    if(prsChannelTx >= 0) RTCC_ChannelInit(0, &ccConfTx);                               // Configure RealTime Clock Capture register 0 to capture Tx active time
    else app_log_error("PRS Channel allocation for RACLTX failed\n");

    int prsChannelRx = PRS_GetFreeChannel(prsTypeAsync);                                // Allocate Peripheral Reflex System channels for Radio Rx active signal
    PRS_SourceAsyncSignalSet(prsChannelRx, PRS_ASYNC_CH_CTRL_SOURCESEL_RACL, PRS_ASYNC_CH_CTRL_SIGSEL_RACLRX);
    const RTCC_CCChConf_TypeDef ccConfRx = { .chMode = rtccCapComChModeCapture, .compMatchOutAction = rtccCompMatchOutActionPulse, .prsSel = prsChannelRx, .inputEdgeSel = rtccInEdgeRising, .compBase = rtccCompBaseCnt };
    if(prsChannelRx >= 0) RTCC_ChannelInit(2, &ccConfRx);                               // Configure RealTime Clock Capture register 2 to capture Rx active time
    else app_log_error("PRS Channel allocation for RACLRX failed\n");

    vcrInit();                                                                         // Initialize the VCR application
  }


void app_process_action(void)
  { int c = 0;
    static uint16_t index = 0;
    static char buffer[80];

    while((c = getchar()) > 0)
      { if((c == '\r') || (c == '\n'))
          { buffer[index] = '\0';
            vcrProcessCommand(buffer);
            index = 0;
          }
        else if(index < sizeof(buffer) - 1) buffer[index++] = c;
      }

    if(buttonReleasedNoHold)                                                            // Handle short button press events -- toggle between VCR vibrations on/off
      { if(gloveConnected || localVCRStarted)
          { localVCRStarted = false;
            vcrStop();
            bluetooth_action('d');
          }
        else
          { localVCRStarted = true;
            bluetooth_action('s');
            vcrStart(&currentParams);
          }
        buttonReleasedNoHold = false;
      }
    if(buttonReleasedAfter2Sec)                                                         // Handle 2 second long button press event
      { vcrAutoCal();                                                                   // Perform autocal on each finger
        buttonReleasedAfter2Sec = false;
      }
    if(buttonReleasedAfter5Sec)                                                         // Handle 5 second long button press event
      { bluetooth_action('a');                                                          // Begin advertising to allow non-bonded (new) connections
        buttonReleasedAfter5Sec = false;
      }
    if(buttonReleasedAfter10Sec)                                                        // Handle 10 second long button press event
      { bluetooth_action('S');                                                          // Begin scanning to make new bonded connections with advertising gloves
        buttonReleasedAfter10Sec = false;
      }
  }

void updateGloveBond(uint8_t connection)                                                // Keeps track of 1 and only 1 bond for a paired glove
  { uint32_t objectType;
    size_t objectSize;
    uint8_t savedBonding = SL_BT_INVALID_BONDING_HANDLE;
    uint8_t newBonding;

    if((nvm3_getObjectInfo(nvm3_defaultHandle, NVM3_VCR_GLOVE_BONDING_KEY, &objectType, &objectSize) == ECODE_NVM3_OK) &&
       (objectType == NVM3_OBJECTTYPE_DATA) &&
       (objectSize == sizeof(uint8_t)))
        nvm3_readData(nvm3_defaultHandle, NVM3_VCR_GLOVE_BONDING_KEY, &savedBonding, sizeof(savedBonding));

    newBonding = connectionList[connection].bonding;
    if(newBonding != savedBonding)
      { nvm3_writeData(nvm3_defaultHandle, NVM3_VCR_GLOVE_BONDING_KEY, &newBonding, sizeof(newBonding));
        if(savedBonding != SL_BT_INVALID_BONDING_HANDLE) sl_bt_sm_delete_bonding(savedBonding);
        app_log_info("New glove bonding saved (%d), old bonding deleted (%d)\n", newBonding, savedBonding);
      }
  }

void stopAdvertiser(sl_sleeptimer_timer_handle_t *handle, void *data)                                               // Halt advertising for unbonded connections and return to advert for bonded only
  { (void) handle;
    (void) data;
    sl_bt_sm_configure(0x10, sl_bt_sm_io_capability_noinputnooutput);
    sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_advertiser_connectable_scannable);                 // Restart advertising
    // app_log_info("Advertising for bonded connections only\n");
  }

void stopScanner(sl_sleeptimer_timer_handle_t *handle, void *data)                                                  // Halt scanning for advertising glove pair
  { (void) handle;
    (void) data;
    sl_sleeptimer_stop_timer(&scannerStopTimer);
    sl_bt_scanner_stop();
    // app_log_info("Scanning stopped\n");
  }

uint16_t bluetooth_action(char action)                                                                              // Handle all of the keyboard & BT commands
  { int16_t vcrConn;
    switch(action)
      { case 'a':
            // app_log_info("Advertising for non-bonded connections\n");
            sl_sleeptimer_start_timer_ms(&advertiserStopTimer, ADV_DURATION, stopAdvertiser, NULL, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
            return(sl_bt_sm_configure(0x00, sl_bt_sm_io_capability_noinputnooutput));
            break;
        case 'd':                                                                       // Disconnect paired VCR device
            if((vcrConn = findGloveConnection()) != -1) return(sl_bt_connection_close(vcrConn));
            else return(SL_STATUS_FAIL);
            break;
        case 'e':                                                                       // Erase bond to currently connected VCR device
            if((vcrConn = findGloveConnection()) != -1) return(sl_bt_sm_delete_bonding(connectionList[vcrConn].bonding));
            else return(SL_STATUS_FAIL);
            break;
        case 'l':                                                                       // List current connections
            app_log_info("Connections:\n");
            for(uint16_t c = 0; c < MAXCONNECTIONS; c++)
              { if(connectionList[c].isOpen)
                    app_log_info("\tConnectionList[%d] %s %s bonding: 0x%.2x\n", c, connectionList[c].btAddressString,
                           connectionList[c].isGlove ? "Glove" : "Not-Glove", connectionList[c].bonding);
              }
            return(SL_STATUS_OK);
            break;
        case 'E':                                                                       // Erase all bonds
            return(sl_bt_sm_delete_bondings());
            break;
        case 's':                                                                       // Scan for known (bonded) connection
            // app_log_info("Scanning for bonded VCR devices\n");
            allowNonBonded = false;
            sl_bt_sm_configure(0x10, sl_bt_sm_io_capability_noinputnooutput);
            sl_bt_scanner_set_parameters(sl_bt_scanner_scan_mode_active, 16, 16);       // Setup Scanning with interval & window each set to 10mS (val / .625)
            sl_sleeptimer_start_timer_ms(&scannerStopTimer, SCAN_DURATION, stopScanner, NULL, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
            return(sl_bt_scanner_start(sl_bt_scanner_scan_phy_1m, sl_bt_scanner_discover_generic));
            break;
        case 'S':                                                                       // Scan for unknown (unbonded) connection
            // app_log_info("Scanning for unbonded VCR devices\n");
            allowNonBonded = true;
            // sl_bt_sm_delete_bondings();
            sl_bt_sm_configure(0x00, sl_bt_sm_io_capability_noinputnooutput);
            sl_bt_scanner_set_parameters(sl_bt_scanner_scan_mode_active, 16, 16);       // Setup Scanning with interval & window each set to 10mS (val / .625)
            sl_sleeptimer_start_timer_ms(&scannerStopTimer, SCAN_DURATION, stopScanner, NULL, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
            return(sl_bt_scanner_start(sl_bt_scanner_scan_phy_1m, sl_bt_scanner_discover_generic));
            break;
        default:
            break;
      }
    return(SL_STATUS_INVALID_PARAMETER);
  }

uint16_t remoteVibrateSet(vibrateSet_t *set)
  { static uint16_t sequenceNumber = 0;
    int16_t gloveConn;
    vibrateCharacteristic_t vc;
    if(gloveConnected && ((gloveConn = findGloveConnection()) != -1))
      { vc.lastTxTime = RTCC_ChannelCaptureValueGet(0);
        vc.scheduledTxTime = RTCC_CounterGet();
        vc.sequenceNumber = sequenceNumber++;
        vc.set = *set;
//        static uint32_t lastlastTxTime = 0;
//        app_log_info("Last Tx = %lu (+%2luCI), Sched Tx = %lu, Sched-Last delta = %lu\n", vc.lastTxTime, ((vc.lastTxTime - lastlastTxTime) / connIntervalInTicks), vc.scheduledTxTime, (vc.scheduledTxTime - vc.lastTxTime));
//        lastlastTxTime = vc.lastTxTime;
        return(sl_bt_gatt_write_characteristic_value(gloveConn, vcrVibrationSetCharacteristic, sizeof(vc), (uint8_t*)&vc));
      }
    return(SL_STATUS_FAIL);
  }

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)                                                       // Handler for bluetooth stack events
  { sl_status_t sc;
    (void) sc;
    switch (SL_BT_MSG_ID(evt->header))                                                      // Handle stack events
      {
        case sl_bt_evt_system_boot_id:         //===========================================// This event indicates the device has started and the radio is ready
            { // Do not call any stack command before receiving this boot event!

              // bd_addr localAddress;
              // uint8_t localAddressType;
              // sl_bt_system_get_identity_address(&localAddress, &localAddressType);
              sc = sl_bt_connection_set_default_parameters (80 /*100mS min interval*/, 160 /*200mS max_interval*/, 0 /*latency*/, 600 /*6S timeout*/, 0, 65535);
              sc = sl_bt_sm_set_bondable_mode(true);
              sc = sl_bt_sm_configure(0x10, sl_bt_sm_io_capability_noinputnooutput);        // Allow connecitons ONLY FROM BONDED devices
              sc = sl_bt_advertiser_create_set(&advertising_set_handle);                    // Create an advertising set
              sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle, sl_bt_advertiser_general_discoverable);           // Generate data for advertising

              sc = sl_bt_advertiser_set_timing(advertising_set_handle, 400 /*250mS min interval*/, 528 /*330mS max interval*/, 0 /*duration*/, 0 /*max adv events*/);

              sc = sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_advertiser_connectable_scannable);                   // Start advertising and enable connections
            }
          break;

        case sl_bt_evt_connection_opened_id:  //============================================// This event indicates that a new connection was opened
              { sl_bt_evt_connection_opened_t *evtp = &evt->data.evt_connection_opened;

                connectionList[evtp->connection].isOpen = 1;
                connectionList[evtp->connection].isGlove = 0;
                connectionList[evtp->connection].btAddress = evtp->address;
                connectionList[evtp->connection].btAddressType = evtp->address_type;
                connectionList[evtp->connection].bonding = evtp->bonding;
                bd_addr_to_string(evtp->address, connectionList[evtp->connection].btAddressString, sizeof(connectionList[evtp->connection].btAddressString));
                // app_log_info("Connection %d opened to %s address %s\n", evtp->connection, addressTypeStrings[evtp->address_type == 1], connectionList[evtp->connection].btAddressString);
                sc = sl_bt_sm_increase_security(evtp->connection);
                sc = sl_bt_gatt_discover_primary_services(evtp->connection);

    #ifdef SL_CATALOG_BLUETOOTH_FEATURE_POWER_CONTROL_PRESENT                // Set remote connection power reporting - needed for Power Control
                sc = sl_bt_connection_set_remote_power_reporting(evt->data.evt_connection_opened.connection,
                  sl_bt_connection_power_reporting_enable);
                app_assert_status(sc);
    #endif // SL_CATALOG_BLUETOOTH_FEATURE_POWER_CONTROL_PRESENT

                app_assert_status(sc);
                // Begin to advertise again in order to allow multiple connections
                // sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_advertiser_connectable_scannable);
                // Stop advertising as we must avoid extraneous Tx and Rx signals from advertising or a second connection.  As these would contaminate the time synchronization process
                sl_bt_advertiser_stop(advertising_set_handle);
              }
            break;

        case sl_bt_evt_connection_closed_id:  //============================================// This event indicates that a connection was closed
              { sl_bt_evt_connection_closed_t *evtp = &evt->data.evt_connection_closed;
                if(connectionList[evtp->connection].isGlove)
                  { vcrStop();
                    gloveConnected = localVCRStarted = false;
                  }
                // app_log_info("Connection %d %s closed\n", evtp->connection, connectionList[evtp->connection].isGlove ? "Glove" : "Not-Glove");
                connectionList[evtp->connection].isOpen = connectionList[evtp->connection].isGlove = 0;
                // sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle, sl_bt_advertiser_general_discoverable);        // Generate data for advertising
                // app_assert_status(sc);

                sc = sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_advertiser_connectable_scannable);                 // Restart advertising after client has disconnected
                app_assert_status(sc);
              }
            break;

        case sl_bt_evt_connection_parameters_id:  //========================================// This event indicates that connection parameters changed or a new bonding level
              { sl_bt_evt_connection_parameters_t *evtp = &evt->data.evt_connection_parameters;
                uint32_t interval = evtp->interval + (evtp->interval >> 2);
                connIntervalInTicks = interval * TICKSPERMS;
//                app_log_info("Connection Parameters:  interval = %lu mS (%lu ticks), latency = %d, timeout = %u mS, %s\n",
//                             interval, connIntervalInTicks, evtp->latency, evtp->timeout * 10, securityModeNames[evtp->security_mode]);
              }
            break;

        case sl_bt_evt_sm_bonded_id:  //====================================================// This event indicates that a bonding attempt succeeded
              { sl_bt_evt_sm_bonded_t *evtp = &evt->data.evt_sm_bonded;
                char *pbString;
                connectionList[evtp->connection].bonding = evtp->bonding;
                if(evtp->bonding != SL_BT_INVALID_BONDING_HANDLE)
                  { pbString = "Bonded";
                    if(connectionList[evtp->connection].isGlove) updateGloveBond(evtp->connection);
                  }
                else pbString = "Paired";
                app_log_info("%s %s\n", pbString, securityModeNames[evtp->security_mode]);
              }
            break;

        case sl_bt_evt_sm_bonding_failed_id:  //============================================// This event indicates that a bonding attempt failed
              { sl_bt_evt_sm_bonding_failed_t *evtp = &evt->data.evt_sm_bonding_failed;
                connectionList[evtp->connection].bonding = SL_BT_INVALID_BONDING_HANDLE;
                app_log_error("Bonding failed: 0x%.4x\n", evtp->reason);
              }
            break;

        case sl_bt_evt_gatt_service_id:  //=================================================// This event indicates that a service has been discovered
              { sl_bt_evt_gatt_service_t *evtp = &evt->data.evt_gatt_service;
                if(uuid_cmp(&evtp->uuid, &vcrServiceUUID) == SL_STATUS_OK)
                  { connectionList[evtp->connection].isGlove = 1;
                    gloveConnected = true;
                    updateGloveBond(evtp->connection);
                    sc = sl_bt_gatt_discover_characteristics(evtp->connection, evtp->service);
                  }
//                char uuidString[40];
//                uuid_to_string(&evtp->uuid, uuidString, sizeof(uuidString));
//                app_log_info("Has service %s\n", uuidString);
              }
            break;

        case sl_bt_evt_gatt_characteristic_id:  //==========================================// This event indicates that a characteristic has been discovered
              { sl_bt_evt_gatt_characteristic_t *evtp = &evt->data.evt_gatt_characteristic;

                // char uuidstr[36];  uuid_to_string(&evtp->uuid, uuidstr, sizeof(uuidstr));
                // app_log_info("Characteristic %d discovered with UUID %s\n", evtp->characteristic, uuidstr);
                if(uuid_cmp(&evtp->uuid, &vcrVibrationSetCharacteristicUUID) == SL_STATUS_OK)
                  { vcrVibrationSetCharacteristic = evtp->characteristic;
                    // app_log_info("vcr vibrationset characteristic found %d\n", evtp->characteristic);
                  }
              }
            break;

        case sl_bt_evt_gatt_server_attribute_value_id:  //==================================// This event indicates that the value of an attribute in the local GATT db was changed by a remote GATT client
              { sl_bt_evt_gatt_server_attribute_value_t *evtp = &evt->data.evt_gatt_server_attribute_value;

                if(evtp->attribute == gattdb_command)                                   // The command characteristic value changed
                  { uint8_t command[128];
                    size_t len;
                    if(sl_bt_gatt_server_read_attribute_value(gattdb_command, 0, sizeof(command), &len, command) == SL_STATUS_OK)
                      { command[len] = '\0';
                        int16_t cmdResult = vcrProcessCommand((char*)command);
                        sc = sl_bt_gatt_server_write_attribute_value(gattdb_command_result, 0, sizeof(cmdResult), (uint8_t*)&cmdResult);
                        sc = sl_bt_gatt_server_notify_all(gattdb_command_result, sizeof(cmdResult), (uint8_t*)&cmdResult);
                        // notifyCommandResult = true;
                      }
                  }
                else if(evtp->attribute == gattdb_vibrationset)                         // The vibrationset characteristic value changed
                  { vibrateCharacteristic_t vc;
                    size_t len;
                    if(sl_bt_gatt_server_read_attribute_value(gattdb_vibrationset, 0, sizeof(vc), &len, (uint8_t*)&vc) == SL_STATUS_OK)
                      { /*    See the PowerPoint slides documenting the process for computing a master-slave timebase translation offset
                         *    then using this to measure the latency from sending/queueing on the master, to receiving/processing on the slave of the vibrate characteristic packet events & times
                         *    and finally adjusting the event delivery times on the slave to subtract out the communications latency.
                         */
                        // Declare static time values that we need from one packet reception to the next
                        static uint32_t TxRxTimebaseTranslationOffset, priorRxActivationTimeN_Rx = 0, priorTxActivationTimeN_1_Tx = 0;
                        static uint16_t priorTxSeqNo;

                        // Set the key values used to calculate the timebase translation offset and latency
                        uint16_t TxSeqNo                = vc.sequenceNumber;
                        uint32_t currentTime_Rx         = RTCC_CounterGet();
                        uint32_t RxActivationTimeN_Rx   = RTCC_ChannelCaptureValueGet(2);
                        uint32_t TxActivationTimeN_1_Tx = vc.lastTxTime;
                        uint32_t TxActivationTimeN_Tx   = TxActivationTimeN_1_Tx + connIntervalInTicks;
                        uint32_t TxScheduledTime_Tx     = vc.scheduledTxTime;

                        // Calculate the timebase translation offset value
                        uint32_t newTxRxTimebaseTranslationOffset = TxActivationTimeN_Tx - RxActivationTimeN_Rx;
                        if((RxActivationTimeN_Rx - priorRxActivationTimeN_Rx) <= (TxActivationTimeN_1_Tx - priorTxActivationTimeN_1_Tx) || (TxSeqNo != priorTxSeqNo + 1))
                            TxRxTimebaseTranslationOffset = newTxRxTimebaseTranslationOffset;
                        //else the Rx Conn Interval slip means we can't use this new timebase translation offset to calculate the master to slave timeline delta & latency.  So retain the prior value!

                        // Calculate the latency from when the master scheduled the set of FUTURE remote glove vibrations until the slave receives and is ready to queue up the FUTURE glove vibrations locally
                        int32_t  latency = // currentTime_Rx - TxScheduledTime_Tx + FIXEDLATENCYADDER;
                                              currentTime_Rx - (TxScheduledTime_Tx - TxRxTimebaseTranslationOffset) + FIXEDLATENCYADDER;

//                        app_log_info(/*"TxSeq = %u t(TXAN-1) = %lu (+%2luCI), t(RXAN) = %lu (+%2luCI), scheduledTx = %lu, currentRx = %lu, */"TxRxTTO = %lu, latency = %4ld (%6.2fmS)\n",
//                               // TxSeqNo,
//                               // TxActivationTimeN_1_Tx, (TxActivationTimeN_1_Tx - priorTxActivationTimeN_1_Tx) / connIntervalInTicks,
//                               // RxActivationTimeN_Rx,   (RxActivationTimeN_Rx   - priorRxActivationTimeN_Rx)   / connIntervalInTicks,
//                               // TxScheduledTime_Tx, currentTime_Rx,
//                               TxRxTimebaseTranslationOffset, latency,  (float)latency / 32.768);

                        // Save time values for calculating packet intervals
                        priorRxActivationTimeN_Rx   = RxActivationTimeN_Rx;
                        priorTxActivationTimeN_1_Tx = TxActivationTimeN_1_Tx;
                        priorTxSeqNo                = vc.sequenceNumber;

                        // Sanity check!
                        if(latency < VIBRATIONDELAY)
                          { for(uint16_t finger = 0; finger < 4; finger++) vc.set.start_time[finger] -= latency;
                          }
                        else app_log_error("Latency out-of-range -- truncated to zero!\n");
                        vcrVibrateSet(&vc.set);
                      }
                  }
              }
            break;

        case sl_bt_evt_gatt_server_characteristic_status_id:  //================================// This event occurs when the remote device enabled or disabled notification
              { sl_bt_evt_gatt_server_characteristic_status_t *evtp = &evt->data.evt_gatt_server_characteristic_status;

              if(evtp->characteristic == gattdb_command_result)
                  { // Command_Result Client Characteristic Configuration descriptor was changed in the gattdb_command_result characteristic.
                    if(evtp->client_config_flags & sl_bt_gatt_notification)
                      { // The client just enabled the notification.  No need to send notification of prior command results though...
                        // app_log_info("Command_Result notification enabled\n");
                      }
                    // else app_log_info("Command_Result notification disabled\n");
                  }
              }
            break;

        case sl_bt_evt_scanner_legacy_advertisement_report_id:  //==============================// This event occurs when server has received an advertisement while scanning
              { sl_bt_evt_scanner_legacy_advertisement_report_t *evtp = &evt->data.evt_scanner_legacy_advertisement_report;

                if(evtp->event_flags & SL_BT_SCANNER_EVENT_FLAG_CONNECTABLE)
                  {
                    uint8_t hasVCRService = false;
                    // char str[24];  bd_addr_to_string(evtp->address, str, sizeof(str));
                    // aoo_log_info("Scan connectable response from %s\n", str);
                    uint8_t len = evtp->data.len;
                    uint8_t *dp = evtp->data.data;
                    dp = evtp->data.data;
                    for(uint8_t i = 0; i < len; )
                      { uint8_t elementlen = *dp++;
                        uint8_t adtype = *dp++;
                        switch(adtype)
                          {
//                           case 0x01:        // Flags
//                                app_log_info("\tFlags = 0x%.2x\n", *dp);
//                                break;
//                           case 0x03:        // List of 16 bit services
//                                aoo_log_info("\t16-bit Services:");
//                                for(uint8_t s = 0; s < elementlen; s += 2)
//                                    app_log_info(" 0x%.4hx", *(uint16_t*)(dp + s));
//                                app_log_info("\n");
//                                break;
                            case 0x07:        // List of 128 bit services
                                // app_log_info("\t128-bit Services:\n");
                                for(uint8_t s = 0; s < elementlen; s += 16)
                                  { if(uuid128cmp(&dp[s], vcrServiceUUID.data) == 0) hasVCRService = 1;
//                                    app_log_info("\t\t");
//                                    for(uint8_t b = 0; b < 16; b++) app_log_info(" %.2x", dp[s+b]);
//                                    app_log_info("\n");
                                  }
                                break;
//                                case 0x08:        // Shortened local name
//                                app_log_info("\tShortened local name: %.*s\n", elementlen - 1, dp);
//                                break;
//                                case 0x09:        // Complete local name
//                                app_log_info("\tComplete local name: %.*s\n", elementlen - 1, dp);
//                                break;
                            default:
                                // app_log_info("\tAD Type = 0x%.2x (%d)\n", adtype, elementlen);
                                break;
                          }
                        if(hasVCRService && (allowNonBonded || (evtp->bonding != SL_BT_INVALID_BONDING_HANDLE)))
                          { uint8_t connection;
                            stopScanner(NULL, NULL);
//                            app_log_info("Scanner found %s VCR device\n",
//                                         evtp->bonding != SL_BT_INVALID_BONDING_HANDLE ? "known/bonded" : "new/unbonded");
                            sc = sl_bt_connection_open(evtp->address, evtp->address_type, sl_bt_gap_phy_1m, &connection);
                          }
                        dp += elementlen - 1;
                        i += elementlen + 1;
                      }
                  }
              }
            break;

        ///////////////////////////////////////////////////////////////////////////
        // Add additional event handlers here as your application requires!      //
        ///////////////////////////////////////////////////////////////////////////

        default:                                              //==============================// Default event handler
            break;
      }
  }

/**************************************************************************//**
 * Callback function of connection close event.
 *
 * @param[in] reason Unused parameter required by the health_thermometer component
 * @param[in] connection Unused parameter required by the health_thermometer component
 *****************************************************************************/
void sl_bt_connection_closed_cb(uint16_t reason, uint8_t connection)
  { (void)reason;
    (void)connection;

  }

/***************************************************************************//**
 * Sends notification of a characteristic.
 *
 ******************************************************************************/
//static sl_status_t send_notification(uint16_t attribute)
//  { sl_status_t sc;
//    uint8_t data[100];
//    size_t len;
//
//    // Read attribute value from GATT db, then send notifications
//    if((sc = sl_bt_gatt_server_read_attribute_value(attribute, 0, sizeof(data), &len, data)) == SL_STATUS_OK)
//      { if((sc = sl_bt_gatt_server_notify_all(attribute, len, data)) == SL_STATUS_OK)
//           app_log_append("Notification sent\n");
//      }
//    return(sc);
//  }

