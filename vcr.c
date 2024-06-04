
#include "vcr.h"
#include "lra.h"
#include "app.h"
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <sl_sleeptimer.h>
#include <sl_power_manager.h>
#include <sl_led.h>
#include <sl_simple_led.h>
#include <sl_simple_led_instances.h>
#include <nvm3_default.h>
#include <sl_bluetooth.h>
#include <sl_bt_api.h>
#include "gatt_db.h"

#define  FW_REVISION      "4.3.6"
#define  DATETIME         "04Jun2024 07:43"

#define  NTIMERHANDLESETS  10
#define  NVM3_VCR_TIMING_PARAMETERS_KEY       0x3719                            //Unique key for NVM object storage

static const char *mirrorNames[] = { "Non-Mirrored", "Mirrored", "Different" };
static uint8_t displayEnabled = 0;        // Boolean flag indicating whether finger map display is on or off
static uint8_t vcrRunning = 0;            // Boolean flag indicating whether vibration is running or not
uint8_t I2CBusMux = 1;              // 0 = use LRA FET multiplexing (single DRV2605 chip), 1 = use I2C bus multiplexing (multiple DRV2605 chips)
static char fingerMap[NFINGERS] = { '-', '-', '-', '-' };
static sl_sleeptimer_timer_handle_t vcrTimerHandle;
static sl_sleeptimer_timer_handle_t fTimerHandle0[NFINGERS * NTIMERHANDLESETS], fTimerHandle1[NFINGERS * NTIMERHANDLESETS];
static sl_sleeptimer_timer_handle_t cycleTimerHandle[NTIMERHANDLESETS];


vcrParameters_t currentParams;

static const uint8_t FingerSequences[24][NFINGERS] = {   // Array of all possible randomly ordered sequences of 4 fingers.
    { 0, 1, 2, 3 },
    { 1, 0, 2, 3 },
    { 2, 0, 1, 3 },
    { 3, 0, 1, 2 },
    { 0, 2, 1, 3 },
    { 1, 2, 0, 3 },
    { 2, 1, 0, 3 },
    { 3, 1, 0, 2 },
    { 0, 3, 1, 2 },
    { 1, 3, 0, 2 },
    { 2, 3, 0, 1 },
    { 3, 2, 0, 1 },
    { 0, 1, 3, 2 },
    { 1, 0, 3, 2 },
    { 2, 0, 3, 1 },
    { 3, 0, 2, 1 },
    { 0, 2, 3, 1 },
    { 1, 2, 3, 0 },
    { 2, 1, 3, 0 },
    { 3, 1, 2, 0 },
    { 0, 3, 2, 1 },
    { 1, 3, 2, 0 },
    { 2, 3, 1, 0 },
    { 3, 2, 1, 0 },
  };

void printSplash()
  { printf("VibroTactile Coordinated Reset Driver  Version: %s %s  (c) 2022-2024 Douglas E. Bartlett\n", FW_REVISION, DATETIME);
  }

uint8_t validVibrationSequence(const vcrParameters_t *p)
  { // Tests vcr sequence referenced by *p for self consistent value.  Returns 1 if sequence is valid.
    bool enoughTimerHandleSets = (p->fingerSpacing * NFINGERS * NTIMERHANDLESETS > 500);
    bool validFrequency = (p->frequency == 0) || ((p->frequency >= LRAFMIN) && (p->frequency <= LRAFMAX));
    return((p->fingerSpacing > p->fingerDuration) && (p->jitter < p->fingerSpacing - p->fingerDuration) &&
           (p->onDuration >= NFINGERS * p->fingerSpacing) && (p->intensity > 8) && (p->intensity <= 127) &&
           validFrequency && enoughTimerHandleSets);
  }

void printVibrationSequence(char *header, const vcrParameters_t *p)
  { // Prints a header string and the vcr parameters values passed as inputs.
    char sd = p->differentJitter ? 'D' : 'S';
    printf("%s\nfinger_duration: %5.2f mS, finger_spacing: %5.2f mS, on_time: %6.1f mS, off_time: %6.1f mS, jitter: %5.2f mS (%c), intensity: %u, L-R %s, frequency: %u\n",
           header, p->fingerDuration, p->fingerSpacing, p->onDuration, p->offDuration, p->jitter, sd, p->intensity, mirrorNames[p->mirroring], p->frequency);
  }

int16_t storeVibrationSequence(const vcrParameters_t *p)
  { // Stores the vcr sequence pointed to by s in non-volatile memory for automatic default vibration sequence startup
    return(nvm3_writeData(nvm3_defaultHandle, NVM3_VCR_TIMING_PARAMETERS_KEY, p, sizeof(*p)));
  }

void printFingerMap(uint8_t finger, char c)
  { fingerMap[finger] = c;                                                              // Mark the fingerMap for the specified fingers
    if(displayEnabled)
      { printf("\t%c  %c  %c  %c\033[1A\n",fingerMap[0], fingerMap[1], fingerMap[2], fingerMap[3]);
        fflush(stdout);
      }
  }

void vcrFingerOn(sl_sleeptimer_timer_handle_t *handle, void *data)
  { (void) *handle;
    uint8_t finger = (uint32_t)data & 0xFF;
    uint8_t intensity = ((uint32_t)data) >> 8;
    lraStart(finger, intensity);                                                        // Turn on the LRA driver
    if(finger == 0)  sl_led_turn_on(&sl_led_led0);                                      // Blink board LED with pointer finger vibration
    printFingerMap(finger, 'X');                                                        // If display is enabled, print the fingerMap
  }

void vcrFingerOff(sl_sleeptimer_timer_handle_t *handle, void *data)
  { (void) *handle;
    uint8_t finger = (uint32_t)data & 0xFF;
    lraStop(finger);                                                                    // Turn off the LRA driver
    if(finger == 0)  sl_led_turn_off(&sl_led_led0);                                     // Blink blue LED with pointer finger vibration
    printFingerMap(finger, '-');                                                        // If display is enabled, print the fingerMap
  }

void vcrVibrateSet(vibrateSet_t *vset)                                                  // Schedule a timers for a set of 4 finger vibrations
  { static int handleSet = 0;
    for(uint8_t finger = 0; finger < NFINGERS; finger++)
      { int handleIndex = handleSet * NFINGERS + finger;
        // We need to rotate through sets of timer handles because the 500mS constant delay causes the finger vibration timers to overlap in time!
        void *fData = (void*)((int32_t)finger | ((int32_t)(vset->intensity) << 8));
        uint32_t msOn =  vset->start_time[finger];
        uint32_t msOff = vset->start_time[finger] + vset->duration[finger];
        sl_sleeptimer_start_timer(&fTimerHandle1[handleIndex], msOn,  vcrFingerOn,  fData, 1, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
        sl_sleeptimer_start_timer(&fTimerHandle0[handleIndex], msOff, vcrFingerOff, fData, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
      }
    if(++handleSet >= NTIMERHANDLESETS) handleSet = 0;
  }

void vcr4FingerCycle(sl_sleeptimer_timer_handle_t *handle, void *data)                  // Queue up the individual finger vibration on/off events
  { (void) *handle;
    vcrParameters_t *p = (vcrParameters_t *)data;
    vibrateSet_t localvset, remotevset;
    const uint8_t *localFingerList = FingerSequences[rand() % 24];                      // localFingerList sets the local finger order
    const uint8_t *remoteFingerList = FingerSequences[rand() % 24];                     // remoteFingerList sets the remote finger order ONLY when mirroring == Different
    localvset.intensity = remotevset.intensity = p->intensity;
    uint32_t time = VIBRATIONDELAY;                                                     // VIBRATONDELAY (500mS * TICKSPERMS) offset for all vibration on/off events.  Allows remote glove to compensate for communications latency!
    uint32_t timeIncrement = p->fingerSpacing * TICKSPERMS;                             // Increment time in units of ticks of the 32768 Hz clock
    uint32_t commonDuration = p->fingerDuration * TICKSPERMS;
    for(uint8_t i = 0; i < NFINGERS; i++)
      { uint16_t localFinger, remoteFinger;
        uint32_t localJitter, remoteJitter;
        localFinger = localFingerList[i];
        switch(p->mirroring)
          { default:
            case Non_Mirrored:
                remoteFinger = localFinger;
                break;
            case Mirrored:
                remoteFinger = 3 - localFinger;
                break;
            case Different:
                remoteFinger = remoteFingerList[i];
                break;
          }
        localJitter =       p->jitter * TICKSPERMS * (rand() % 100000) / 100000;
        if(!p->differentJitter) remoteJitter = localJitter;
        else remoteJitter = p->jitter * TICKSPERMS * (rand() % 100000) / 100000;
        localvset.start_time[localFinger] =   time + localJitter;
        remotevset.start_time[remoteFinger] = time + remoteJitter;
        localvset.duration[localFinger] =     commonDuration;
        remotevset.duration[remoteFinger] =   commonDuration;
        time += timeIncrement;
      }
    vcrVibrateSet(&localvset);                                                          // Schedule the LOCAL vibrations
    remoteVibrateSet(&remotevset);                                                      // Schedule the REMOTE vibrations
  }

void vcrRootCycle(sl_sleeptimer_timer_handle_t *handle, void *data)                     // Schedule the root vcr cycles
  { (void) *handle;
    vcrParameters_t *p = (vcrParameters_t *)data;
    uint32_t time = 0;
    uint32_t timeIncrement = NFINGERS * p->fingerSpacing * TICKSPERMS;                  // Time is incremented in units of ticks of the 32768 Hz clock
    uint32_t timeQuit = p->onDuration * TICKSPERMS - timeIncrement;
    for(uint16_t i = 0; (i < NTIMERHANDLESETS) && (time < timeQuit); i++)
      { sl_sleeptimer_start_timer(&cycleTimerHandle[i], time, vcr4FingerCycle, p, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
        time += timeIncrement;
      }
  }

int16_t vcrStart(vcrParameters_t *p)
  { uint8_t ret = 0;
    if(!vcrRunning && validVibrationSequence(p))
      { // BUG BUG BUG   need to send the vibration parameter set to the remote glove to ensure consistent mirroring, intensity & frequency  BUG BUG BUG BUG
        for(uint8_t f = 0; f < NFINGERS; f++) lraSetFrequency(f, p->frequency);
        vcrRootCycle(NULL, p);
        uint32_t period = (p->onDuration + p->offDuration) * TICKSPERMS;
        sl_sleeptimer_start_periodic_timer(&vcrTimerHandle, period, vcrRootCycle, p, 0, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
        printVibrationSequence("vCR started", p);
        ret = vcrRunning = 1;
      }
    return(ret);
  }

void vcrStop(void)
  { // Stops any currently running vcr sequence and halts all vibration which might be running.
    sl_sleeptimer_stop_timer(&vcrTimerHandle);                                          // Stop all running VCR timers
    for(uint8_t i = 0; i < NTIMERHANDLESETS; i++)
        sl_sleeptimer_stop_timer(&cycleTimerHandle[i]);
    for(uint8_t i = 0; i < NFINGERS * NTIMERHANDLESETS; i++)
      { sl_sleeptimer_stop_timer(&fTimerHandle0[i]);
        sl_sleeptimer_stop_timer(&fTimerHandle1[i]);
      }
    sl_led_turn_off(&sl_led_led0);                                                      // Turn off the blue LED
    for(uint8_t f = 0; f < NFINGERS; f++)                                               // For each finger
      { lraStop(f);                                                                     // Stop the vibration
        fingerMap[f] = '-';                                                             // Reset the fingerMap
      }
    vcrRunning = 0;
  }

void statusPrint(uint8_t finger)
  { uint8_t gainTable[4] = { 5, 10, 20, 30 };
    uint8_t st = lraGetStatus(finger);
    drv2605_autocal_t acal;
    lraGetAutoCalParams(finger, &acal);
    float comp = 1.0 + acal.a_cal_comp / 255.0;
    float bemf = (acal.a_cal_bemf / 255.0) * 1.22 / gainTable[acal.bemf_gain & 0x03];
    printf("F%u\t%3u hZ\t%4.2f V  a_cal_comp = %6.4f (0x%.2x)  a_cal_bemf = %7.5f (0x%.2x)  bemf_gain = %u (0x%.2x)%s%s\n",
        finger, lraGetResonantFrequency(finger), lraGetBatteryVoltage(finger) / 1000.0,
        comp, acal.a_cal_comp, bemf, acal.a_cal_bemf, gainTable[acal.bemf_gain], acal.bemf_gain,
        ((st & 0x02) ? " Over Temp" : ""), ((st & 0x01) ? " Over Current" : ""));
  }

void initFailPrint(uint8_t finger, lraInitResults_t *irp)
  { const char *diagString = (irp->diag_failed ? "  Diagnostics Failed" : "");
    const char *acalString = (irp->acal_failed ? "  Autocal Failed" : "");
    const char *ioString = (irp->io_failed ? "  IO failed  " : "");
    const char *shortString = (irp->shorted ? "  Short Circuit" : "");
    const char *tempString = (irp->over_temp ? "  Over Temp" : "");
    printf("F%u\t%s%s%s%s%s\n", finger,
             ioString, diagString, acalString, shortString, tempString);
  }

int16_t vcrAutoCal(void)
  { int16_t retval = SL_STATUS_OK;
    vcrStop();
    for(uint8_t finger = 0; finger < NFINGERS; finger++)                                               // For each finger
      { int16_t stat;
        lraInitResults_t ir;
        if((stat = lraReset(finger)) != 0)
           { printf("lraReset(%d) failed %d!\n", finger, stat);
             retval = SL_STATUS_FAIL;
           }
        stat = lraInit(finger, &ir);
        if(stat == 0)  statusPrint(finger);
        else
          { initFailPrint(finger, &ir);
            statusPrint(finger);
            retval = SL_STATUS_FAIL;
          }
      }
    vcrStop();
    return(retval);
  }

int16_t vcrProcessCommand(char *cmd)
  { int16_t retval = SL_STATUS_OK;
    switch(cmd[0])
      { case '\0':
            break;
        case '?':
            printSplash();
            puts("Operational Commands:");
            puts("\td\t\tStore the current vCR parameters in EEPROM as new defaults");
            puts("\tf [<frequency>]\tSet/display VCR vibration frequency");
            puts("\ti [<intensity>]\tSet/display VCR vibration intensity (0-127)");
            puts("\tj [<s | d>]\tSet/display VCR jitter (Same, Different)");
            puts("\tm [<n | m | d>]\tSet/display VCR mirroring (Non-Mirrored, Mirrored, Different)");
            puts("\ts\t\tStop VCR stimulus");
            puts("\tv\t\tStart VCR stimulus");
            puts("\tV <duration> <spacing> [on_time] [off_time] [jitter]\tSet VCR parameters timing");
            puts("\tV\t\tDisplay current VCR parameters timing, jitter, intensity & mirroring");

            puts("\nDebug & diagnostic Commands:");
            puts("\tA\t\tReset LRA driver & perform initialization & autocal on each finger");
            puts("\tBa\t\tBluetooth - Begin advertising to allow non-bonded connections");
            puts("\tBd\t\tBluetooth - Disconnect from VCR pair");
            puts("\tBe\t\tBluetooth - Erase bond to currently connected VCR pair");
            puts("\tBl\t\tBluetooth - List Connections");
            puts("\tBs\t\tBluetooth - Scan & connect to known/bonded VCR pair");
            puts("\tBS\t\tBluetooth - Scan & connect to new/unbonded VCR pair");
            puts("\tBE\t\tBluetooth - Erase ALL bonds");
            puts("\tD <finger>\tDump all LRA driver registers for specified finger");
            puts("\tE1\t\tSet EM1 minimum energy mode");                    // Keeps UART clocks running to allow VCOM stdin from debugger/USB
            puts("\tE3\t\tSet EM3 minimum energy mode");
            puts("\tF <finger>\tStart constant vibration on specified finger");
            puts("\tI <finger>\tInitialize & autocal LRA driver on specified finger");
            puts("\tM\t\tDisplay finger vibration map");
            puts("\tQ\t\tDisplay LRA resonant frequency, battery voltage & autocal parameters");
            puts("\tR\t\tReset & initialize LRA drivers");
            puts("\tX <mode>\tSet the multiplexing mode (0 = LRA Fet Mux, 1 = I2C Bus Mux");
            puts("\t@ <finger> <register> <value>\tSet DRV2605 register value");
            break;
        case 'd':
            if(validVibrationSequence(&currentParams))
              { if((retval = storeVibrationSequence(&currentParams)) == SL_STATUS_OK)
                    printVibrationSequence("Saved new default vCR parameters", &currentParams);
              }
            else retval = SL_STATUS_INVALID_CONFIGURATION;
            break;
        case 'f':
          { int16_t frequency;
            int8_t nf = sscanf(cmd+1, "%hu", &frequency);
            if(nf <= 0)
              { if(currentParams.frequency == 0) puts("Frequency = auto resonance");
                else printf("Frequency = %u\n", currentParams.frequency);
              }
            else if((nf == 1) && ((frequency == 0) || ((frequency >= LRAFMIN) && (frequency <= LRAFMAX))))
              { uint16_t period = 10153.315 / frequency + 0.5;        // Compute rounded LRA open loop period register value
                currentParams.frequency = 10153.315 / period + 0.5;   // Convert rounded period back to frequency
              }
            else retval = SL_STATUS_INVALID_PARAMETER;
          }
            break;
        case 'i':
              { int16_t intensity;
                int8_t nf = sscanf(cmd+1, "%hu", &intensity);
                if(nf <= 0) printf("Intensity = %u\n", currentParams.intensity);
                else if((nf == 1) && (intensity >= 0) && (intensity <= 127))  currentParams.intensity = intensity;
                else retval = SL_STATUS_INVALID_PARAMETER;
              }
            break;
        case 'j':
              { char cJitter;
                int8_t nf = sscanf(cmd+1, " %c", &cJitter);
                if(nf <= 0) printf("Jitter = %5.2f (%s)\n", currentParams.jitter, currentParams.differentJitter ? "different" : "same");
                else if(cJitter == 'd') currentParams.differentJitter = 1;
                else if(cJitter == 's') currentParams.differentJitter = 0;
                else retval = SL_STATUS_INVALID_PARAMETER;
              }
            break;
       case 'm':
              { char cMirror;
                int8_t nf = sscanf(cmd+1, " %c", &cMirror);
                if(nf <= 0) printf("Mirroring = %s\n", mirrorNames[currentParams.mirroring]);
                else if(cMirror == 'n') currentParams.mirroring = Non_Mirrored;
                else if(cMirror == 'm') currentParams.mirroring = Mirrored;
                else if(cMirror == 'd') currentParams.mirroring = Different;
                else retval = SL_STATUS_INVALID_PARAMETER;
              }
            break;
        case 's':
            vcrStop();
            break;
        case 'v':
            if(vcrRunning) retval = SL_STATUS_BUSY;
            else if(!vcrStart(&currentParams))
               { retval = SL_STATUS_INVALID_CONFIGURATION;
                 puts("Invalid vCR sequence -- bug!");
               }
            break;
        case 'A':
            vcrStop();
            retval = vcrAutoCal();
            break;
        case 'B':
            retval = bluetooth_action(cmd[1]);
            break;
        case 'D':
            { uint16_t finger = 0;
              int16_t nf;
              nf = sscanf(cmd+1, "%hu", &finger);
              if((nf == 1) && (finger < NFINGERS))  lraDumpRegisters(finger);
              else retval = SL_STATUS_INVALID_PARAMETER;
            }
            break;
        case 'E':
            { static bool requireEM1 = false;
              if((cmd[1] == '1') && !requireEM1)
                { sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);                          // Require EM1 to keep the VCOM UART RX channel working
                  requireEM1 = true;
                }
              else if((cmd[1] == '3') & requireEM1)
                { sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
                  requireEM1 = false;
                }
              printf("EM%c minimum\n", requireEM1 ? '1' : '3');
            }
            break;
        case 'F':
            {   // There is a bug in sscanf when scanning "%hhu %hhu" in which the second uint8_t value is not processed!
                // uint8_t nf = sscanf(cmd+1, "%1s %hhu %hhu", hand, &fv.finger, &fv.vibration);
                // So we work around it!
                uint16_t finger;
                uint8_t nf = sscanf(cmd+1, "%hu", &finger);
                vcrStop();
                if((nf == 1) && (finger < NFINGERS))  vcrFingerOn(NULL, (void*)((uint32_t)finger | (currentParams.intensity << 8)));
                else retval = SL_STATUS_INVALID_PARAMETER;
            }
            break;
        case 'I':
              { uint16_t finger = 0;
                int16_t nf;
                nf = sscanf(cmd+1, "%hu", &finger);
                vcrStop();
                if((nf == 1) && (finger < NFINGERS))
                  { lraInitResults_t ir;
                    if(lraInit(finger, &ir) == 0) statusPrint(finger);          // If initialization of the LRA driver while connected to this finger succeeded
                    else
                      { initFailPrint(finger, &ir);
                        retval = SL_STATUS_FAIL;
                      }
                  }
                else retval = SL_STATUS_INVALID_PARAMETER;
              }
            break;
        case 'M':
            displayEnabled = !displayEnabled;
            break;
        case 'Q':
            vcrStop();
            statusPrint(0);
            break;
        case 'R':
              { vcrStop();
                for(uint8_t finger = 0; finger < NFINGERS; finger++)
                  { int16_t iostat;
                    if((iostat = lraReset(finger)) != 0)
                      { retval = SL_STATUS_FAIL;
                        printf("Reset(%d) failure returned 0x%x\n", finger, iostat);
                      }
                  }
              }
            break;
        case 'V':
              { vcrStop();
                vcrParameters_t p;
                int8_t nf = sscanf(cmd+1, "%f %f %f %f %f", &p.fingerDuration, &p.fingerSpacing, &p.onDuration, &p.offDuration, &p.jitter);
                if(nf < 5) p.jitter = 0.0;
                if(nf < 4) p.offDuration = 0.0;
                if(nf < 3) p.onDuration = NFINGERS * p.fingerSpacing;
                p.differentJitter = currentParams.differentJitter;              // Retain current different jitter boolean value
                p.intensity = currentParams.intensity;                          // Retain current intensity level
                p.mirroring = currentParams.mirroring;                          // Retain current mirroring
                if(nf <= 0) printVibrationSequence("Current vCR sequence", &currentParams);
                else if((nf >= 2) && validVibrationSequence(&p)) currentParams = p;
                else retval = SL_STATUS_INVALID_CONFIGURATION;
              }
            break;
        case 'X':
              { sscanf(cmd+1, "%hhu", &I2CBusMux);
                printf("Finger select mode = %s mux\n", I2CBusMux ? "I2C bus" : "LRA FET");
              }
            break;
        case '@':
              { uint16_t finger, wreg, wvalue;
                int8_t nf = sscanf(cmd+1, "%hd %hx %hx", &finger, &wreg, &wvalue);
                uint8_t reg = wreg;
                uint8_t value = wvalue;
                if((nf == 3) && (finger < NFINGERS) && (wreg <= 255) && (wvalue <= 255))
                  { uint8_t oldvalue;
                    retval = lraGetRegister(finger, reg, &oldvalue);
                    printf("lraSetRegister(%d, 0x%.2x, 0x%.2x (%d)), previously was 0x%.2x (%d)\n", finger, reg, value, value, oldvalue, oldvalue);
                    retval = lraSetRegister(finger, reg, value);
                  }
                else retval = SL_STATUS_INVALID_PARAMETER;
              }
            break;
        default:
            // Echo input data back to host for use as event markers during data recording
            retval = SL_STATUS_INVALID_PARAMETER;
            break;
      }
    if(retval == SL_STATUS_INVALID_PARAMETER)           printf("%c command invalid parameter(s)\n", cmd[0]);
    else if(retval == SL_STATUS_INVALID_CONFIGURATION)  printf("%c command invalid configuration\n", cmd[0]);
    else if(retval == SL_STATUS_FAIL)                   printf("%c command failed\n", cmd[0]);
    else if(retval != SL_STATUS_OK)                     printf("%c command returned error code %0x.4x\n", cmd[0], retval);
    return(retval);
  }

void vcrInit(void)
  { const vcrParameters_t defaultSeq = { .fingerDuration = 100.0, .fingerSpacing = 166.666, .onDuration = 2000.0, .offDuration = 1333.333,
                                         .jitter = 0.0, .differentJitter = 0, .intensity = 127, .mirroring = Mirrored, .frequency = 0 };
    uint16_t iostat;
    uint32_t objectType;
    size_t objectSize;

    sl_bt_gatt_server_write_attribute_value(gattdb_firmware_revision_string, 0, sizeof(FW_REVISION), (uint8_t*)FW_REVISION);
    sl_bt_gatt_server_write_attribute_value(gattdb_software_revision_string, 0, sizeof(DATETIME), (uint8_t*)DATETIME);

    printSplash();

    if(I2CBusMux == 0)                                                        // If there is no I2C bus multiplexer, then we must have a fet multiplexer for a single LRA driver
      { for(uint8_t pin = 1; pin < 5; pin++)                                  // Configure the port B GPIO pins used to gate the finger multiplexer FETS
          { GPIO_PinModeSet(gpioPortB, pin, gpioModePushPull, 0);
            GPIO_PinOutClear(gpioPortB, pin);
          }
      }
    else                                                                      // Verify I2C bus mux presence
      { if(!lraTestI2CMux())
            printf("TCA4596A I2C Multiplexor did not respond properly!\n");
      }

    for(uint8_t finger = 0; finger < NFINGERS; finger++)
        if((iostat = lraReset(finger)) != 0) printf("Reset(%d) failure returned 0x%x\n", finger, iostat);

    // Read the default VCR timing parameters from non-volatile memory

    if((nvm3_getObjectInfo(nvm3_defaultHandle, NVM3_VCR_TIMING_PARAMETERS_KEY, &objectType, &objectSize) == ECODE_NVM3_OK) &&
       (objectType == NVM3_OBJECTTYPE_DATA) &&
       (objectSize == sizeof(currentParams)))
        nvm3_readData(nvm3_defaultHandle, NVM3_VCR_TIMING_PARAMETERS_KEY, &currentParams, sizeof(currentParams));
    else
      { puts("NVM vcrParameters mis-match.  Using programmed defaults.");
        currentParams = defaultSeq;
        if(storeVibrationSequence(&defaultSeq) != ECODE_NVM3_OK) puts("Failed to save default vcrParameters to NVM");
      }
  }
