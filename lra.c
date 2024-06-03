/* ========================================
 *
 * Copyright Douglas E. Bartlett, 2018-2023
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Douglas E. Bartlett.
 *
 * ========================================
*/
#include "vcr.h"
#include "lra.h"
#include "i2cdriver.h"
#include <stdio.h>
#include <em_gpio.h>

#define DRV2605_BROADCAST               0x58
#define DRV2605_ADDRESS                 0x5A        // I2C Slave address of the DRV2605 chips
#define TCA9546A_ADDRESS                0x70        // I2C Slave address of the TCA9546A I2C multiplexor
#define NCHANNELS                       4           // Number of LRA channels available


// Below are register number constants and register access structures for the TI DRV2605L LRA driver chip

#define REG_STATUS                      0x00
    typedef union
      { uint8_t byte;
        struct
          { uint8_t over_current  :1;
            uint8_t over_temp     :1;
            uint8_t reserved2     :1;
            uint8_t diag_result   :1;
            uint8_t reserved4     :1;
            uint8_t device_id     :3;
          };
      } drv2605_status_t;
#define REG_MODE                        0x01
    typedef union
      { uint8_t byte;
        struct
          { uint8_t mode          :3;
            uint8_t reserved      :3;
            uint8_t standby       :1;
            uint8_t reset         :1;
          };
      } drv2605_mode_t;
    #define MODE_GOTRIGGER              0x00
    #define MODE_EXTEDGETRIGGER         0x01
    #define MODE_EXTLEVELTRIGGER        0x02
    #define MODE_RTP                    0x05
    #define MODE_DIAGNOSTICS            0x06
    #define MODE_AUTOCAL                0x07
#define REG_RTP_INPUT                   0x02
#define REG_LIBSELECT                   0x03
    #define LIBSELECTLRA                6
#define REG_WAVESEQUENCER0              0x04        // continues through 0x0B
#define REG_GO                          0x0C
#define REG_OVERDRIVETIMEOFFSET         0x0D        // Overdrive Time Offset (ms) = ODT[7:0] × 5 ms
#define REG_SUSTAINPOSTIMEOFFSET        0x0E        // Sustain-Time Positive Offset (ms) = SPT[7:0] × 5 ms
#define REG_SUSTAINNEGTIMEOFFSET        0x0F        // Sustain-Time Negative Offset (ms) = SNT[7:0] × 5 ms
#define REG_BRAKETIMEOFFSET             0x10        // Brake Time Offset (ms) = BRT[7:0] × 5 ms
#define REG_RATEDVOLTAGE                0x16        // See equation 3 in DRV2605 datasheet.  Must be written before autocalibration!
#define REG_ODVOLTAGECLAMP              0x17        // 0.02196 x ODC[7:0].  Must be written before autocalibration!
#define REG_A_CAL_COMP                  0x18
#define REG_A_CAL_BEMF                  0x19
#define REG_FEEDBACKCONTROL             0x1A
    typedef union
      { uint8_t byte;
        struct
          { uint8_t bemf_gain     :2;
            uint8_t loop_gain     :2;
            uint8_t brake_factor  :3;
            uint8_t n_erm_lra     :1;
          };
      } drv2605_feedbackcontrol_t;
#define REG_CONTROL1                    0x1B
    typedef union
      { uint8_t byte;
        struct
          { uint8_t drive_time    :5;                 // Drive time (ms) = DRIVE_TIME[4:0] × 0.1 ms + 0.5 ms
            uint8_t ac_couple     :1;
            uint8_t reserved6     :1;
            uint8_t start_boot    :1;
          };
      } drv2605_control1_t;
#define REG_CONTROL2                    0x1C
    typedef union
      { uint8_t byte;
        struct
          { uint8_t idiss_time    :2;
            uint8_t blanking_time :2;
            uint8_t sample_time   :2;
            uint8_t brake_stabil  :1;
            uint8_t bidi_input    :1;
          };
      } drv2605_control2_t;
#define REG_CONTROL3                    0x1D
    typedef union
      { uint8_t byte;
        struct
          { uint8_t lra_open_loop :1;
            uint8_t n_pwm_analog  :1;
            uint8_t lra_drive_mode  :1;
            uint8_t data_format_rtp :1;
            uint8_t supply_comp_dis :1;
            uint8_t erm_open_loop :1;
            uint8_t ng_thresh     :2;
          };
      } drv2605_control3_t;
#define REG_CONTROL4                    0x1E
    typedef union
      { uint8_t byte;
        struct
          { uint8_t otp_program   :1;
            uint8_t reserved1     :1;
            uint8_t otp_status    :1;
            uint8_t reserved3     :1;
            uint8_t auto_cal_time :2;
            uint8_t reserved67    :2;
          };
      } drv2605_control4_t;
#define REG_OPENLOOPPERIOD              0x20
#define REG_VBATTERY                    0x21        // VDD (V) = VBAT[7:0] × 5.6V / 255
#define REG_LRAPERIOD                   0x22        // LRA period (us) = LRA_Period[7:0] × 98.46 μs

static inline int8_t lraSelectChannel(uint8_t ch)                             // Set the LRA multiplexor to select the desired channel
  { static uint8_t priorCh = 255;
    if(ch >= NCHANNELS) return(-1);
    if(ch != priorCh)                                                         // Avoid glitches on LRA select lines if reselecting the same finger
      { if(I2CBusMux)
            i2c_write(TCA9546A_ADDRESS, 1 << ch, NULL, 0);
        else
          { GPIO_PortOutClear(gpioPortB, 0x1E);                               // Set all LRA select lines low
            // Port/Pin assignments:  F0 = B4, F1 = B3, F2 = B2, F3 = B1
            GPIO_PortOutSet(gpioPortB, 0x02 << ((NCHANNELS-1) - ch));         // Set the one LRA select line for the desired finger high
          }
        priorCh = ch;                                                         // Remember the most recently selected finger
      }
    return(ch);
  }

uint8_t lraTestI2CMux()
  { for(uint8_t wdata = 0; wdata < 16; wdata++)
      { uint8_t rdata;
        int16_t writeOk = i2c_write(TCA9546A_ADDRESS, wdata, NULL, 0);
        int16_t readOk  = i2c_single_read(TCA9546A_ADDRESS, &rdata);
        if((writeOk != 0) && (readOk != 0) && (wdata != rdata)) return(0);
      }
    return(1);
  }

static uint8_t waitUntilGoClear(void)
  { int16_t iostat;
    uint8_t go;
    do
      { iostat = i2c_read(DRV2605_ADDRESS, REG_GO, &go, 1);
      }
    while((iostat == 0) && (go & 0x01));
    if(iostat != 0) return(iostat);
    else return(go);
  }

int16_t lraReset(uint8_t ch)
  { int16_t iostat;
    drv2605_status_t status;
    drv2605_mode_t mode;
    if(lraSelectChannel(ch) < 0) return(-1);
    if((iostat = i2c_read(DRV2605_ADDRESS, REG_STATUS, &status, 1)) != 0)                 // Read status register to see if dvr2605 is present
        return(iostat);
    if(status.device_id != 7)                                                             // Test for expected DRV2605 device id value
      { printf("ch%d wrong device id (%d)\n", ch, status.device_id);
        return(-1);
      }
    mode.reset = 1;  mode.standby = 0;  mode.mode = MODE_GOTRIGGER;
    if((iostat = i2c_write(DRV2605_ADDRESS, REG_MODE, &mode, 1)) != 0)                    // Reset device
        return(iostat);
    for(uint8_t i = 10; i ; i--)
      { iostat = i2c_read(DRV2605_ADDRESS, REG_MODE, &mode, 1);                           // Read back mode register to verify that reset bit clears
        // char errmsg[120];
        // printf("iostat = %u  mode = 0x%.2x  status = 0x%.2x (%s)\n", iostat, mode.byte, status, errmsg);
        // if(i < 8) CyDelay(100);
        if((iostat == 0) && (mode.reset == 0)) break;
      }
    if(mode.reset == 1)                                                                   // If reset failed to clear, return error
        return(-1);
    mode.reset = 0;  mode.standby = 0;  mode.mode = MODE_GOTRIGGER;
    if((iostat = i2c_write(DRV2605_ADDRESS, REG_MODE, &mode, 1)) != 0)                    // Exit standby state
        return(iostat);
    if((iostat = i2c_read(DRV2605_ADDRESS, REG_MODE, &mode, 1)) != 0)                     // Read back mode register to verify that it left standby
        return(iostat);
    if((mode.reset == 1) || (mode.standby == 1)) return(-1);

    union                                                                                 // Declare packed structure for all 5 control registers
      { uint8_t bytes[9];
        struct
          { uint8_t ratedvoltage;
            uint8_t od_clamp;
            uint8_t a_cal_comp;
            uint8_t a_cal_bemf;
            drv2605_feedbackcontrol_t fbc;
            drv2605_control1_t ctrl1;
            drv2605_control2_t ctrl2;
            drv2605_control3_t ctrl3;
            drv2605_control4_t ctrl4;
          };
      } crs;

    for(uint8_t i = 0; i < sizeof(crs); i++) crs.bytes[i] = 0;                            // Fill structure with zeros

    // Initialize the control register setting values in structure crs before writing the entire sequence to the DRV2605
    crs.ratedvoltage = 70;                                                                // 1.8 V RMS AC Sign wave modified by datasheet equation
    crs.od_clamp = 94;                                                                    // 1.8 volts / 0.0192
    crs.a_cal_comp = 0x0F;                                                                // Value determined heuristically by characterizing 8mm diameter LRAs
    crs.a_cal_bemf = 0x88;                                                                // Value determined heuristacally by characterizing 8mm diameter LRAs
    crs.fbc.n_erm_lra = 1;                                                                // a. ERM_LRA — selection will depend on desired actuator.
    crs.fbc.brake_factor = 3;                                                             // b. FB_BRAKE_FACTOR[2:0] — A value of 2 is valid for most actuators.
    crs.fbc.loop_gain = 1;                                                                // c. LOOP_GAIN[1:0] — A value of 2 is valid for most actuators.
    crs.fbc.bemf_gain = 2;
    crs.ctrl4.auto_cal_time = 3;                                                          // f. AUTO_CAL_TIME[1:0] — A value of 3 is valid for most actuators.
    crs.ctrl1.start_boot = 1;
    crs.ctrl1.drive_time = 16;                                                            // g. DRIVE_TIME[3:0] - 2.128mS is the half period for a 235 Hz nominal resonance
    crs.ctrl2.bidi_input = 1;
    crs.ctrl2.brake_stabil = 1;
    crs.ctrl2.sample_time = 3;                                                            // h. SAMPLE_TIME[1:0] — A value of 3 is valid for most actuators.
    crs.ctrl2.blanking_time = 1;                                                          // i. BLANKING_TIME[1:0] — A value of 1 is valid for most actuators.
    crs.ctrl2.idiss_time = 1;                                                             // j. IDISS_TIME[1:0] — A value of 1 is valid for most actuators.
    crs.ctrl3.ng_thresh = 2;
    crs.ctrl3.erm_open_loop = 1;
    crs.ctrl3.supply_comp_dis = 0;
    crs.ctrl3.data_format_rtp = 0;
    crs.ctrl3.lra_drive_mode = 0;
    crs.ctrl3.n_pwm_analog = 0;
    crs.ctrl3.lra_open_loop = 0;

    // printf("ch%d FB & Control Reg values(%d): %.2x, %.2x, %.2x, %.2x, %.2x\n", ch, sizeof(crs), crs.bytes[4], crs.bytes[5], crs.bytes[6], crs.bytes[7], crs.bytes[8]);
    if((iostat = i2c_write(DRV2605_ADDRESS, REG_RATEDVOLTAGE, &crs, sizeof(crs))) != 0)          // Set the control registers from rated_voltage (0x16) through control4 (0x1E)
        return(iostat);

    mode.reset = 0; mode.standby = 1; mode.mode = MODE_RTP;
    if((iostat = i2c_write(DRV2605_ADDRESS, REG_MODE, &mode, 1)) != 0)                    // Place DRV2605 into Standby state & RTP mode
        return(iostat);
    return(0);
  }

 int16_t lraInit(uint8_t ch, lraInitResults_t *results)
  { int16_t iostat;
    drv2605_status_t status;
    drv2605_mode_t mode;
    uint8_t go = 1;
    if(lraSelectChannel(ch) < 0) return(-1);
    if(results == NULL) return(-1);
    // static_assert(sizeof(lraInitResults_t) == 1, "lraInitResults_t is not 1 byte in size!");
    results->io_failed = 1;                                                               // Presume an I/O failure so early return(0)'s have proper initResults flags
    results->diag_failed = 0;
    results->acal_failed = 0;
    results->shorted = 0;
    results->over_temp = 0;

    mode.byte = 0; mode.mode = MODE_DIAGNOSTICS;                                          // Configure device for diagnostics
    if((iostat = i2c_write(DRV2605_ADDRESS, REG_MODE, &mode, 1)) != 0)                    // Place device in diagnostics mode
        return(iostat);
    if((iostat = i2c_write(DRV2605_ADDRESS, REG_GO, &go, 1)))                             // Set the GO bit to start diagnostics
        return(iostat);
    if((iostat = waitUntilGoClear()) != 0) return(iostat);                                // Spin until the GO bit is cleared (diagnostics completed)
    if((iostat = i2c_read(DRV2605_ADDRESS, REG_STATUS, &status, 1)) != 0)                 // Read status register to see if diagnostics was successful
        return(iostat);
    if(status.diag_result == 1)                                                           // If diagnostics test failed, set appropriate initResults flags
      { results->diag_failed = 1;
        if(status.over_current) results->shorted = 1;
        if(status.over_temp)    results->over_temp = 1;
      }

    mode.byte = 0; mode.mode = MODE_AUTOCAL;                                              // Configure device for autocal
    if((iostat = i2c_write(DRV2605_ADDRESS, REG_MODE, &mode, 1)) != 0)                    // Place device in autocal mode
        return(iostat);
    if((iostat = i2c_write(DRV2605_ADDRESS, REG_GO, &go, 1)) != 0)                        // Set the GO bit to start autocal
        return(iostat);
    if((iostat = waitUntilGoClear()) != 0) return(iostat);                                // Spin until the GO bit is cleared (autocal completed)
    if((iostat = i2c_read(DRV2605_ADDRESS, REG_STATUS, &status, 1)) != 0)                 // Read status register to see if autocal was successful
        return(iostat);
    if(status.diag_result == 1)                                                           // If autocal failed, set appropriate initResults flags
      { results->acal_failed = 1;
        if(status.over_current) results->shorted = 1;
        if(status.over_temp)    results->over_temp = 1;
      }

    mode.reset = 0;  mode.standby = 0;  mode.mode = MODE_GOTRIGGER;
    if((iostat = i2c_write(DRV2605_ADDRESS, REG_MODE, &mode, 1)) != 0)                    // Exit standby state
        return(iostat);
    uint8_t intensity = 0;
    if((iostat = i2c_write(DRV2605_ADDRESS, REG_RTP_INPUT, &intensity, 1)) != 0)          // Set RTP intensity to zero
        return(iostat);
    mode.byte = 0;   mode.standby = 0;  mode.mode = MODE_RTP;
    if((iostat = i2c_write(DRV2605_ADDRESS, REG_MODE, &mode, 1)) != 0)                    // Configure device for RTP mode
        return(iostat);
    results->io_failed = 0;                                                               // OK, now we know there were no IOErrors
    if(results->diag_failed || results->acal_failed || results->shorted || results->over_temp || results->io_failed) return(-1);
    else return(0);
  }

int16_t lraGetAutoCalParams(uint8_t ch, drv2605_autocal_t *acp)
  { int16_t iostat;
    if(lraSelectChannel(ch) < 0) return(-1);
    if(acp == NULL) return(-1);
    if((iostat = i2c_read(DRV2605_ADDRESS, REG_A_CAL_COMP, acp, 3)) == 0)                 // Read A_CAL_COMP, A_CAL_BEMF & FEEDBACKCONTROL registers
        acp->bemf_gain &= 0x03;                                                           // Mask off the 2 bemf_comp bits
    return(iostat);
  }

int16_t lraSetAutoCalParams(uint8_t ch, drv2605_autocal_t *acp)
  { int16_t iostat;
    uint8_t fbc;
    drv2605_autocal_t ac;
    if(lraSelectChannel(ch) < 0) return(-1);
    if(acp == NULL) return(-1);
    if((iostat = i2c_read(DRV2605_ADDRESS, REG_FEEDBACKCONTROL, &fbc, 1)) != 0)           // Read the FEEDBACKCONTROL register (for r/modify/w)
        return(iostat);
    ac.a_cal_comp = acp->a_cal_comp;                                                      // Copy args into temp ac structure
    ac.a_cal_bemf = acp->a_cal_bemf;
    ac.bemf_gain  = ((fbc & 0xFC) | (acp->bemf_gain & 0x03));                             // Merge the new bemf_gain into temp ac structure
    return(i2c_write(DRV2605_ADDRESS, REG_FEEDBACKCONTROL, &ac, 3));                      // Write merged autocal parameters to device
  }

int16_t lraSetFrequency(uint8_t ch, uint16_t frequency)
  { int16_t iostat;
    uint8_t period;
    drv2605_control3_t regcntl3;
    if(lraSelectChannel(ch) < 0) return(-1);
    if((frequency == 0) || ((frequency >= LRAFMIN) && (frequency <= LRAFMAX)))
      { iostat = i2c_read(DRV2605_ADDRESS, REG_CONTROL3, &regcntl3, sizeof(regcntl3));    // Read REG_CONTROL3 for read/modify/write
        if(frequency == 0)
          { regcntl3.lra_open_loop = 0;                                                   // Clear open loop bit
            period = 0;
          }
        else
          { regcntl3.lra_open_loop = 1;                                                   // Set open loop bit
            period = 10153.315 / frequency + 0.5;                                         // Compute rounded LRA open loop period register value
          }
        iostat = i2c_write(DRV2605_ADDRESS, REG_OPENLOOPPERIOD, &period, 1);
        iostat = i2c_write(DRV2605_ADDRESS, REG_CONTROL3, (uint8_t*)&regcntl3, sizeof(regcntl3));
      }
    else iostat = -1;
    return(iostat);
  }

int16_t lraSetEffectSequence(uint8_t ch, uint8_t *sequence, uint8_t len)
  { if(lraSelectChannel(ch) < 0) return(-1);
    if(len > 8) return(-1);                                                               // Check for sequence that is too long to fit DRV2605
    return(i2c_write(DRV2605_ADDRESS, REG_WAVESEQUENCER0, sequence, len));                // Write new effect sequence to the sequence registers
  }

int16_t lraPlayEffectSequence(uint8_t ch)
  { int16_t iostat;
    drv2605_mode_t mode;
    uint8_t libsel = LIBSELECTLRA;                                                        // Setup use of the DRV2605 LRA vibration library and sequencer
    if(lraSelectChannel(ch) < 0) return(-1);
    if((iostat = i2c_write(DRV2605_ADDRESS, REG_LIBSELECT, &libsel, 1)) != 0)             // Select the LRA library
        return(iostat);
    mode.mode = MODE_GOTRIGGER;
    if((iostat = i2c_write(DRV2605_ADDRESS, REG_MODE, &mode, 1)) != 0)                    // Put DRV2605 in active GO trigger mode
        return(iostat);
    uint8_t go = 1;
    if((iostat = i2c_write(DRV2605_ADDRESS, REG_GO, &go, 1)) != 0)                        // Set the GO bit to start effect sequence
        return(iostat);
    return(0);
 }

int16_t lraGetBatteryVoltage(uint8_t ch)
  { int16_t iostat;
    uint8_t data;
    if(lraSelectChannel(ch) < 0) return(-1);
    if((iostat = i2c_read(DRV2605_ADDRESS, REG_VBATTERY, &data, 1)) == 0)                 // Read battery voltage register
        return((5600L * data) / 255);
    else return(-1);
  }

int16_t lraGetResonantFrequency(uint8_t ch)
  { int16_t iostat;
    uint8_t data;
    if(lraSelectChannel(ch) < 0) return(-1);
    if((iostat = i2c_read(DRV2605_ADDRESS, REG_LRAPERIOD, &data, 1)) == 0)                // Read resonant period register
        return((10156L + (data >> 1)) / data);                                            // Convert period to rounded frequency (hZ)
    else return(-1);
  }

int16_t lraGetStatus(uint8_t ch)
  { int16_t iostat;
    uint8_t data;
    if(lraSelectChannel(ch) < 0) return(-1);
    if((iostat = i2c_read(DRV2605_ADDRESS, REG_STATUS, &data, 1)) == 0)                   // Read DRV2605L status register
        return(data);      // Read status register
    else return(-1);
  }

int16_t lraDumpRegisters(uint8_t ch)
  { int16_t iostat;
    uint8_t data[35];
    if(lraSelectChannel(ch) < 0) return(-1);
    if((iostat = i2c_read(DRV2605_ADDRESS, 0, &data, sizeof(data))) == 0)                 // Read registers
        for(uint8_t reg = 0; reg < sizeof(data); reg++)
            printf("reg[0x%.2x] = 0x%.2x\n", reg, data[reg]);
    return(iostat);
  }

int16_t lraGetRegister(uint8_t ch, uint8_t reg, uint8_t *value)
  { if(lraSelectChannel(ch) < 0) return(-1);
    return(i2c_read(DRV2605_ADDRESS, reg, value, sizeof(*value)));
  }

int16_t lraSetRegister(uint8_t ch, uint8_t reg, uint8_t value)
  { if(lraSelectChannel(ch) < 0) return(-1);
    return(i2c_write(DRV2605_ADDRESS, reg, &value, sizeof(value)));
  }

int16_t lraStandby(uint8_t ch)
  { drv2605_mode_t mode;
    mode.standby = 1; mode.mode = MODE_RTP;
    if(lraSelectChannel(ch) < 0) return(-1);
    return(i2c_write(DRV2605_ADDRESS, REG_MODE, &mode, 1));                               // Place DRV2605 into Standby state
  }

int16_t lraActive(uint8_t ch)
  { drv2605_mode_t mode;
    mode.byte = 0; mode.mode = MODE_RTP;
    if(lraSelectChannel(ch) < 0) return(-1);
    return(i2c_write(DRV2605_ADDRESS, REG_MODE, &mode, 1));                               // Place DRV2605 into RTP mode
  }

int16_t lraStart(uint8_t ch, uint8_t intensity)
  { drv2605_mode_t mode;
    if(lraSelectChannel(ch) < 0) return(-1);
    mode.byte = 0; mode.mode = MODE_RTP;
    i2c_write(DRV2605_ADDRESS, REG_MODE, &mode, 1);                                       // Place DRV2605 into RTP mode & out of standby
    return(i2c_write(DRV2605_ADDRESS, REG_RTP_INPUT, &intensity, 1));                     // Set RTP intensity
  }

int16_t lraStop(uint8_t ch)
  { uint8_t zero = 0;
    drv2605_mode_t mode;
    if(lraSelectChannel(ch) < 0) return(-1);
    i2c_write(DRV2605_ADDRESS, REG_RTP_INPUT, &zero, 1);                                  // Set RTP intensity to zero (off)
    mode.reset = 0; mode.standby = 1; mode.mode = MODE_RTP;
    return(i2c_write(DRV2605_ADDRESS, REG_MODE, &mode, 1));                               // Place DRV2605 into Standby state
  }
