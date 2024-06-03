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

#ifndef LRA_H
#define LRA_H

#include <sys/types.h>

#define LRAFMIN      80
#define LRAFMAX      495

typedef struct
  { uint8_t a_cal_comp;
    uint8_t a_cal_bemf;
    uint8_t bemf_gain;
  } drv2605_autocal_t;

typedef struct
  { uint8_t io_failed     :1;
    uint8_t diag_failed   :1;
    uint8_t acal_failed   :1;
    uint8_t shorted       :1;
    uint8_t over_temp     :1;
    uint8_t pad           :3;
  } lraInitResults_t;

uint8_t lraTestI2CMux();
int16_t lraReset(uint8_t ch);
int16_t lraInit(uint8_t ch, lraInitResults_t *results);
int16_t lraGetAutoCalParams(uint8_t ch, drv2605_autocal_t *acp);
int16_t lraSetAutoCalParams(uint8_t ch, drv2605_autocal_t *acp);
int16_t lraSetFrequency(uint8_t ch, uint16_t frequency);
int16_t lraSetEffectSequence(uint8_t ch, uint8_t *sequence, uint8_t len);
int16_t lraPlayEffectSequence(uint8_t ch);
int16_t lraGetBatteryVoltage(uint8_t ch);
int16_t lraGetResonantFrequency(uint8_t ch);
int16_t lraGetStatus(uint8_t ch);
int16_t lraDumpRegisters(uint8_t ch);
int16_t lraGetRegister(uint8_t ch, uint8_t reg, uint8_t *value);
int16_t lraSetRegister(uint8_t ch, uint8_t reg, uint8_t value);

int16_t lraStandby(uint8_t ch);
int16_t lraActive(uint8_t ch);
int16_t lraStart(uint8_t ch, uint8_t intensity);
int16_t lraStop(uint8_t ch);
#endif
