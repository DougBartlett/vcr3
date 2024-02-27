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

#ifndef VCR_H
#define VCR_H
#include <sys/types.h>

#define  NFINGERS          4
#define  TICKSPERMS        32.768
#define  VIBRATIONDELAY    ((int)(500 * TICKSPERMS))                   // All vibrations are delayed by 500mS to allow the remove glove to compensate for communications latency!

typedef enum { Non_Mirrored, Mirrored, Different } mirror_t;

typedef struct vcrParameters
  { float    fingerDuration;              // Time for which 1 finger vibrates (mS).
    float    fingerSpacing;               // Time between start of 1 finger vibration and start of next finger vibration (mS).
    float    onDuration;                  // Time period over which fingers are randomly vibrated (mS).  Should be a multiple of 4 x fingerSpacing.
    float    offDuration;                 // Time period over which no finger vibations occur (mS).
    float    jitter;                      // Magnitude of time jitter (mS) if non-zero, a random jitter up to this value is applied to each fingers start/stop times
    uint8_t  differentJitter;             // Flag:  0 = same jitter to each hand; non-zero = different jitter
    uint8_t  intensity;                   // Vibration intensity/strength (0-127)
    mirror_t mirroring;                   // Specifies if/how to apply vibration pattern to the fingers of two hands
    uint16_t frequency;                   // Desired LRA vibration frequency in Hz or 0 for auto-resonance mode
  } vcrParameters_t;

  typedef struct vibrateSet
  { uint32_t start_time[NFINGERS];        // The relative time from “now” when each finger begins vibrating (timerticks)
    uint32_t duration[NFINGERS];          // How long a each finger vibration lasts (timerticks)
    uint16_t intensity;                   // The intensity (strength) of vibration for each of fingers 0..3 (0-255)
                                          // Note that the vibrateSet_t does not sent either VCR mirroring or frequency to the remote glove.  This must be handled separately!
  } vibrateSet_t;

extern uint8_t fingerSelectI2C;
extern vcrParameters_t currentParams;

void    vcrVibrateSet(vibrateSet_t *vset);
int16_t vcrAutoCal(void);
int16_t vcrStart(vcrParameters_t *p);
void    vcrStop();
int16_t vcrProcessCommand(char *cmd);
void    vcrInit(void);

#endif
