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

#ifndef APP_H
#define APP_H
#include <sys/types.h>
#include <sl_bgapi.h>
#include "vcr.h"

typedef struct
  { uint32_t lastTxTime;
    uint32_t scheduledTxTime;
    uint16_t sequenceNumber;
    vibrateSet_t set;
  } vibrateCharacteristic_t;

void bd_addr_to_string(bd_addr address, char *str, uint16_t maxlen);
uint16_t string_to_bd_addr(char *str, bd_addr *address);
uint16_t bluetooth_action(char action);
uint16_t remoteVibrateSet(vibrateSet_t *set);

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/

void appPrintSplash(void);
void app_init(void);

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void);

#endif  // APP_H
