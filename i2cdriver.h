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

#ifndef I2CDRIVER_H
#define I2CDRIVER_H
#include <sys/types.h>

int16_t i2c_read(const uint8_t address, uint8_t reg, void *data, const uint8_t size);
int16_t i2c_write(const uint8_t address, uint8_t reg, void *data, const uint8_t size);
int16_t i2c_single_read(const uint8_t address, uint8_t *reg);
void    i2c_getErrorMessage(int16_t status, char *strptr, uint8_t maxlen);

#endif
