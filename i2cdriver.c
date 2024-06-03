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
#include "i2cdriver.h"
#include <string.h>
#include "sl_i2cspm.h"
#include <sl_i2cspm_instances.h>

#define i2cInvalidBus   -10


int16_t i2c_read(const uint8_t address, uint8_t reg, void *data, const uint8_t size)
  { I2C_TransferSeq_TypeDef seq;
    seq.addr = address << 1;
    seq.flags = I2C_FLAG_WRITE_READ;
    seq.buf[0].data = &reg;
    seq.buf[0].len = 1;
    seq.buf[1].data = data;
    seq.buf[1].len = size;
    return(I2CSPM_Transfer(sl_i2cspm_mikroe, &seq));
  }

int16_t i2c_write(const uint8_t address, uint8_t reg, void *data, const uint8_t size)
  { I2C_TransferSeq_TypeDef seq;
    seq.addr = address << 1;
    seq.flags = I2C_FLAG_WRITE_WRITE;
    seq.buf[0].data = &reg;
    seq.buf[0].len = 1;
    seq.buf[1].data = data;
    seq.buf[1].len = size;
    return(I2CSPM_Transfer(sl_i2cspm_mikroe, &seq));
  }

int16_t i2c_single_read(const uint8_t address, uint8_t *reg)
  { I2C_TransferSeq_TypeDef seq;
    seq.addr = address << 1;
    seq.flags = I2C_FLAG_READ;
    seq.buf[0].data = &reg;
    seq.buf[0].len = 1;
    seq.buf[1].data = NULL;
    seq.buf[1].len = 0;
    return(I2CSPM_Transfer(sl_i2cspm_mikroe, &seq));
  }

void i2c_getErrorMessage(int16_t status, char *strptr, uint8_t maxlen)
  { char *mp;
    switch(status)
      { case i2cTransferInProgress:
           mp = "Transfer in Progress";
           break;
        case i2cTransferDone:
            mp = "Success";
            break;
        case i2cTransferNack:
            mp = "NACKed";
            break;
        case i2cTransferBusErr:
            mp = "Bus Error";
            break;
        case i2cTransferArbLost:
            mp = "Arb Lost";
            break;
        case i2cTransferUsageFault:
            mp = "Usage Fault";
            break;
        case i2cTransferSwFault:
            mp = "SW Fault";
            break;
        case i2cInvalidBus:
            mp = "Invalid Bus";
            break;
        default:
            mp = "Unknown Error";
            break;
      }

    if(strptr != NULL) strncpy(strptr, mp, maxlen);
  }
