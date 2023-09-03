#include <stddef.h>
#include "em_cmu.h"
#include "em_gpio.h"
#include "sl_assert.h"
#include "sl_i2cspm.h"
#include "sl_udelay.h"

I2C_TransferReturn_TypeDef i2c_slave_read (I2C_TypeDef *i2c, uint16_t addr, uint8_t command, uint8_t *val, uint8_t len)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef sta;
  uint8_t                    i2c_write_data[1];
  uint8_t                    i2c_read_data[32];
  uint8_t command_aux = command;
  int8_t idx = 0;

  do
  {
    seq.addr  = addr << 1;
    seq.flags = I2C_FLAG_WRITE_READ;
    /* Select command to issue */
    i2c_write_data[0] = command_aux;
    seq.buf[0].data   = i2c_write_data;
    seq.buf[0].len    = 1;
    /* Select location/length of data to be read */
    seq.buf[1].data = i2c_read_data;
    seq.buf[1].len  = len;

    sta = I2CSPM_Transfer(i2c, &seq);

    if (sta != i2cTransferDone)
    {
      return sta;
    }

    if (NULL != val)
    {
      *(val + idx) = i2c_read_data[0];
    }
    command_aux++;
    idx++;
  }while (--len);

  return sta;
}

I2C_TransferReturn_TypeDef i2c_slave_write (I2C_TypeDef *i2c, uint8_t addr, uint8_t command, uint8_t *val, uint8_t len)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef sta;
  uint8_t                    i2c_write_cmd[1];
  uint8_t                    i2c_write_data[32];
  uint8_t command_aux = command;
  int8_t idx = 0;

  do
  {
    seq.addr  = addr << 1;
    seq.flags = I2C_FLAG_WRITE_WRITE;
    /* Select command to issue */
    i2c_write_cmd[0] = command_aux;
    seq.buf[0].data   = i2c_write_cmd;
    seq.buf[0].len    = 1;
    /* Select location/length of data to be read */
    seq.buf[1].data = (val + idx);
    seq.buf[1].len  = 1;

    sta = I2CSPM_Transfer(i2c, &seq);

    if (sta != i2cTransferDone)
    {
      return sta;
    }
    command_aux++;
    idx++;
  }while (--len);

  return (sta);
}

