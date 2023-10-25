/*
 * IMU_funcs.h
 *
 *  Created on: 9 февр. 2022 г.
 *      Author: User
 */

#ifndef INC_IMU_FUNCS_H_
#define INC_IMU_FUNCS_H_

#include <spi.h>
#include <i2c.h>
/*typedef struct
  {
    uint64_t mTimestamp; //!< Штамп времени в нс
    uint32_t mTimeS;     //!< Секунды штампа времени
    uint32_t mTimeNS;    //!< Наносекунды штампа времени
    float    mX;         //!< Координаты
    float    mY;
    float    mZ;
    float    mW;
    float    mAcc;       //!< Точность представления

    uint8_t  mSensorId;  //!< Идентификатор сенсора
  } Quaternion;*/

void usbPrintf( const char *format, ... );

//void uartPrintf( const char *format, ... );

void spi_init( SPI_HandleTypeDef *hspi );

int8_t spiTransmit( uint8_t *data, uint16_t size );

int8_t spiReceiv( uint8_t *data, uint16_t size );

int8_t bhy2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

int8_t bhy2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

void i2c_init( I2C_HandleTypeDef *hi2c, uint8_t devAddr );

int8_t i2cTransmit( uint8_t *data, uint16_t size );

int8_t i2cReceiv( uint8_t *data, uint16_t size );

int8_t bhy2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

int8_t bhy2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

void bhy2_delay_us(uint32_t us, void *private_data);

#endif /* INC_IMU_FUNCS_H_ */
