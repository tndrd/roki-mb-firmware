/*
 * IMU_funcs.c
 *
 *  Created on: 9 февр. 2022 г.
 *      Author: User
 */

#include "usb_device.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include "usbd_cdc_if.h"


#include "bhy2.h"
#include "bhy2_hif.h"
#include "bhy2_parse.h"
#include "bhy2_defs.h"

#include "IMU_funcs.h"
#include <stm32h7xx_hal.h>

//#include "Bosch_SHUTTLE_BHI260.fw.h"
//Порт, на котором находится imu
//#define BHY2_I2C hi2c2

//Собственный адрес микросхемы
//#define BHY2_ADDR (0x28 << 1)

//Идентификатор сенсора (микросхема содержит кучу виртуальных сенсоров)
#define QUAT_SENSOR_ID     BHY2_SENSOR_ID_GAMERV
//Размер рабочего буфера

//Ошибки обмена с imu
#define IMU_ERR_OK   0
#define IMU_ERR_I2C -1
#define IMU_ERR_SPI -2

I2C_HandleTypeDef *mI2c;        //!< Порт I2C, по которому осуществляется обмен
uint8_t            mDevAddress; //!< Адрес устройства, с которым выполняется обмен

SPI_HandleTypeDef  *mSPI;       //!< Порт SPI, по которому осуществляется обмен


void usbPrintf( const char *format, ... )
  {
  static char line[128];
  va_list args;

  /* fprintf() */
  va_start( args, format );
  vsprintf( line, format, args );
  va_end(args);

  CDC_Transmit_FS( (uint8_t*)line, strlen(line) );
  }

void spi_init( SPI_HandleTypeDef *hspi)
{
  mSPI = hspi;
}

//!
//! \brief spiTransmit Передача с использованием канала DMA
//! \param data        Данные для передачи
//! \param size        Количество байт для передачи
//! \return            true - если операция выполнена успешно
//!
int8_t spiTransmit( uint8_t *data, uint16_t size )
  {
  //Инициировать передачу по DMA
	HAL_SPI_Transmit_DMA( mSPI, data, size );
	while (HAL_SPI_GetState(mSPI) != HAL_SPI_STATE_READY);
	//HAL_SPI_Transmit(mSPI, data, size, 5);
  //Возвращает истину при успешном завершении операции
  return mSPI->ErrorCode == HAL_SPI_ERROR_NONE;
  }

//!
//! \brief spiReceiv Прием с использованием канала DMA
//! \param data      Указатель на буфер, в который размещаются принятые данные
//! \param size      Количество принимаемых байтов
//! \return            true - если операция выполнена успешно
//!
int8_t spiReceiv( uint8_t *data, uint16_t size )
  {
  //Инициировать передачу по DMA
	//HAL_SPI_Receive(mSPI, data, size,  5);
	HAL_SPI_Receive_DMA( mSPI, data, size );
  //Ожидать завершения приема
	while (HAL_SPI_GetState(mSPI) != HAL_SPI_STATE_READY);
  //Возвращает истину при успешном завершении операции
  return mSPI->ErrorCode == HAL_SPI_ERROR_NONE;
  }

//!
//! \brief bhy2_spi_read Прочитать данные из некоторого регистра
//! \param reg_addr      Адрес регистра
//! \param reg_data      Буфер, в который нужно записать прочитанные из регистра данные
//! \param length        Длина данных
//! \param intf_ptr      Указатель на интерфейс (не используем)
//! \return              Результат операции
//!
int8_t bhy2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
  {
  //I2cDmaTransfer *i2cDmaTransfer = (I2cDmaTransfer*)intf_ptr;
  //Выставляем CS в 0
  //HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
  //GPIOA->BSRR = (uint32_t)SPI_CS_Pin << (16U);
  GPIOE->BSRR = (uint32_t)IMU_CS_Pin << (16U);

  //Записываем адрес регистра
  if( !spiTransmit( &reg_addr, 1 ) )
    return IMU_ERR_SPI;

  //Читаем содержимое регистра
  if( spiReceiv( reg_data, length ) )
  {
	  //Выставляем CS в 1
	  //SPI_CS_GPIO_Port->BSRR = (uint32_t)SPI_CS_Pin;
	  IMU_CS_GPIO_Port->BSRR = (uint32_t)IMU_CS_Pin;
	  //Возвращаем успех операции
    return IMU_ERR_OK;
  }
  //SPI_CS_GPIO_Port->BSRR = (uint32_t)SPI_CS_Pin;
  IMU_CS_GPIO_Port->BSRR = (uint32_t)IMU_CS_Pin;
  //Выставляем CS в 1


  return IMU_ERR_SPI;
  }


//!
//! \brief bhy2_i2c_write Записать данные в некоторый регистр
//! \param reg_addr       Адрес регистра
//! \param reg_data       Буфер с данными, которые нужно записать в регистр
//! \param length         Длина данных
//! \param intf_ptr       Указатель на интерфейс (не используем)
//! \return               Результат операции
//!
int8_t bhy2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
  {
  //Промежуточный буфер нужен чтобы передать данные вместе с адресом регистра
  static uint8_t buffer[65];

  if( length > 64 )
    return -4;

  //I2cDmaTransfer *i2cDmaTransfer = (I2cDmaTransfer*)intf_ptr;

  buffer[0] = reg_addr; //записываем в старший бит адреса 0 для записи данных
  //Скопировать данные из исходного буфера в промежуточный, чтобы адрес регистра был частью буфера данных
  memcpy( buffer + 1, reg_data, length );
  //memcpy( buffer + 1, reg_data, length );
  //Выставляем CS в 0
  //GPIOA->BSRR = (uint32_t)SPI_CS_Pin << (16U);
  GPIOE->BSRR = (uint32_t)IMU_CS_Pin << (16U);
  //if( spiTransmit( buffer + 1, length + 1 ) )
  if( spiTransmit( buffer, length + 1) )
  {
	  //Выставляем CS в 1
	  //SPI_CS_GPIO_Port->BSRR = (uint32_t)SPI_CS_Pin;
	  IMU_CS_GPIO_Port->BSRR = (uint32_t)IMU_CS_Pin;
	  return IMU_ERR_OK;
  }
  //Выставляем CS в 1
  //SPI_CS_GPIO_Port->BSRR = (uint32_t)SPI_CS_Pin;
  IMU_CS_GPIO_Port->BSRR = (uint32_t)IMU_CS_Pin;
  return IMU_ERR_I2C;
  }

//============================================================================================================
//  Функции взаимодействия с аппаратурой для обеспечения обмена по I2C
//Низкоуровневый драйвер

void i2c_init( I2C_HandleTypeDef *hi2c, uint8_t devAddr )
{
  mI2c = hi2c;
  mDevAddress = devAddr;
}

uint8_t summir(uint8_t a, uint8_t b)
{

return a+b;
}

//!
//! \brief i2cTransmit Передача с использованием канала DMA
//! \param data        Данные для передачи
//! \param size        Количество байт для передачи
//! \return            true - если операция выполнена успешно
//!
int8_t i2cTransmit( uint8_t *data, uint16_t size )
  {
  //Инициировать передачу по DMA
  HAL_I2C_Master_Transmit_DMA( mI2c, mDevAddress, data, size );
  //HAL_I2C_Master_Transmit(mI2c, mDevAddress, data, size, 10);
  while (HAL_I2C_GetState(mI2c) != HAL_I2C_STATE_READY);
  //Возвращает истину при успешном завершении операции
  return mI2c->ErrorCode == HAL_I2C_ERROR_NONE;
  }


//!
//! \brief i2cReceiv Прием с использованием канала DMA
//! \param data      Указатель на буфер, в который размещаются принятые данные
//! \param size      Количество принимаемых байтов
//! \return            true - если операция выполнена успешно
//!
int8_t i2cReceiv( uint8_t *data, uint16_t size )
  {
  //Инициировать передачу по DMA
  HAL_I2C_Master_Receive_DMA( mI2c, mDevAddress, data, size );
  //HAL_I2C_Master_Receive(mI2c, mDevAddress, data, size,  10);
  //Ожидать завершения приема
  while (HAL_I2C_GetState(mI2c) != HAL_I2C_STATE_READY);
  //Возвращает истину при успешном завершении операции
  return mI2c->ErrorCode == HAL_I2C_ERROR_NONE;
  }



//!
//! \brief bhy2_i2c_read Прочитать данные из некоторого регистра
//! \param reg_addr      Адрес регистра
//! \param reg_data      Буфер, в который нужно записать прочитанные из регистра данные
//! \param length        Длина данных
//! \param intf_ptr      Указатель на интерфейс (не используем)
//! \return              Результат операции
//!
int8_t bhy2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
  {
  //I2cDmaTransfer *i2cDmaTransfer = (I2cDmaTransfer*)intf_ptr;

  //Записываем адрес регистра
  if( !i2cTransmit( &reg_addr, 1 ) )
    return IMU_ERR_I2C;

  //Читаем содержимое регистра
  if( i2cReceiv( reg_data, length ) )
    //Возвращаем успех операции
    return IMU_ERR_OK;

  return IMU_ERR_I2C;
  }

//!
//! \brief bhy2_i2c_write Записать данные в некоторый регистр
//! \param reg_addr       Адрес регистра
//! \param reg_data       Буфер с данными, которые нужно записать в регистр
//! \param length         Длина данных
//! \param intf_ptr       Указатель на интерфейс (не используем)
//! \return               Результат операции
//!
int8_t bhy2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
  {
  //Промежуточный буфер нужен чтобы передать данные вместе с адресом регистра
  static uint8_t buffer[65];

  if( length > 64 )
    return -4;

  //I2cDmaTransfer *i2cDmaTransfer = (I2cDmaTransfer*)intf_ptr;

  buffer[0] = reg_addr;
  //Скопировать данные из исходного буфера в промежуточный, чтобы адрес регистра был частью буфера данных
  memcpy( buffer + 1, reg_data, length );
  if( i2cTransmit( buffer, length + 1 ) )
    return IMU_ERR_OK;

  return IMU_ERR_I2C;
  }



//!
//! \brief bhy2_delay_us Сформировать задержку на заданное количество мкс
//! \param us            Количество мкс, на которое нужно сформировать задержку
//! \param private_data  Неиспользуемый указатель на устройство
//!
void bhy2_delay_us(uint32_t us, void *private_data)
  {
  //Чтобы компилятор не ругался на неиспользуемую переменную
  (void)private_data;
  //Если задержка превышает 1мс, то задерживаем средствами ОС
  if( us >= 1000 )
    HAL_Delay( us / 1000 );
  else {
    //Задержка менее 1мс, делаем тупым счетом
    while( us-- ) {
      __NOP();
      }
    }
  }







