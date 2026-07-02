#ifndef stm32RoboClaw_h
#define stm32RoboClaw_h

#include <stdarg.h>
#include <stddef.h>
#include <inttypes.h>

#include "main.h"   // or your exact MCU family


class stm32RoboClaw  
{
  UART_HandleTypeDef* huart;
  uint16_t crc;
  uint32_t timeout;


  enum{
    M1DUTY=32,
    M2DUTY=33};

public:
  stm32RoboClaw(UART_HandleTypeDef* huart, uint32_t tout);
  virtual ~stm32RoboClaw();

  bool DutyM1(uint8_t address, int16_t duty);
	bool DutyM2(uint8_t address, int16_t duty);

  bool overflow();
	virtual int read();
	virtual size_t write(uint8_t byte);
	virtual void flush();
	void clear();

private:
  void crc_clear();
	void crc_update (uint8_t data);
	uint16_t crc_get();
	bool write_n(uint8_t cnt,...);
	bool read_n(uint8_t cnt,uint8_t address,uint8_t cmd,...);
};

#endif
