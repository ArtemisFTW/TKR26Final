#include "stm32roboclaw.h"

#include <stdarg.h>
#include <stddef.h>
#include <inttypes.h>

#define MAXRETRY 2
#define SetDWORDval(arg) (uint8_t)(((uint32_t)arg)>>24),(uint8_t)(((uint32_t)arg)>>16),(uint8_t)(((uint32_t)arg)>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(((uint16_t)arg)>>8),(uint8_t)arg

stm32RoboClaw::stm32RoboClaw(UART_HandleTypeDef* huart,uint32_t tout)
{
    timeout = tout;
    this->huart = huart;
}

stm32RoboClaw::~stm32RoboClaw()
{
}

bool stm32RoboClaw::DutyM1(uint8_t address, int16_t duty)
{
    return write_n(4,address,M1DUTY,SetWORDval(duty));
}

bool stm32RoboClaw::DutyM2(uint8_t address, int16_t duty)
{
    return write_n(4,address,M2DUTY,SetWORDval(duty));
}

void stm32RoboClaw::crc_clear()
{
    crc=0;
}

void stm32RoboClaw::crc_update (uint8_t data)
{
    int i;
    crc = crc ^ ((uint16_t)data << 8);
    for (i=0; i<8; i++)
    {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc <<= 1;
    }
}

uint16_t stm32RoboClaw::crc_get()
{
    return crc;
}

bool stm32RoboClaw::write_n(uint8_t cnt, ... )
{
    va_list ap;
    uint8_t i;
    crc_clear();

    va_start(ap, cnt);
    for(i=0;i<cnt;i++){
        uint8_t data = va_arg(ap,int);
        crc_update(data);
        HAL_UART_Transmit(huart,&data,1,timeout);
    }
    va_end(ap);

    uint16_t ccrc = crc_get();
    uint8_t data = (ccrc>>8) & 0xFF;
    HAL_UART_Transmit(huart,&data,1,timeout);
    data = ccrc & 0xFF;
    HAL_UART_Transmit(huart,&data,1,timeout);

    return true;
}

bool stm32RoboClaw::read_n(uint8_t cnt,uint8_t address,uint8_t cmd,...)
{
    uint32_t value=0;
    va_list ap;
    uint8_t i;
    int16_t data;

    flush();

    data=0;
    crc_clear();
    write(address);
    crc_update(address);
    write(cmd);
    crc_update(cmd);

    //send data with crc
    va_start(ap, cmd);
    for(i=0;i<cnt;i++){
        uint32_t *ptr = va_arg(ap, uint32_t *);

        if(data!=-1){
            data = read();
            crc_update(data);

            value=(uint32_t)data<<24;
        }
        else{
            break;
        }

        if(data!=-1){
            data = read();
            crc_update(data);
            value|=(uint32_t)data<<16;
        }
        else{
            break;
        }

        if(data!=-1){
            data = read();
            crc_update(data);
            value|=(uint32_t)data<<8;
        }
        else{
            break;
        }

        if(data!=-1){
            data = read();
            crc_update(data);
            value|=(uint32_t)data;
        }
        else{
            break;
        }

        *ptr = value;
    }
    va_end(ap);

    if(data!=-1){
        uint16_t ccrc;
        data = read();
        if(data!=-1){
            ccrc = data << 8;
            data = read();
            if(data!=-1){
                ccrc |= data;
                return crc_get()==ccrc;
            }
        }
    }
    return false;
}

size_t stm32RoboClaw::write(uint8_t cnt)
{
    HAL_UART_Transmit(huart,&cnt,1,timeout);
    return 1;
}

bool stm32RoboClaw::overflow()
{
    return (__HAL_UART_GET_FLAG(huart,UART_FLAG_ORE) != RESET);
}

int stm32RoboClaw::read()
{
    uint8_t byte;
    if(HAL_UART_Receive(huart,&byte,1,timeout) == HAL_OK){
        return byte;
    }
    return -1;
}

void stm32RoboClaw::flush()
{
    __HAL_UART_CLEAR_OREFLAG(huart);
}

void stm32RoboClaw::clear()
{
    __HAL_UART_CLEAR_OREFLAG(huart);
}

