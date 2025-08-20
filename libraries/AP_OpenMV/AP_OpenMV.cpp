/************************************************************ 

Copyright (C), 1988 - 1999, Guangzhou zhier Creative Technology Co., Ltd. 

FileName: AP_OpenMV.cpp

Author:Serein_Y    Version:1.0.0    Date:2025.07.06

Description: Userd for communicating with the BMS and obtaining battery voltage and current data

Version: implement serial communication

Function List: 

    1.init(): Function initialization function

History: 

Serein_Y 25/05/28 0.1.0 build this module

***********************************************************/ 

#define AP_SERIALMANAGER_OPENMV_BUAD                 57600
#define AP_SERIALMANAGER_OPENMV_BUFSIZE_RX              64
#define AP_SERIALMANAGER_OPENMV_BUFSIXE_TX              64

#include "AP_OpenMV.h"

extern const AP_HAL::HAL& hal;

AP_OpenMV::AP_OpenMV(void)
{
   _port = NULL;
   _step = 0;
}

void AP_OpenMV::init()
{
   const AP_SerialManager &serial_manager = AP::serialmanager();

   _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_OPEN_MV, 0);

   if(_port){
      _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
      _port->begin(AP_SERIALMANAGER_OPENMV_BUAD, AP_SERIALMANAGER_OPENMV_BUFSIZE_RX, AP_SERIALMANAGER_OPENMV_BUFSIXE_TX);
   }
}

/************************************************* 

Function: 

Description: 

Calls: 

Called By: 

Table Accessed: 

Table Updated: 

Input: 

Output: 

Return: 

Others: 

*************************************************/
void AP_OpenMV::updata()
{
   /*接口是否存在*/
   if(_port == NULL)
      return;

   /*读取串口缓冲期存储了多少变量*/
   int16_t numc = _port->available();
   uint8_t data;
   uint8_t checksum = 0;

   for(int16_t i = 0; i < numc; i++){
      data = _port->read();

      switch(_step){
         case 0:
            if(data == 0xA5)
               _step = 1;
         break;
         case 1:
            if(data == 0x5A)
               _step = 2;
            else
               _step = 0;
         break;
         case 2:
            _cx_temp = data;
            _step = 3;
         break;
         case 3:
            _cy_temp = data;
            _step = 4;
         break;
         case 4:
            checksum = _cx_temp + _cy_temp;
            if(checksum == data){
               cx = _cx_temp;
               cy = _cy_temp;
               last_fram_ms = AP_HAL::millis();
            }

            _step = 0;
         break;
         default:
            _step = 0;
      }
   }
}
/*
namespace AP {
    AP_OpenMV *OpenMV() {
        return AP_OpenMV::get_singleton();
    }
};*/
