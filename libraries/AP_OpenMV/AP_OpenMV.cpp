/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* 
   Graupner Hott Telemetry library
   Hott telemetry runs at 19200 8N1 on a non-inverted half-duplex UART

   With thanks to Graupner and betaflight
*/

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

namespace AP {
    AP_OpenMV *OpenMV() {
        return AP_OpenMV::get_singleton();
    }
};