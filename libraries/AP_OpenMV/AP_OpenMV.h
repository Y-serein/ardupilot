
#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>

class AP_OpenMV{
public:
    AP_OpenMV();

    /*Do not allow capies 定义拷贝构造函数（copy constructor）的声明，并通过 = delete 明确禁止该构造函数的使用。*/

/*
    AP_OpenMV(const AP_OpenMV &other) = delete;
    AP_OpenMV & operator = (const AP_OpenMV&) = delete;
*/
    CLASS_NO_COPY(AP_OpenMV);
    /*
    static AP_OpenMV *get_singleton(void){
        return singleton;
    }
    */
    void init(void);

    void updata(void);

    uint8_t cx;
    uint8_t cy;

    uint32_t last_fram_ms;
private:
    static AP_OpenMV *singleton;
    AP_HAL::UARTDriver *_port;

    uint8_t _step;
    uint8_t _cx_temp;
    uint8_t _cy_temp;

};
/*
namespace AP {
    AP_OpenMV *OpenMV();
};
*/