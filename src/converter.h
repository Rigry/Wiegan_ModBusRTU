#pragma once

#include "pin.h"
#include "flash.h"
#include "modbus_slave.h"
#include "wiegan.h"

struct Control
{
   bool enable  :1;
   bool disable :1;
   uint16_t     :14;
}__attribute__((packed));


struct In_regs {
   
   UART::Settings uart_set;         // 0
   uint16_t modbus_address;         // 1
   uint16_t password;               // 2
   uint16_t factory_number;         // 3
   Control control;                 // 4
}__attribute__((packed));

   struct Out_regs {

   uint16_t device_code;            // 0
   uint16_t factory_number;         // 1
   UART::Settings uart_set;         // 2
   uint16_t modbus_address;         // 3
   uint16_t high_value;             // 4
   uint16_t low_value;              // 5
   Control control;                 // 6

}__attribute__((packed));

#define ADR(reg) GET_ADR(In_regs, reg)

template<class Flash_data, class Modbus>
class Converter
{
    Wiegan& wiegan;
    Pin& led;
    Pin& beep;
    Modbus& modbus;
    Flash_data& flash;

    void enable(){
        wiegan.reset_number();
    }
    void disable(){
        wiegan.reset_number();
    }

public:
    Converter (Wiegan& wiegan, Pin& led, Pin& beep, Modbus& modbus, Flash_data& flash)
        : wiegan{wiegan}
        , led   {led}
        , beep  {beep}
        , modbus{modbus}
        , flash {flash}
    {}

    void operator() () {
        
      //   wiegan.get_number();
        modbus.outRegs.high_value = wiegan.get_high_bits();
        modbus.outRegs.low_value  = wiegan.get_low_bits();

        modbus([&](auto registr){
            static bool unblock = false;
            switch (registr) {
                case ADR(uart_set):
                   flash.uart_set
                      = modbus.outRegs.uart_set
                      = modbus.inRegs.uart_set;
                break;
                case ADR(modbus_address):
                   flash.modbus_address 
                      = modbus.outRegs.modbus_address
                      = modbus.inRegs.modbus_address;
                break;
                case ADR(password):
                   unblock = modbus.inRegs.password == 1207;
                break;
                case ADR(factory_number):
                   if (unblock) {
                      unblock = false;
                      flash.factory_number 
                         = modbus.outRegs.factory_number
                         = modbus.inRegs.factory_number;
                   }
                   unblock = true;
                break;
                case ADR(control):
                   if (modbus.inRegs.control.enable) {
                      enable();
                      led = modbus.outRegs.control.enable = modbus.inRegs.control.enable;
                   } else if (modbus.inRegs.control.disable) {
                      disable();
                      beep = modbus.outRegs.control.disable = modbus.inRegs.control.disable;
                   }
                break;
            } // switch
        }); // modbus([&](auto registr)
    } //void operator() ()
};