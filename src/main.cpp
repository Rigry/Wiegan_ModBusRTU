#define STM32F030x6
#define F_CPU   48000000UL

#include "periph_rcc.h"
#include "init_clock.h"
#include "converter.h"


/// эта функция вызывается первой в startup файле
extern "C" void init_clock () { init_clock<8_MHz,F_CPU>(); } 

using D0   = mcu::PA6;
using D1   = mcu::PA7;
using LED  = mcu::PA0;
using BEEP = mcu::PA1;

using TX  = mcu::PA9;
using RX  = mcu::PA10;
using RTS = mcu::PA8;



int main()
{
   struct Flash_data {
      uint16_t factory_number = 0;
      UART::Settings uart_set = {
         .parity_enable  = false,
         .parity         = USART::Parity::even,
         .data_bits      = USART::DataBits::_8,
         .stop_bits      = USART::StopBits::_1,
         .baudrate       = USART::Baudrate::BR9600,
         .res            = 0
      };
      uint8_t  modbus_address = 1;
      uint16_t model_number   = 0;
   }flash;

   [[maybe_unused]] auto _ = Flash_updater<
        mcu::FLASH::Sector::_10
      , mcu::FLASH::Sector::_9
   >::make (&flash);


   volatile decltype(auto) modbus = Modbus_slave<In_regs, Out_regs>
                 ::make<mcu::Periph::USART1, TX, RX, RTS>
                       (flash.modbus_address, flash.uart_set);

   decltype(auto) wiegan = Wiegan::make<D0, D1>();

   decltype(auto) led  = Pin::make<LED, mcu::PinMode::Output>();
   decltype(auto) beep = Pin::make<BEEP, mcu::PinMode::Output>();

   using Flash  = decltype(flash);
   using Modbus = Modbus_slave<In_regs, Out_regs>;
   Converter<Flash, Modbus> converter {wiegan, led, beep, modbus, flash};

   modbus.outRegs.device_code       = 1;
   modbus.outRegs.factory_number    = flash.factory_number;
   modbus.outRegs.modbus_address    = flash.modbus_address;
   modbus.outRegs.uart_set          = flash.uart_set;
   modbus.arInRegsMax[ADR(uart_set)]= 0b11111111;
   modbus.inRegsMin.modbus_address  = 1;
   modbus.inRegsMax.modbus_address  = 255;


   while(1){
      converter();
      __WFI();
   }

}



