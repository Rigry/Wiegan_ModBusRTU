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

using FAC = mcu::PB4;



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
      uint16_t qty_request    = 0;
      uint16_t reset_time     = 0;
      uint16_t tone_sound     = 0;
      uint16_t version        = 1;
   }flash;

   [[maybe_unused]] auto _ = Flash_updater<
        mcu::FLASH::Sector::_10
      , mcu::FLASH::Sector::_9
   >::make (&flash);

   uint8_t modbus_address{1};
   UART::Settings uart_set = {
      .parity_enable  = false,
      .parity         = USART::Parity::even,
      .data_bits      = USART::DataBits::_8,
      .stop_bits      = USART::StopBits::_1,
      .baudrate       = USART::Baudrate::BR9600,
      .res            = 0
   };

   decltype(auto) factory = Pin::make<FAC,  mcu::PinMode::Input>();

   if (not factory) {
      modbus_address = flash.modbus_address;
      uart_set = flash.uart_set;
   }

   volatile decltype(auto) modbus = Modbus_slave<In_regs, Out_regs>
                 ::make<mcu::Periph::USART1, TX, RX, RTS>
                       (modbus_address, uart_set);

   decltype(auto) wiegan = Wiegan::make<D0, D1>();

   decltype(auto) led     = Pin::make<LED,  mcu::PinMode::Output>();
   decltype(auto) beep    = Pin::make<BEEP, mcu::PinMode::Output>();

   using Flash  = decltype(flash);
   using Modbus = Modbus_slave<In_regs, Out_regs>;
   Converter<Flash, Modbus> converter {wiegan, led, beep, modbus, flash};

   modbus.outRegs.qty_request    = flash.qty_request;
   modbus.outRegs.reset_time     = flash.reset_time;
   modbus.outRegs.tone_sound     = flash.tone_sound;
   modbus.outRegs.modbus_address = flash.modbus_address;
   modbus.outRegs.baudrate       = flash.uart_set.baudrate  == USART::Baudrate::BR115200 ? 6 :
                                   flash.uart_set.baudrate  == USART::Baudrate::BR57600  ? 5 :
                                   flash.uart_set.baudrate  == USART::Baudrate::BR38400  ? 4 :
                                   flash.uart_set.baudrate  == USART::Baudrate::BR28800  ? 3 :
                                   flash.uart_set.baudrate  == USART::Baudrate::BR19200  ? 2 :
                                   flash.uart_set.baudrate  == USART::Baudrate::BR14400  ? 1 :
                                   flash.uart_set.baudrate  == USART::Baudrate::BR9600   ? 0 : 255;
   modbus.outRegs.data_bits      = flash.uart_set.data_bits == USART::DataBits::_8 ? 0 :
                                   flash.uart_set.data_bits == USART::DataBits::_9 ? 1 : 255;
   modbus.outRegs.parity         = (not flash.uart_set.parity_enable) ? 0 :
                                   flash.uart_set.parity    == USART::Parity::even ? 1 :
                                   flash.uart_set.parity    == USART::Parity::odd  ? 2 : 255;
   modbus.outRegs.stop_bits      = flash.uart_set.stop_bits == USART::StopBits::_1 ? 0 :
                                   flash.uart_set.stop_bits == USART::StopBits::_2 ? 1 : 255;
   modbus.outRegs.version        = flash.version;

   modbus.inRegsMin.low_bits        = 0;
   modbus.inRegsMax.low_bits        = 0;
   modbus.inRegsMin.high_bits       = 0;
   modbus.inRegsMax.high_bits       = 0;
   modbus.inRegsMin.led             = 0;
   modbus.inRegsMax.led             = 1;
   modbus.inRegsMin.beep            = 0;
   modbus.inRegsMax.beep            = 1;
   modbus.inRegsMin.qty_request     = 0;
   modbus.inRegsMax.qty_request     = 10;
   modbus.inRegsMin.reset_time      = 0;
   modbus.inRegsMax.reset_time      = 10;
   modbus.inRegsMin.tone_sound      = 0;
   modbus.inRegsMax.tone_sound      = 2;
   modbus.inRegsMin.modbus_address  = 1;
   modbus.inRegsMax.modbus_address  = 255;
   modbus.inRegsMin.baudrate        = 0;
   modbus.inRegsMax.baudrate        = 6;
   modbus.inRegsMin.data_bits       = 0;
   modbus.inRegsMax.data_bits       = 1;
   modbus.inRegsMin.parity          = 0;
   modbus.inRegsMax.parity          = 2;
   modbus.inRegsMin.stop_bits       = 0;
   modbus.inRegsMax.stop_bits       = 1;


   while(1){
      converter();
      __WFI();
   }

}



