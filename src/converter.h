#pragma once

#include "pin.h"
#include "flash.h"
#include "modbus_slave.h"
#include "wiegan.h"
#include "timers.h"

struct In_regs {
   uint16_t low_bits;        // 0 (0...65535)
   uint16_t high_bits;       // 1 (0...65535)
   uint16_t led;             // 2 (0 - off, 1 - on)
   uint16_t beep;            // 3 (0 - off, 1 - on)
   uint16_t reserv_1;        // 4
   uint16_t qty_request;     // 5 (0...10)
   uint16_t reset_time;      // 6 (0...10 s)
   uint16_t tone_sound;      // 7 (0 - const, 1 - single_long, 2 - triple_short)
   uint16_t reserv_2;        // 8
   uint16_t reserv_3;        // 9
   uint16_t modbus_address;  // 10 (1...255)
   uint16_t baudrate;        // 11 (0 - 9600, 1 - 14400, 2 - 19200, 3 - 28800, 4 - 38400, 5 - 57600, 6 - 115200)
   uint16_t data_bits;       // 12 (0 - 7, 1 - 8)
   uint16_t parity;          // 13 (0 - none, 1 - even, 2 - odd)
   uint16_t stop_bits;       // 14 (0 - 1, 1 - 2)
}__attribute__((packed));

struct Out_regs {
   uint16_t low_bits;        // 0
   uint16_t high_bits;       // 1
   uint16_t led;             // 2
   uint16_t beep;            // 3
   uint16_t reserv_1;        // 4
   uint16_t qty_request;     // 5
   uint16_t reset_time;      // 6
   uint16_t tone_sound;      // 7
   uint16_t reserv_2;        // 8
   uint16_t reserv_3;        // 9
   uint16_t modbus_address;  // 10
   uint16_t baudrate;        // 11 (0 - 9600, 1 - 14400, 2 - 19200, 3 - 28800, 4 - 38400, 5 - 57600, 6 - 115200)
   uint16_t data_bits;       // 12 (0 - 8, 1 - 9)
   uint16_t parity;          // 13 (0 - none, 1 - even, 2 - odd)
   uint16_t stop_bits;       // 14 (0 - 1, 1 - 2)
   uint16_t reserv_4[5];     // 15...19
   uint16_t version;         // 20
}__attribute__((packed));

#define ADR(reg) GET_ADR(In_regs, reg)

template<class Flash_data, class Modbus>
class Converter : TickSubscriber
{
    Wiegan& wiegan;
    Pin& led;
    Pin& beep;
    Modbus& modbus;
    Flash_data& flash;
    Timer beep_long {500_ms};
    Timer beep_short{100_ms};
    Timer pause{};
    uint8_t cnt     {0};

    int time {0}; // время для сброса

    void reset_time (uint8_t t)
    {
        if (t == time) {
            wiegan.reset_number();
            wiegan.enable();
            time = 0;
            tick_unsubscribe();
        } 
    }

    void switch_led() {
        if (modbus.inRegs.reset_time != 0 or modbus.inRegs.qty_request != 0)
            led = static_cast<bool>(modbus.outRegs.high_bits & modbus.outRegs.low_bits);
    }

    void switch_beep() {
        if (modbus.inRegs.tone_sound == 0) {
            beep = static_cast<bool>(modbus.inRegs.beep);
        } else if (modbus.inRegs.tone_sound == 1 and modbus.inRegs.beep) {
            beep ^= beep_long.event();
        } else if (modbus.inRegs.tone_sound == 2 and modbus.inRegs.beep) {
            if (beep_short.event() and cnt != 6) {
                cnt++;
                beep ^= 1;
            } 
            if (cnt == 6) {
                pause.start(500_ms);
                beep = false;
                if (pause.done()) {
                    pause.stop();
                    cnt = 0;
                }
            }
        }

        beep = static_cast<bool>(modbus.inRegs.beep) ? beep : false;
    }

    void notify() override
    {
        time++;
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

      modbus.outRegs.high_bits = wiegan.get_high_bits();
      modbus.outRegs.low_bits  = wiegan.get_low_bits();

        modbus([&](auto registr){
            switch (registr) {
                case ADR(low_bits):
                    if (modbus.inRegs.qty_request == 0 and modbus.inRegs.reset_time == 0) {
                        modbus.outRegs.low_bits
                            = modbus.inRegs.low_bits;
                        wiegan.reset_number();
                    }
                break;
                case ADR(high_bits):
                    if (modbus.inRegs.qty_request == 0 and modbus.inRegs.reset_time == 0) {
                        modbus.outRegs.high_bits
                            = modbus.inRegs.high_bits;
                        wiegan.reset_number();
                    }
                break;
                case ADR(led):
                    if (modbus.inRegs.qty_request == 0 and modbus.inRegs.reset_time == 0) { 
                        modbus.outRegs.led
                            = modbus.inRegs.led;
                        led = static_cast<bool>(modbus.inRegs.led);
                    }
                break;
                case ADR(beep):
                    modbus.outRegs.beep
                        = modbus.inRegs.beep;
                break;
                case ADR(qty_request):
                    if (flash.reset_time == 0) {
                        flash.qty_request
                            = modbus.outRegs.qty_request
                            = modbus.inRegs.qty_request;
                    }
                break;
                case ADR(reset_time):
                    if (flash.qty_request == 0) {
                        flash.reset_time
                            = modbus.outRegs.reset_time
                            = modbus.inRegs.reset_time;
                    }
                break;
                case ADR(tone_sound):
                    flash.tone_sound
                        = modbus.outRegs.tone_sound
                        = modbus.inRegs.tone_sound;
                break;
                case ADR(modbus_address):
                   flash.modbus_address 
                      = modbus.outRegs.modbus_address
                      = modbus.inRegs.modbus_address;
                break;
                case ADR(baudrate):
                    modbus.outRegs.baudrate
                      = modbus.inRegs.baudrate;
                    if (modbus.inRegs.baudrate == 0)
                        flash.uart_set.baudrate = USART::Baudrate::BR9600;
                    else if (modbus.inRegs.baudrate == 1)
                        flash.uart_set.baudrate = USART::Baudrate::BR14400;
                    else if (modbus.inRegs.baudrate == 2)
                        flash.uart_set.baudrate = USART::Baudrate::BR19200;
                    else if (modbus.inRegs.baudrate == 3)
                        flash.uart_set.baudrate = USART::Baudrate::BR28800;
                    else if (modbus.inRegs.baudrate == 4)
                        flash.uart_set.baudrate = USART::Baudrate::BR38400;
                    else if (modbus.inRegs.baudrate == 5)
                        flash.uart_set.baudrate = USART::Baudrate::BR57600;
                    else if (modbus.inRegs.baudrate == 6)
                        flash.uart_set.baudrate = USART::Baudrate::BR115200;
                break;
                case ADR(data_bits):
                    modbus.outRegs.data_bits
                        = modbus.inRegs.data_bits;
                    if (modbus.inRegs.data_bits == 0)
                        flash.uart_set.data_bits = USART::DataBits::_8;
                    else if (modbus.inRegs.data_bits == 1)
                        flash.uart_set.data_bits = USART::DataBits::_9;
                break;
                case ADR(parity):
                    modbus.outRegs.parity
                        = modbus.inRegs.parity;
                    if (modbus.inRegs.parity == 0) {
                        flash.uart_set.parity_enable = false;
                        flash.uart_set.parity = USART::Parity::even;
                    } else if (modbus.inRegs.parity == 1) {
                        flash.uart_set.parity_enable = true;
                        flash.uart_set.parity = USART::Parity::even;
                    } else if (modbus.inRegs.parity == 1) {
                        flash.uart_set.parity_enable = true;
                        flash.uart_set.parity = USART::Parity::odd;
                    }
                break;
                case ADR (stop_bits):
                    modbus.outRegs.stop_bits
                        = modbus.inRegs.stop_bits;
                    if (modbus.inRegs.stop_bits == 0)
                        flash.uart_set.stop_bits = USART::StopBits::_1;
                    else if (modbus.inRegs.stop_bits == 1)
                        flash.uart_set.stop_bits = USART::StopBits::_2;
                break;
            } // switch
        }); // modbus([&](auto registr)
        switch_beep();
        switch_led();
        if (modbus.inRegs.reset_time > 0 and modbus.inRegs.qty_request == 0) {
            if (wiegan.new_card()) {
                wiegan.enable(false);
                tick_subscribe();
            }
            reset_time(modbus.inRegs.reset_time * 1000);
        }
    } //void operator() ()
};