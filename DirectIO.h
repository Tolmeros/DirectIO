/*
  DirectIO.h - Main include file for Direct IO library
  Copyright (c) 2015 Michael Marchetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _DIRECTIO_H
#define _DIRECTIO_H 1

#include "include/ports.h"

#ifndef INPUT_PULLUP
// for boards that don't support pullups
#define INPUT_PULLUP INPUT
#endif

const u8 NO_PIN = 255;
#include "include/pin.h"
#include "include/analog.h"

#if !defined(DIRECTIO_FALLBACK)

template <u8 pin>
class Input {
    // An standard digital input. read() returns true if the signal is asserted (high).
    public:
        Input(boolean pullup=true) {
            pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
        }
        boolean read() {
            return _pins<pin>::input_read();
        }
        operator boolean() {
            return read();
        }
};

template <u8 pin>
class Output {
    // An digital output with direct port I/O
    public:
        Output(boolean initial_value=LOW) {
            pinMode(pin, OUTPUT);

            // include a call to digitalWrite here which will
            // turn off PWM on this pin, if needed
            digitalWrite(pin, initial_value);
        }
        void write(boolean value) {
            _pins<pin>::output_write(value);
        }
        Output& operator =(boolean value) {
            write(value);
            return *this;
        }
        void toggle() {
            write(! read());
        }
        void pulse(boolean value=HIGH) {
            write(value);
            write(! value);
        }
        boolean read() {
            return _pins<pin>::output_read();
        }
        operator boolean() {
            return read();
        }
};

template <u8 pin>
class OutputLow {
    // An digital output with direct port I/O
    public:
        OutputLow(boolean initial_value=HIGH) {
            pinMode(pin, OUTPUT);

            // include a call to digitalWrite here which will
            // turn off PWM on this pin, if needed
            digitalWrite(pin, initial_value);
        }
        void write(boolean value) {
            _pins<pin>::output_write(!value);
        }
        OutputLow& operator =(boolean value) {
            write(value);
            return *this;
        }
        void toggle() {
            write(! read());
        }
        void pulse(boolean value=LOW) {
            write(value);
            write(! value);
        }
        boolean read() {
            return !_pins<pin>::output_read();
        }
        operator boolean() {
            return read();
        }
};

template <class port, u8 start_bit=0, u8 nbits=8>
class InputPort {
    // A set of digital inputs which are contiguous and
    // located in a single MCU I/O port. This abandons
    // the pin number model for specifying I/O pins,
    // in order to gain fast, simultaneous
    // multi-bit reads and writes.
    public:
        InputPort() {
            setup();
        }

        void setup() {
            // set port pin directions to input
            port::port_enable_inputs(mask);
        }

        u8 read() {
            // mask to select bits of interest, then shift so
            // that output can be treated as normal integers.
            return (*port_t(port::in) & mask) >> start_bit;
        }
        operator u8() {
            return read();
        }

    private:
        static const u8 mask = ((u8(1) << nbits) - 1) << start_bit;
};

template <class port, u8 start_bit=0, u8 nbits=8>
class OutputPort {
    // A set of digital outputs which are contiguous and
    // located in a single MCU I/O port. This abandons
    // the pin number model for specifying I/O pins,
    // in order to gain fast, simultaneous
    // multi-bit reads and writes.
    public:
        OutputPort() {
            setup();
        }

        void setup() {
            // set port pin directions to output
            port::port_enable_outputs(mask);
        }

        void write(port_data_t value) {
            atomic {
                // read-modify-write cycle
                port_data_t v = port::port_output_read();
                port_data_t shifted = value << start_bit;
                v |= shifted & mask;
                v &= (shifted | ~mask);
                port::port_output_write(v);
            }
        }
        OutputPort& operator =(port_data_t value) {
            write(value);
            return *this;
        }
        port_data_t read() {
            // mask to select bits of interest, then shift so
            // that output can be treated as normal integers.
            return (port::port_output_read() & mask) >> start_bit;
        }
        operator port_data_t() {
            return read();
        }

    private:
        static const port_data_t mask = ((port_data_t(1) << nbits) - 1) << start_bit;
};

template <class port>
class OutputPort<port, 0, 8 * sizeof(port_data_t)> {
    // Specialization for a complete MCU output port.
    // This simplifies writes, which no longer require
    // a read/modify/write cycle. This reduces the
    // bit manipulation required, and also eliminates
    // the need to disable/reenable interrupts during writes.
    public:
        OutputPort() {
            setup();
        }

        void setup() {
            // set port pin directions to output
            port::port_enable_outputs(-1);
        }

        void write(u8 value) {
            port::port_output_write(value);
        }
        OutputPort& operator =(u8 value) {
            write(value);
            return *this;
        }
        u8 read() {
            return port::port_output_read();
        }
        operator u8() {
            return read();
        }
};

template <class port, u8 start_bit=0, u8 nbits=8>
class BiDirectionalPort {
    // A set of digital outputs which are contiguous and
    // located in a single MCU I/O port. This abandons
    // the pin number model for specifying I/O pins,
    // in order to gain fast, simultaneous
    // multi-bit reads and writes.
    public:
        BiDirectionalPort() {
            setup();
        }

        void setup() {
            // set port pin directions to input
            setInput();
        }

        void setInput() {
            port::port_enable_inputs(mask);
        }

        void setOutput() {
            port::port_enable_outputs(mask);
        }

        void write(port_data_t value) {
            // set port pin directions to output
            setOutput();

            atomic {
                // read-modify-write cycle
                port_data_t v = port::port_output_read();
                port_data_t shifted = value << start_bit;
                v |= shifted & mask;
                v &= (shifted | ~mask);
                port::port_output_write(v);
            };
        }

        BiDirectionalPort& operator =(port_data_t value) {
            write(value);
            return *this;
        }

        port_data_t read() {
            // mask to select bits of interest, then shift so
            // that output can be treated as normal integers.
            return (port::port_output_read() & mask) >> start_bit;
        }

        operator port_data_t() {
            return read();
        }

    private:
        static const port_data_t mask = ((port_data_t(1) << nbits) - 1) << start_bit;
};

template <class port_0, class port_1, u8 start_bit_0=0, u8 nbits_0=8, u8 start_bit_1=0, u8 nbits_1=8>
class BiDirectionalPortDouble {
    // A set of digital outputs which are contiguous and
    // located in a single MCU I/O port. This abandons
    // the pin number model for specifying I/O pins,
    // in order to gain fast, simultaneous
    // multi-bit reads and writes.
    public:
        BiDirectionalPortDouble() {
            setup();
        }

        void setup() {
            // set port pin directions to input
            setInput();
        }

        void setInput() {
            port_0::port_enable_inputs(mask_0);
            port_1::port_enable_inputs(mask_1);
        }

        void setOutput() {
            port_0::port_enable_outputs(mask_0);
            port_1::port_enable_outputs(mask_1);
        }

        void write(u16 value) {
            // set port pin directions to output
            setOutput();

            atomic {
                // read-modify-write cycle
                port_data_t v = port_0::port_output_read();
                port_data_t shifted = (value & mask_0) << start_bit_0;
                v |= shifted & mask_0;
                v &= (shifted | ~mask_0);
                port_0::port_output_write(v);

                v = port_1::port_output_read();
                shifted = ( (value >> nbits_0) & mask_1) << start_bit_1;
                v |= shifted & mask_1;
                v &= (shifted | ~mask_1);
                port_1::port_output_write(v);
            };

        }

        BiDirectionalPortDouble& operator =(u16 value) {
            write(value);
            return *this;
        }

        u16 read() {
            // mask to select bits of interest, then shift so
            // that output can be treated as normal integers.
            return (((port_1::port_output_read() & mask_1) >> start_bit_1) << nbits_0) | ((port_0::port_output_read() & mask_0) >> start_bit_0);
        }

        operator u16() {
            return read();
        }

    private:
        static const port_data_t mask_0 = ((port_data_t(1) << nbits_0) - 1) << start_bit_0;
        static const port_data_t mask_1 = ((port_data_t(1) << nbits_1) - 1) << start_bit_1;
};

template <class port_0, class port_1, u8 start_bit_0=0, u8 nbits_0=8, u8 start_bit_1=0, u8 nbits_1=8>
class BiDirectionalPortDouble8bit {
    // A set of digital outputs which are contiguous and
    // located in a single MCU I/O port. This abandons
    // the pin number model for specifying I/O pins,
    // in order to gain fast, simultaneous
    // multi-bit reads and writes.
    public:
        BiDirectionalPortDouble() {
            setup();
        }

        void setup() {
            // set port pin directions to input
            setInput();
        }

        void setInput() {
            port_0::port_enable_inputs(mask_0);
            port_1::port_enable_inputs(mask_1);
        }

        void setOutput() {
            port_0::port_enable_outputs(mask_0);
            port_1::port_enable_outputs(mask_1);
        }

        void write(port_data_t value) {
            // set port pin directions to output
            setOutput();

            atomic {
                // read-modify-write cycle
                port_data_t v = port_0::port_output_read();
                port_data_t shifted = (value & mask_0) << start_bit_0;
                v |= shifted & mask_0;
                v &= (shifted | ~mask_0);
                port_0::port_output_write(v);

                v = port_1::port_output_read();
                shifted = ( (value >> nbits_0) & mask_1) << start_bit_1;
                v |= shifted & mask_1;
                v &= (shifted | ~mask_1);
                port_1::port_output_write(v);
            };

        }

        BiDirectionalPortDouble8bit& operator =(port_data_t value) {
            write(value);
            return *this;
        }

        port_data_t read() {
            // mask to select bits of interest, then shift so
            // that output can be treated as normal integers.
            return (((port_1::port_output_read() & mask_1) >> start_bit_1) << nbits_0) | ((port_0::port_output_read() & mask_0) >> start_bit_0);
        }

        operator port_data_t() {
            return read();
        }

    private:
        static const port_data_t mask_0 = ((port_data_t(1) << nbits_0) - 1) << start_bit_0;
        static const port_data_t mask_1 = ((port_data_t(1) << nbits_1) - 1) << start_bit_1;
};

#else // DIRECTIO_FALLBACK

// These classes offer compatilbity with alternate Arduino-compatible devices, so
// that libraries can depend on DirectIO without limiting portability.
// These classes delegate to digitalRead and digitalWrite, so they trade off
// performance for portability.
template <u8 pin>
class Input {
    // An standard digital input. read() returns true if the signal is asserted (high).
    public:
        Input(boolean pullup=true) {
            pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
        }
        boolean read() {
            return digitalRead(pin);
        }
        operator boolean() {
            return read();
        }
};

template <u8 pin>
class Output {
    // A standard digital output 
    public:
        Output(boolean initial_value=LOW) {
            pinMode(pin, OUTPUT);
            digitalWrite(pin, initial_value);
        }
        void write(boolean value) {
            digitalWrite(pin, value);
        }
        Output& operator =(boolean value) {
            write(value);
            return *this;
        }
        void toggle() {
            write(! read());
        }
        void pulse(boolean value=HIGH) {
            write(value);
            write(! value);
        }
        boolean read() {
            return digitalRead(pin);
        }
        operator boolean() {
            return read();
        }
};

template <u8 pin>
class OutputLow {
    // An digital output with direct port I/O
    public:
        OutputLow(boolean initial_value=HIGH) {
            pinMode(pin, OUTPUT);
            digitalWrite(pin, initial_value);
        }
        void write(boolean value) {
            digitalWrite(pin, !value);
        }
        OutputLow& operator =(boolean value) {
            write(value);
            return *this;
        }
        void toggle() {
            write(! read());
        }
        void pulse(boolean value=LOW) {
            write(value);
            write(! value);
        }
        boolean read() {
            return !digitalRead(pin);
        }
        operator boolean() {
            return read();
        }
};

// TODO: fallbacks for InputPort and OutputPort

#endif // DIRECTIO_FALLBACK

template <u8 pin>
class InputLow {
    // An active low digital input. read() returns true if the signal is asserted (low).
    public:
        InputLow(boolean pullup=true) : input(pullup) {}
        boolean read() {
            return ! input.read();
        }
        operator boolean() {
            return read();
        }

    private:
        Input<pin> input;
};

// This macro lets you temporarily set an output to a value,
// and toggling back at the end of the code block. For example:
//
// Output<2> cs;
// Output<3> data;
// with(cs, LOW) {
//     data = HIGH;
// }
//
// is equivalent to:
// cs = LOW;
// data = HIGH;
// cs = HIGH;

#define with(pin, val) for(boolean _loop_##pin=((pin=val),true);_loop_##pin; _loop_##pin=((pin=!val), false))

template <>
class Output<NO_PIN> {
    // This specialization of Output is used when a module supports
    // an optional pin and that pin is not being used. For example,
    // if a chip supports an Output Enable pin but it is wired
    // permanently HIGH in the circuit, there won't be an actual
    // output pin connected to it. In the software, we will use this
    // type of Output which is basically a no-op.
    public:
        Output(boolean /*initial_value*/=LOW) {}
        void write(boolean /*value*/) {}
        Output& operator =(boolean /*value*/) {
            return *this;
        }
        void toggle() {}
        void pulse(boolean /*value*/=HIGH) {}

        boolean read() {
            return LOW;
        }
        operator boolean() {
            return read();
        }
};

#endif // _DIRECTIO_H
