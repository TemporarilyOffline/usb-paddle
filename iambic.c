/** \file
 * Iambic USB keyer.
 *
 * Read an iambic keyer on port D and generate key press events
 * based on the Morse code input.
 *
 * (c) 2012 Trammell Hudson <hudson@osresearch.net>
 *
 * Based on the
 * Keyboard example with debug channel, for Teensy USB Development Board
 * http://www.pjrc.com/teensy/usb_keyboard.html
 * Copyright (c) 2008 PJRC.COM, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usb_keyboard_debug.h"
#include "print.h"

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

#define LED_PIN (1 << 6)

static inline void
led_on(void)
{
	PORTD |= LED_PIN;
	DDRB |=  1 << 7; // OC0A enabled
}

static inline void
led_off(void)
{
	PORTD &= ~LED_PIN;
	DDRB &=  ~(1 << 7); // OC0A disabled
}

static inline void
led_config(void)
{
	DDRD |= LED_PIN;
}

uint8_t number_keys[10]=
	{KEY_0,KEY_1,KEY_2,KEY_3,KEY_4,KEY_5,KEY_6,KEY_7,KEY_8,KEY_9};

uint16_t idle_count=0;


static inline int
is_dah(
	const uint8_t value
)
{
	return bit_is_clear(value, 4);
}


static inline int
is_dit(
	const uint8_t value
)
{
	return bit_is_clear(value, 5);
}

static uint16_t dah_count;
static uint16_t dit_count;


static void
busy_wait(
	uint16_t delay_ticks
)
{
	OCR1A = TCNT1 + delay_ticks;
	TIFR1 |= (1 << OCF1A);

	while (bit_is_clear(TIFR1, OCF1A))
	{
		const uint8_t port = PINB;
		if (is_dah(port))
			dah_count++;
		if (is_dit(port))
			dit_count++;
	}
}


static void
wait(
	uint16_t on_ticks,
	uint16_t off_ticks
)
{
	dit_count = dah_count = 0;

	led_on();
	busy_wait(on_ticks);

	led_off();
	busy_wait(off_ticks);
}


static void
bad_input(void)
{
	uint8_t tccr0b = TCCR0B;
	TCCR0B = 0x04; // Clk/256
	led_on();
	print("!!!\n");
	_delay_ms(40);
	led_off();

	// Restore the old timer
	TCCR0B = tccr0b;
}


int main(void)
{
	uint8_t b, d, mask, i, reset_idle;
	uint8_t b_prev=0xFF, d_prev=0xFF;

	// set for 16 MHz clock
	CPU_PRESCALE(0);
	led_config();

	// Configure all port B and port D pins as inputs with pullup resistors.
	// See the "Using I/O Pins" page for details.
	// http://www.pjrc.com/teensy/pins.html
	DDRD = 0x00;
	DDRB = 0x00;
	PORTB = 0xFF;
	PORTD = 0xFF;

	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();
	while (!usb_configured())
	{
		/* busy wait */
	}

	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

/*
* http://en.wikipedia.org/wiki/Morse_code#Representation.2C_timing_and_speeds
*
* International Morse code is composed of five elements:
*
* short mark, dot or 'dit' (·) — 'dot duration' is one unit long
* longer mark, dash or 'dah' (–) — three units long
* inter-element gap between the dots and dashes within a character
*   - one dot duration or one unit long
* short gap (between letters) — three units long
* medium gap (between words) — seven units long[19]
*
* Based upon a 50 dot duration standard word such as PARIS, the time
* for one dot duration or one unit can be computed by the formula:
* T = 1200 / W
* or
* T = 6000 / C
* Where: T is the unit time, or dot duration, in milliseconds,
* W is the speed in wpm, and C is the speed in cpm.
*/

#include "morse.h"

	// 
	// 256*1024 clock cycles, or approx 61 Hz when using 16 MHz clock
	// This demonstrates how to use interrupts to implement a simple
	// inactivity timeout.
	TCCR0A = 0x00;
	TCCR0B = 0x05;
	//TIMSK0 = (1<<TOIE0);

	// TCNT1 runs at CLK/1024, which is 0.064 ms per tick
	// This is pretty close to 16 ticks/ms.  The output compare
	// sets OCF flag on match
	TCCR1B = 5; // clk/1024
        // const uint8_t cpm = 15;
	const uint8_t cpm = 25;
	const uint16_t dit_time = (1600 / cpm) * 16;
	const uint16_t button_threshold = 16384; // dit_time * 1024 * 4;

	// TCNT0 is configured for a 500 Hz tone on OC0A
	// At CLK/64, this is turns on and off each time
	// a full counter is made, which creates a 488 Hz
	// square wave.
	TCCR0A = (1 << COM0A0) | (1 << COM0A1) | 0x03;
	TCCR0B = 0x03; // CLK/64
	OCR0A = 0x80; // low volume

	// Set mode to fast, inverted PWM
	
	led_off();

	uint8_t value = 1;
	uint8_t bits = 0;
	uint8_t last_bit = 0;

	while (1)
	{
		// Read PINB, which has both input pins
		// since there is a pull-up, the pins will be pulled
		// to ground when the switch is hit.

		if (bits > 7)
		{
			// Too many symbols.  Ignore it
			bad_input();
			goto reset;
		} else
		if (is_dit(PINB))
		{
			// The dit key is held down and
			// If the last bit sent was also a dit and the
			// dah switch is enabled, send a dah instead.
start_dit_bit:
			bits++;
			value <<= 1;
			print(".");

			wait(dit_time, dit_time);
			if (dah_count > button_threshold)
				goto start_dah_bit;

			continue;
		} else
		if (is_dah(PINB))
		{
start_dah_bit:
			bits++;
			value = (value << 1) | 1;
			print("-");

			wait(3*dit_time, dit_time);
			if (dit_count > button_threshold)
				goto start_dit_bit;

			continue;
		} else
		if (bits == 0)
		{

#if 0
			if (last_send_time && now() > last_send_time)
				usb_keyboard_press(KEY_SPACE, 0);
			last_send_time = 0;
#endif
			continue;
		}

		// Neither key is held down and the single space time
		// has elapsed (since wait_delay is blocking).

		// Delay one more cycle to be sure, abort the delay
		// as soon as a button is pressed
		OCR1A = TCNT1 + dit_time * 2;
		TIFR1 |= (1 << OCF1A);

		while (bit_is_clear(TIFR1, OCF1A))
		{
			const uint8_t port = PINB;
			if (is_dah(port))
				goto start_dah_bit;
			if (is_dit(port))
				goto start_dit_bit;
		}

		// Timeout has passed; check to see if the
		// value exist in the map
		uint8_t c = pgm_read_byte(&morse[value]);

		//last_send_time = now();
		print(" = ");
		phex(value);

		if (!c)
		{
			bad_input();
			goto reset;
		}

		pchar('\n');
		
		const uint8_t modbit = c & 0x80 ? KEY_SHIFT : 0;
		c &= ~0x80;
		usb_keyboard_press(c, modbit);

reset:
		bits = 0;
		value = 1;
	}
}
