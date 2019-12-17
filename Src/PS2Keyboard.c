/*
 * PS2Keyboard.c
 *
 *  Created on: 10 de nov de 2018
 *      Author: Ailson
 */


#include "PS2Keyboard.h"

const PROGMEM PS2Keymap_t PS2Keymap_US = {
  // without shift
	{0, PS2_F9, 0, PS2_F5, PS2_F3, PS2_F1, PS2_F2, PS2_F12,
	0, PS2_F10, PS2_F8, PS2_F6, PS2_F4, PS2_TAB, '`', 0,
	0, 0 /*Lalt*/, 0 /*Lshift*/, 0, 0 /*Lctrl*/, 'q', '1', 0,
	0, 0, 'z', 's', 'a', 'w', '2', 0,
	0, 'c', 'x', 'd', 'e', '4', '3', 0,
	0, ' ', 'v', 'f', 't', 'r', '5', 0,
	0, 'n', 'b', 'h', 'g', 'y', '6', 0,
	0, 0, 'm', 'j', 'u', '7', '8', 0,
	0, ',', 'k', 'i', 'o', '0', '9', 0,
	0, '.', '/', 'l', ';', 'p', '-', 0,
	0, 0, '\'', 0, '[', '=', 0, 0,
	0 /*CapsLock*/, 0 /*Rshift*/, PS2_ENTER /*Enter*/, ']', 0, '\\', 0, 0,
	0, 0, 0, 0, 0, 0, PS2_BACKSPACE, 0,
	0, '1', 0, '4', '7', 0, 0, 0,
	'0', '.', '2', '5', '6', '8', PS2_ESC, 0 /*NumLock*/,
	PS2_F11, '+', '3', '-', '*', '9', PS2_SCROLL, 0,
	0, 0, 0, PS2_F7 },
  // with shift
	{0, PS2_F9, 0, PS2_F5, PS2_F3, PS2_F1, PS2_F2, PS2_F12,
	0, PS2_F10, PS2_F8, PS2_F6, PS2_F4, PS2_TAB, '~', 0,
	0, 0 /*Lalt*/, 0 /*Lshift*/, 0, 0 /*Lctrl*/, 'Q', '!', 0,
	0, 0, 'Z', 'S', 'A', 'W', '@', 0,
	0, 'C', 'X', 'D', 'E', '$', '#', 0,
	0, ' ', 'V', 'F', 'T', 'R', '%', 0,
	0, 'N', 'B', 'H', 'G', 'Y', '^', 0,
	0, 0, 'M', 'J', 'U', '&', '*', 0,
	0, '<', 'K', 'I', 'O', ')', '(', 0,
	0, '>', '?', 'L', ':', 'P', '_', 0,
	0, 0, '"', 0, '{', '+', 0, 0,
	0 /*CapsLock*/, 0 /*Rshift*/, PS2_ENTER /*Enter*/, '}', 0, '|', 0, 0,
	0, 0, 0, 0, 0, 0, PS2_BACKSPACE, 0,
	0, '1', 0, '4', '7', 0, 0, 0,
	'0', '.', '2', '5', '6', '8', PS2_ESC, 0 /*NumLock*/,
	PS2_F11, '+', '3', '-', '*', '9', PS2_SCROLL, 0,
	0, 0, 0, PS2_F7 },
	0
};

#define BREAK     0x01
#define MODIFIER  0x02
#define SHIFT_L   0x04
#define SHIFT_R   0x08
#define ALTGR     0x10

// The ISR for the external interrupt
void ps2interrupt(Keyboard_TypeDef* keyboard)
{
	static uint8_t bitcount=0;
	static uint8_t incoming=0;
	static uint32_t prev_ms=0;
	uint32_t now_ms;
	int8_t n, val;

	val = HAL_GPIO_ReadPin(keyboard->DataPort, keyboard->DataPin);
	now_ms = HAL_GetTick() ;
	if (now_ms - prev_ms > 250) {
		bitcount = 0;
		incoming = 0;
	}
	prev_ms = now_ms;
	n = bitcount - 1;
	if (n <= 7) {
		incoming |= (val << n);
	}
	bitcount++;
	if (bitcount == 11) {
		uint8_t i = keyboard->head + 1;
		if (i >= BUFFER_SIZE) i = 0;
		if (i != keyboard->tail) {
			keyboard->buffer[i] = incoming;
			keyboard->head = i;
		}
		bitcount = 0;
		incoming = 0;
	}
}

static inline uint8_t get_scan_code(Keyboard_TypeDef* keyboard)
{
	uint8_t c, i;

	i = keyboard->tail;
	if (i == keyboard->head) return 0;
	i++;
	if (i >= BUFFER_SIZE) i = 0;
	c = keyboard->buffer[i];
	keyboard->tail = i;
	return c;
}

static char get_iso8859_code(Keyboard_TypeDef* keyboard)
{
	static uint8_t state=0;
	uint8_t s;
	char c;

	while (1) {
		s = get_scan_code(keyboard);
		if (!s) return 0;
		if (s == 0xF0) {
			state |= BREAK;
		} else if (s == 0xE0) {
			state |= MODIFIER;
		} else {
			if (state & BREAK) {
				if (s == 0x12) {
					state &= ~SHIFT_L;
				} else if (s == 0x59) {
					state &= ~SHIFT_R;
				} else if (s == 0x11 && (state & MODIFIER)) {
					state &= ~ALTGR;
				}
				// CTRL, ALT & WIN keys could be added
				// but is that really worth the overkeyboard->head?
				state &= ~(BREAK | MODIFIER);
				continue;
			}
			if (s == 0x12) {
				state |= SHIFT_L;
				continue;
			} else if (s == 0x59) {
				state |= SHIFT_R;
				continue;
			} else if (s == 0x11 && (state & MODIFIER)) {
				state |= ALTGR;
			}
			c = 0;
			if (state & MODIFIER) {
				switch (s) {
				  case 0x70: c = PS2_INSERT;      break;
				  case 0x6C: c = PS2_HOME;        break;
				  case 0x7D: c = PS2_PAGEUP;      break;
				  case 0x71: c = PS2_DELETE;      break;
				  case 0x69: c = PS2_END;         break;
				  case 0x7A: c = PS2_PAGEDOWN;    break;
				  case 0x75: c = PS2_UPARROW;     break;
				  case 0x6B: c = PS2_LEFTARROW;   break;
				  case 0x72: c = PS2_DOWNARROW;   break;
				  case 0x74: c = PS2_RIGHTARROW;  break;
				  case 0x4A: c = '/';             break;
				  case 0x5A: c = PS2_ENTER;       break;
				  default: break;
				}
			} else if ((state & ALTGR) && keyboard->keymap->uses_altgr) {
				if (s < PS2_KEYMAP_SIZE)
					c = pgm_read_byte(keyboard->keymap->altgr + s); // MUDAR
			} else if (state & (SHIFT_L | SHIFT_R)) {
//				if (s < PS2_KEYMAP_SIZE)
				//	c = pgm_read_byte(keyboard->keymap->shift + s);
			} else {
				if (s < PS2_KEYMAP_SIZE)
					c = pgm_read_byte(keyboard->keymap->noshift + s);
			}
			state &= ~(BREAK | MODIFIER);
			if (c) return c;
		}
	}
}

uint8_t keyboardAvailable(Keyboard_TypeDef* keyboard) {
	if (keyboard->CharBuffer || keyboard->UTF8next) return 1;
	keyboard->CharBuffer = get_iso8859_code(keyboard);
	if (keyboard->CharBuffer) return 1;
	return 0;
}

void keyboardClear(Keyboard_TypeDef* keyboard) {
	keyboard->CharBuffer = 0;
	keyboard->UTF8next = 0;
}

uint8_t keyboardReadScanCode(Keyboard_TypeDef* keyboard) 
{
	return get_scan_code(keyboard);
}

uint8_t keyboardRead(Keyboard_TypeDef* keyboard) { 
	uint8_t result;

	result = keyboard->UTF8next;
	if (result) {
		keyboard->UTF8next = 0;
	} else {
		result = keyboard->CharBuffer;
		if (result) {
			keyboard->CharBuffer = 0;
		} else {
			result = get_iso8859_code(keyboard);
		}
		if (result >= 128) {
			keyboard->UTF8next = (result & 0x3F) | 0x80;
			result = ((result >> 6) & 0x1F) | 0xC0;
		}
	}
	if (!result) return -1;
	return result;
}

int keyboardReadUnicode(Keyboard_TypeDef* keyboard) { 
	int result;

	result = keyboard->CharBuffer;
	if (!result) result = get_iso8859_code(keyboard);
	if (!result) return -1;
	keyboard->UTF8next = 0;
	keyboard->CharBuffer = 0;
	return result;
}


/* PIN CONFIGURATION: both DataPin and IQRPin must be iniciatized with PULLUP configuration
*/

void keyboardBegin(Keyboard_TypeDef* keyboard, GPIO_TypeDef* data_port, uint16_t data_pin, GPIO_TypeDef* iqr_port, uint16_t iqr_pin) {

  keyboard->DataPort = data_port;
  keyboard->DataPin = data_pin;

  keyboard->IQRPort = iqr_port;
  keyboard->IQRPin = iqr_pin;

  keyboard->keymap = &PS2Keymap_US;

  keyboard->head = 0;
  keyboard->tail = 0;
  keyboard->CharBuffer = 0;
  keyboard->UTF8next = 0;

}
