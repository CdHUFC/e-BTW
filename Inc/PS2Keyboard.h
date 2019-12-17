/*
 * PS2Keyboard.h
 *
 *  Created on: 10 de nov de 2018
 *      Author: fox
 */

#ifndef PS2KEYBOARD_H_
#define PS2KEYBOARD_H_

#include "stm32f1xx_it.h"
#include "gpio.h"
#include "pgmspace.h"
// Every call to read() returns a single byte for each
// keystroke.  These configure what byte will be returned
// for each "special" key.  To ignore a key, use zero.
#define PS2_TAB				9
#define PS2_ENTER			13
#define PS2_BACKSPACE			127
#define PS2_ESC				27
#define PS2_INSERT			0
#define PS2_DELETE			127
#define PS2_HOME			0
#define PS2_END				0
#define PS2_PAGEUP			25
#define PS2_PAGEDOWN			26
#define PS2_UPARROW			11
#define PS2_LEFTARROW			8
#define PS2_DOWNARROW			10
#define PS2_RIGHTARROW			21
#define PS2_F1				0
#define PS2_F2				0
#define PS2_F3				0
#define PS2_F4				0
#define PS2_F5				0
#define PS2_F6				0
#define PS2_F7				0
#define PS2_F8				0
#define PS2_F9				0
#define PS2_F10				0
#define PS2_F11				0
#define PS2_F12				0
#define PS2_SCROLL			0
#define PS2_EURO_SIGN			0

#define PS2_INVERTED_EXCLAMATION	161 // Â¡
#define PS2_CENT_SIGN			162 // Â¢
#define PS2_POUND_SIGN			163 // Â£
#define PS2_CURRENCY_SIGN		164 // Â¤
#define PS2_YEN_SIGN			165 // Â¥
#define PS2_BROKEN_BAR                 	166 // Â¦
#define PS2_SECTION_SIGN		167 // Â§
#define PS2_DIAERESIS			168 // Â¨
#define PS2_COPYRIGHT_SIGN		169 // Â©
#define PS2_FEMININE_ORDINAL		170 // Âª
#define PS2_LEFT_DOUBLE_ANGLE_QUOTE	171 // Â«
#define PS2_NOT_SIGN			172 // Â¬
#define PS2_HYPHEN			173
#define PS2_REGISTERED_SIGN		174 // Â®
#define PS2_MACRON			175 // Â¯
#define PS2_DEGREE_SIGN			176 // Â°
#define PS2_PLUS_MINUS_SIGN		177 // Â±
#define PS2_SUPERSCRIPT_TWO		178 // Â²
#define PS2_SUPERSCRIPT_THREE		179 // Â³
#define PS2_ACUTE_ACCENT		180 // Â´
#define PS2_MICRO_SIGN			181 // Âµ
#define PS2_PILCROW_SIGN		182 // Â¶
#define PS2_MIDDLE_DOT			183 // Â·
#define PS2_CEDILLA			184 // Â¸
#define PS2_SUPERSCRIPT_ONE		185 // Â¹
#define PS2_MASCULINE_ORDINAL		186 // Âº
#define PS2_RIGHT_DOUBLE_ANGLE_QUOTE	187 // Â»
#define PS2_FRACTION_ONE_QUARTER	188 // Â¼
#define PS2_FRACTION_ONE_HALF		189 // Â½
#define PS2_FRACTION_THREE_QUARTERS	190 // Â¾
#define PS2_INVERTED_QUESTION_MARK	191 // Â¿
#define PS2_A_GRAVE			192 // Ã€
#define PS2_A_ACUTE			193 // Ã�
#define PS2_A_CIRCUMFLEX		194 // Ã‚
#define PS2_A_TILDE			195 // Ãƒ
#define PS2_A_DIAERESIS			196 // Ã„
#define PS2_A_RING_ABOVE		197 // Ã…
#define PS2_AE				198 // Ã†
#define PS2_C_CEDILLA			199 // Ã‡
#define PS2_E_GRAVE			200 // Ãˆ
#define PS2_E_ACUTE			201 // Ã‰
#define PS2_E_CIRCUMFLEX		202 // ÃŠ
#define PS2_E_DIAERESIS			203 // Ã‹
#define PS2_I_GRAVE			204 // ÃŒ
#define PS2_I_ACUTE			205 // Ã�
#define PS2_I_CIRCUMFLEX		206 // ÃŽ
#define PS2_I_DIAERESIS			207 // Ã�
#define PS2_ETH				208 // Ã�
#define PS2_N_TILDE			209 // Ã‘
#define PS2_O_GRAVE			210 // Ã’
#define PS2_O_ACUTE			211 // Ã“
#define PS2_O_CIRCUMFLEX		212 // Ã”
#define PS2_O_TILDE			213 // Ã•
#define PS2_O_DIAERESIS			214 // Ã–
#define PS2_MULTIPLICATION		215 // Ã—
#define PS2_O_STROKE			216 // Ã˜
#define PS2_U_GRAVE			217 // Ã™
#define PS2_U_ACUTE			218 // Ãš
#define PS2_U_CIRCUMFLEX		219 // Ã›
#define PS2_U_DIAERESIS			220 // Ãœ
#define PS2_Y_ACUTE			221 // Ã�
#define PS2_THORN			222 // Ãž
#define PS2_SHARP_S			223 // ÃŸ
#define PS2_a_GRAVE			224 // Ã 
#define PS2_a_ACUTE			225 // Ã¡
#define PS2_a_CIRCUMFLEX		226 // Ã¢
#define PS2_a_TILDE			227 // Ã£
#define PS2_a_DIAERESIS			228 // Ã¤
#define PS2_a_RING_ABOVE		229 // Ã¥
#define PS2_ae				230 // Ã¦
#define PS2_c_CEDILLA			231 // Ã§
#define PS2_e_GRAVE			232 // Ã¨
#define PS2_e_ACUTE			233 // Ã©
#define PS2_e_CIRCUMFLEX		234 // Ãª
#define PS2_e_DIAERESIS			235 // Ã«
#define PS2_i_GRAVE			236 // Ã¬
#define PS2_i_ACUTE			237 // Ã­
#define PS2_i_CIRCUMFLEX		238 // Ã®
#define PS2_i_DIAERESIS			239 // Ã¯
#define PS2_eth				240 // Ã°
#define PS2_n_TILDE			241 // Ã±
#define PS2_o_GRAVE			242 // Ã²
#define PS2_o_ACUTE			243 // Ã³
#define PS2_o_CIRCUMFLEX		244 // Ã´
#define PS2_o_TILDE			245 // Ãµ
#define PS2_o_DIAERESIS			246 // Ã¶
#define PS2_DIVISION			247 // Ã·
#define PS2_o_STROKE			248 // Ã¸
#define PS2_u_GRAVE			249 // Ã¹
#define PS2_u_ACUTE			250 // Ãº
#define PS2_u_CIRCUMFLEX		251 // Ã»
#define PS2_u_DIAERESIS			252 // Ã¼
#define PS2_y_ACUTE			253 // Ã½
#define PS2_thorn			254 // Ã¾
#define PS2_y_DIAERESIS			255 // Ã¿

#define PS2_KEYMAP_SIZE 136

typedef struct {
	uint8_t noshift[PS2_KEYMAP_SIZE];
	//uint8_t shift[PS2_KEYMAP_SIZE];
	unsigned int uses_altgr;
    /*
     * "uint8_t uses_altgr;" makes the ESP8266 - NodeMCU modules crash.
     * So, I replaced it with an int and... It works!
     * I think it's because of the 32-bit architecture of the ESP8266
     * and the use of the flash memory to store the keymaps.
     * Maybe I'm wrong, it remains a hypothesis.
     */
	uint8_t altgr[PS2_KEYMAP_SIZE];
} PS2Keymap_t;


#define BUFFER_SIZE 45
typedef struct{
	volatile uint8_t buffer[BUFFER_SIZE]; //MUDEI static volatile
	volatile uint8_t head, tail; //MUDEI static volatile
	uint8_t DataPin; //MUDEI static
	GPIO_TypeDef* DataPort;
	uint8_t IQRPin;
	GPIO_TypeDef* IQRPort;
	uint8_t CharBuffer; //MUDEI static
	uint8_t UTF8next; //MUDEI static
	const PS2Keymap_t *keymap;
} Keyboard_TypeDef;



/* ---------------FUNCTION DEFINITIONS---------------*/
void keyboardBegin(Keyboard_TypeDef* keyboard, GPIO_TypeDef* data_port, uint8_t data_pin, GPIO_TypeDef* iqr_port, uint8_t iqr_pin);
void ps2interrupt(Keyboard_TypeDef* keyboard);
uint8_t keyboardAvailable(Keyboard_TypeDef* keyboard);
void keyboardClear(Keyboard_TypeDef* keyboard);
uint8_t keyboardReadScanCode(Keyboard_TypeDef* keyboard);
uint8_t keyboardRead(Keyboard_TypeDef* keyboard);
void interruption();
// ******************AO DAR BEGIN NO TELCADO, SETAR CharBuffer, UTF8next e keymap para 0

//**********COLOCAR A MATRIZ DO LAYOUT DO TECLADO!!!1 ******************

const PROGMEM PS2Keymap_t PS2Keymap_US;

/*extern const PROGMEM PS2Keymap_t PS2Keymap_German;
extern const PROGMEM PS2Keymap_t PS2Keymap_French;
extern const PROGMEM PS2Keymap_t PS2Keymap_Spanish;
extern const PROGMEM PS2Keymap_t PS2Keymap_Italian;
extern const PROGMEM PS2Keymap_t PS2Keymap_UK;
*/



#endif /* PS2KEYBOARD_H_ */
