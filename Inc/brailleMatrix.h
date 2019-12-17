/*
 * brailleMatrix.h
 *
 *  Created on: 6 de dez de 2018
 *      Author: minhatu
 */

#ifndef BRAILLEMATRIX_H_
#define BRAILLEMATRIX_H_

#include "stm32f1xx_it.h"
#include <string.h>



void fillLineWithBraille( unsigned char *linhaBraille, unsigned char line, unsigned char letter);
void feedBuffer(unsigned char* buffer, short SIZE, unsigned char c);
void clearBuffer(unsigned char* buffer);
void reverse (unsigned char* buffer);
#endif /* BRAILLEMATRIX_H_ */
