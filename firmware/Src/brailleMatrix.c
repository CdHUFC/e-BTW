

#include "brailleMatrix.h"


/* Para letras: preenche os 2 pontos referentes à linha do caractere informado
 * Para números: preenche os 3 pontos referentes à linha do caractere informado
 * linhaBraille: a linha de um caractere
 * line: a linha que vai ser alocada ou 0, ou 1, ou 2
 * letter: letra recebida do teclado
 */

static unsigned char lettersBraille[28][3][2]={
  {{48,49},{48,48},{48,48}},
  {{48,49},{48,49},{48,48}},
  {{49,49},{48,48},{48,48}},
  {{49,49},{49,48},{48,48}},
  {{48,49},{49,48},{48,48}},
  {{49,49},{48,49},{48,48}},
  {{49,49},{49,49},{48,48}},
  {{48,49},{49,49},{48,48}},
  {{49,48},{48,49},{48,48}},
  {{49,48},{49,49},{48,48}},
  {{48,49},{48,48},{48,49}},
  {{48,49},{48,49},{48,49}},
  {{49,49},{48,48},{48,49}},
  {{49,49},{49,48},{48,49}},
  {{48,49},{49,48},{48,49}},
  {{49,49},{48,49},{48,49}},
  {{49,49},{49,49},{48,49}},
  {{48,49},{49,49},{48,49}},
  {{49,48},{48,49},{48,49}},
  {{49,48},{49,49},{48,49}},
  {{48,49},{48,48},{49,49}},
  {{48,49},{48,49},{49,49}},
  {{49,48},{49,49},{49,48}},
  {{49,49},{48,48},{49,49}},
  {{49,49},{49,48},{49,49}},
  {{48,49},{49,48},{49,49}},
  {{48,48},{48,48},{48,48}}
            };



/*
 * OVERFLOW NA MEMÓRIA FLASH
 */
//unsigned char numbersBraille[10][3][4]={
//  {{1,0,1,0},{1,1,1,0},{0,0,1,1}},
//  {{0,1,1,0},{0,0,1,0},{0,0,1,1}},
//  {{0,1,1,0},{0,1,1,0},{0,0,1,1}},
//  {{1,1,1,0},{0,0,1,0},{0,0,1,1}},
//  {{1,1,1,0},{1,0,1,0},{0,0,1,1}},
//  {{0,1,1,0},{1,0,1,0},{0,0,1,1}},
//  {{1,1,1,0},{0,1,1,0},{0,0,1,1}},
//  {{1,1,1,0},{1,1,1,0},{0,0,1,1}},
//  {{0,1,1,0},{1,1,1,0},{0,0,1,1}},
//  {{1,0,1,0},{0,1,1,0},{0,0,1,1}}
//                  };
void fillLineWithBraille( unsigned char *lineBraille, unsigned char line, unsigned char letter){

	/*
	 * OVERFLOW NA MEMÓRIA FLASH
	 */
	if(letter>=48 && letter<=57){
//		for(int i=0;i<4;i++){
//			lineBraille[i]=numbersBraille[letter-48][line][i];
//		}
	}
	else{
		if(letter>=65 && letter<=90){
			letter+=32;
		}

		for(int i=0;i<2;i++){
			if(letter>=97 && letter<=122){
				lineBraille[i]=lettersBraille[letter-97][line][i];
			}
			if(letter==32){
				lineBraille[i]=0;
			}
		}
		lineBraille[2]='\0';
		lineBraille[3]='\0';


	}
}



/*
 * Adiciona caracteres a uma string
 */

void feedBuffer(unsigned char* buffer, short SIZE, unsigned char c){
    unsigned short len = strlen((const char*)buffer);
    if(len != (SIZE-1)){
        buffer[len] = c;
        buffer[len+1] = '\0';
    }
}

/*
 * Limpa uma string
 */
void clearBuffer(unsigned char* buffer){
    buffer[0] = '\0';
}

