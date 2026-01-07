#ifndef SRC_OLED96_H_
#define SRC_OLED96_H_

#include <stdint.h>

void OledInit(uint8_t IsPere, uint8_t IsInverse);
void OledClear();

void OledPrintChar(int col, int page, uint8_t ch);
//void OledPrintCharX2(int col, int page, uint8_t ch);
void OledPrintCharX1_5(int col, int page, uint8_t ch);
void OledPrintCharX3(int col, int page, uint8_t ch);


int OledPrintX1(int col, int page, const char * str);
//int OledPrintX2(int col, int page, const char * str);
int OledPrintX1_5(int col, int page, const char * str);
int OledPrintX3(int col, int page, const char * str);




#endif /* SRC_OLED96_H_ */
