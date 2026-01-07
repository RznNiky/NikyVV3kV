#ifndef Niky_I2C_File_Invluded_AAASSSDDDFWQE1
#define Niky_I2C_File_Invluded_AAASSSDDDFWQE1

#include <stdint.h>

void I2C_Start();
void I2C_Stop();
void I2C_RepStart();

void I2C_Out(uint8_t I2C_DataByte);

uint8_t I2C_Input();   // только 8 бит, ACK бит надо отдельно

void I2C_Input_Ask();  // варианты бита 9
void I2C_Input_NAsk();


extern uint8_t I2C_Error;

void I2CSetAlt(uint8_t a);



#endif
