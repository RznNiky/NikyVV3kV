#include "NikyI2C.h"

#include "main.h"

uint8_t I2C_Error;



static inline void I2C_SCL_Hi()
{
  LL_GPIO_SetOutputPin(NikyI2C_SCL_GPIO_Port, NikyI2C_SCL_Pin);
}
static inline void I2C_SCL_Lo()
{
  LL_GPIO_ResetOutputPin(NikyI2C_SCL_GPIO_Port, NikyI2C_SCL_Pin);
}
static inline void I2C_SDA_Hi()
{
  LL_GPIO_SetOutputPin(NikyI2C_SDA_GPIO_Port, NikyI2C_SDA_Pin);
}
static inline void I2C_SDA_Lo()
{
  LL_GPIO_ResetOutputPin(NikyI2C_SDA_GPIO_Port, NikyI2C_SDA_Pin);
};
static inline uint32_t I2C_SDA_Read()
{
  return LL_GPIO_IsInputPinSet(NikyI2C_SDA_GPIO_Port, NikyI2C_SDA_Pin);
}
static inline void I2C_Wait()
{
  const uint32_t T0 = TIM2->CNT;
  while (TIM2->CNT - T0 < 420/2) ;
}

void I2C_Start()
{
  I2C_SCL_Hi();
  I2C_SDA_Hi();
  I2C_Wait();
  I2C_SDA_Lo();
  I2C_Wait();
};

void I2C_Stop()
{
  I2C_SCL_Lo();
  I2C_Wait();
  I2C_SDA_Lo();
  I2C_Wait();
  I2C_SCL_Hi();
  I2C_Wait();
  I2C_SDA_Hi();
  I2C_Wait();
};
    ;
void I2C_RepStart()
{
  I2C_SCL_Lo();
  I2C_Wait();
  I2C_SDA_Hi();
  I2C_Wait();
  I2C_SCL_Hi();
  I2C_Wait();
  I2C_SDA_Lo();
  I2C_Wait();
};



void I2C_Out(uint8_t I2C_DataByte)  // на входе w - передаваемый по I2C байт
{
  for (int i=0; i<9; i++)
  {
	I2C_SCL_Lo();
    I2C_Wait();
    ;
    if (I2C_DataByte & 0x80) I2C_SDA_Hi(); else I2C_SDA_Lo();  // выводим на шину SDA старший информационный бит
    I2C_Wait();    // и ждем
    ;
    I2C_SCL_Hi();  // устанавливаем высокий уровень SCL и ждем
    I2C_Wait();
    ;
    // теперь можно посмотреть, какой сигнал реально на шине данных
    uint8_t rd = I2C_SDA_Read() ? 0x80 : 0;

    if ((I2C_DataByte ^ rd) & 0x80) // отличаются данные
    {
      if (i==8) return; // ACK принят, отлично
      I2C_Error |= 2;   // ошибка, состояние шины данных при передаче не соответствует норме
      //return;
    };

    I2C_DataByte <<= 1;  // готовимся передавать следующий бит
    I2C_DataByte |= 1;
  };

  // Ошибка, все 9-ть бит переданы как есть, т.е. ACK отсутствует
  I2C_Error |= 1;
};

uint8_t I2C_Input()
{
  uint8_t I2C_DataByte = 0;
  for (int i=0; i<8; i++)
  {
    I2C_SCL_Lo();
    I2C_Wait();
    I2C_SDA_Hi();
    I2C_Wait();
    I2C_SCL_Hi();
    I2C_Wait();
    ;
    I2C_DataByte <<= 1;
    if (I2C_SDA_Read()) I2C_DataByte |= 1; // читаем бит данных
  };
  return I2C_DataByte;
};

void I2C_Input_Ask()
{
  I2C_SCL_Lo();
  I2C_Wait();
  I2C_SDA_Lo();
  I2C_Wait();
  I2C_SCL_Hi();
  I2C_Wait();
};

void I2C_Input_NAsk()
{
  I2C_SCL_Lo();
  I2C_Wait();
  I2C_SDA_Hi();
  I2C_Wait();
  I2C_SCL_Hi();
  I2C_Wait();
};


