#ifndef SRC_NIKYUSBDEV_H_
#define SRC_NIKYUSBDEV_H_

#define IsMidiUsbEn (0)  // 1 для компиляции с аудиоинтерфейсами миди

#include <stdint.h>

#include "main.h"


// Вызвать в самом начале до старта usb
void MyUsbDevInitGlobal();

// Старт моего устройства usb
void MyUsbDevStart();


extern volatile uint8_t m_UsbCmdRetBuf[256];  // и буфер ответа для отправки в комп через EP1_IN

// Если EP1_AutoSendStatus не ноль, то HAL_PCD_DataInStageCallback() начинает что-то отправлять по своей инициативе
//
extern volatile int m_EP1_AutoSendStatus;
extern uint8_t * volatile m_EP1_AutoSend_Buf;
extern volatile int m_EP1_AutoSend_BufSz;


#define OutUsartBuf2_NMax (2048) // СТРОГО степень двойки !!!
#define OutUsartBuf2_Mask (OutUsartBuf2_NMax-1)

extern volatile uint16_t m_OutUsartBufferPosWr;
extern volatile uint8_t m_OutUsartBuf2[OutUsartBuf2_NMax];
extern volatile int m_OutUsartBufferStatus;
//extern volatile int m_OutUsartBufferStatusClr;
extern volatile uint16_t m_OutUsartBufferPosSend;
void MyUsbDebug_HAL_PCD_EP_Flush(uint8_t ep_adr);


static inline void EP2_Send()
{
  if (m_OutUsartBufferStatus) return;  // USB отправка уже работает, надо ждать окончания
  USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_SOFM; // переадресовать вызов прерыванию с приоритетом usb
}


extern volatile uint8_t m_UsbMidiOutBufPosWr;
extern volatile uint32_t m_UsbMidiOutBuf[256];
extern volatile int m_UsbMidiOutBufStatus;
extern volatile uint8_t m_UsbMidiOutBufPosSend;

extern uint8_t volatile NikyUsbDebug_GetUrlN;
extern uint8_t volatile NikyUsbDebug_GetBOSDescriptorN;



static inline void EP3Midi_Send()
{
  if (m_UsbMidiOutBufStatus) return;  // USB уже работает, надо ждать окончания
  USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_SOFM; // переадресовать вызов прерыванию с приоритетом usb
}


#define OutUsartBuf_WrB(PosWr, Data) { m_OutUsartBuf2[PosWr++] = Data; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0; }
#define OutUsartBuf_WrW(PosWr, Data) { m_OutUsartBuf2[PosWr++] = Data; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0; m_OutUsartBuf2[PosWr++] = Data>>8; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0; }
#define OutUsartBuf_WrD(PosWr, Data) { m_OutUsartBuf2[PosWr++] = Data; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0; m_OutUsartBuf2[PosWr++] = Data>>8; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0; m_OutUsartBuf2[PosWr++] = Data>>16; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0; m_OutUsartBuf2[PosWr++] = Data>>24; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0; }
#define OutUsartBuf_WrQ(PosWr, Data) { m_OutUsartBuf2[PosWr++] = Data; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0; m_OutUsartBuf2[PosWr++] = Data>>8; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0; m_OutUsartBuf2[PosWr++] = Data>>16; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0; m_OutUsartBuf2[PosWr++] = Data>>24; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0;   m_OutUsartBuf2[PosWr++] = Data>>32; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0; m_OutUsartBuf2[PosWr++] = Data>>40; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0; m_OutUsartBuf2[PosWr++] = Data>>48; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0; m_OutUsartBuf2[PosWr++] = Data>>56; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0; }

#define OutUsartBuf_WrQ5(PosWr, Data) { m_OutUsartBuf2[PosWr++] = Data; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0; m_OutUsartBuf2[PosWr++] = Data>>8; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0; m_OutUsartBuf2[PosWr++] = Data>>16; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0; m_OutUsartBuf2[PosWr++] = Data>>24; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0;   m_OutUsartBuf2[PosWr++] = Data>>32; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0; }



// Отмена какой либо предыдущей отправки на EP1_IN
//
void NikyUsbDev_EP1Clr();


// Посыл ответа в комп, чисто вызов HAL_PCD_EP_Transmit()
//
void NikyUsbDev_EP1Send(volatile uint8_t *Buf, int Sz);

// Посыл ответа на команду, опредыдущая отправка сперва отменяется
// Хотя по хорошему лучше прерывать сперва NikyUsbDev_EP1Clr() до модификации буфера
//
void NikyUsbDev_EP1OutCmdRet(volatile uint8_t *Buf, int Sz);






#endif /* SRC_NIKYUSBDEV_H_ */
