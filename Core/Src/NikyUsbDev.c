#include "NikyUsbDev.h"

//#include "usbd_conf.h"
//#include "usbd_def.h"

#include <assert.h> // ради static_assert

#define NDBG 0   // 1 для отладки обмена usb через ком-порт


#if IsMidiUsbEn == 1
#include "NikyUsbDecMidi.h"
#endif

#define __ToCCMRAM // __attribute__((section (".ccmram")))


extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

static void Usart_Out(uint8_t value)  // только ради отладки обмена
{
  //while (!LL_USART_IsActiveFlag_TXE(USART2)) ;  // Check if the USART Transmit Data Register Empty Flag is set
  //LL_USART_TransmitData8(USART2, value);
};

static void Usart_Out16(uint16_t value)
{
	Usart_Out(value);
	Usart_Out(value>>8);
};
/*static void Usart_Out32(uint32_t value)
{
	Usart_Out16(value);
	Usart_Out16(value>>16);
};*/

volatile uint8_t m_UsbCmdBuf[64]  __ALIGNED(4);     // принятая команда от компа на EP1_OUT
volatile uint8_t m_UsbCmdRetBuf[256] __ALIGNED(4);  // и буфер ответа для отправки в комп через EP1_IN

static uint8_t m_UsbMidiRcvBuf[64]  __ALIGNED(4);   // буфер приёма данных миди от компа

uint8_t volatile NikyUsbDebug_GetUrlN = 0;
uint8_t volatile NikyUsbDebug_GetBOSDescriptorN = 0;
uint8_t volatile NikyUsbDebug_Get_MS_OS_20_Descriptor_N = 0;



// Если EP1_AutoSendStatus не ноль, то HAL_PCD_DataInStageCallback() начинает что-то отправлять по своей инициативе
//
volatile int m_EP1_AutoSendStatus __ToCCMRAM;
uint8_t * volatile m_EP1_AutoSend_Buf __ToCCMRAM;
volatile int m_EP1_AutoSend_BufSz __ToCCMRAM;

// Для отправки данных в комп через EP2
#define OutUsartBuf2_NMax (2048) // СТРОГО степень двойки !!!
#define OutUsartBuf2_Mask (OutUsartBuf2_NMax-1)
//
// Для отправки данных в комп через EP2
//
volatile uint16_t m_OutUsartBufferPosWr = 0;
volatile uint8_t m_OutUsartBuf2[OutUsartBuf2_NMax] __attribute__((section (".ccmram")));
volatile int m_OutUsartBufferStatus = 255;
//volatile int m_OutUsartBufferStatusClr = 0;
volatile uint16_t m_OutUsartBufferPosSend = 0;

static uint8_t m_OutUsartBufExt[64] __attribute__((section (".ccmram")));  // буфер отправки одного макс. пакета, если он разорван границей буфера

void AddDebugEP2(uint8_t UsbAct, uint8_t UsbData)
{
  uint16_t PosWr = m_OutUsartBufferPosWr;
  m_OutUsartBuf2[PosWr++] = 0xcd;    if (PosWr >= OutUsartBuf2_NMax) PosWr = 0;
  m_OutUsartBuf2[PosWr++] = 0xd3;    if (PosWr >= OutUsartBuf2_NMax) PosWr = 0;
  m_OutUsartBuf2[PosWr++] = UsbAct;  if (PosWr >= OutUsartBuf2_NMax) PosWr = 0;
  m_OutUsartBuf2[PosWr++] = UsbData; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0;
  m_OutUsartBufferPosWr = PosWr;
  //EP2_Send();
};

// Для отправки данных в комп данных миди через EP3
//
volatile uint8_t m_UsbMidiOutBufPosWr __ToCCMRAM;
volatile uint32_t m_UsbMidiOutBuf[256] __ALIGNED(4) __ToCCMRAM ;
volatile int m_UsbMidiOutBufStatus __ToCCMRAM;
volatile uint8_t m_UsbMidiOutBufPosSend __ToCCMRAM;

static volatile uint8_t g_IsConfigSetAlready __ToCCMRAM;

void MyUsbDevInitGlobal()
{
	// Проинициализируем глобальные переменные, поскольку они в CCMRAM и это не делается автоматом

	m_EP1_AutoSendStatus = 0;
	m_EP1_AutoSend_Buf = NULL;
	m_EP1_AutoSend_BufSz = 0;

	m_OutUsartBufferPosWr = 0;
	m_OutUsartBufferStatus = 1;  // не ноль - занято, посыл запрещен, сбрасывается в ProcessSetupToken_SetConfig()
	m_OutUsartBufferPosSend = 0;

	m_UsbMidiOutBufPosWr = 0;
	m_UsbMidiOutBufStatus = 1;
	m_UsbMidiOutBufPosSend = 0;

	g_IsConfigSetAlready = 0;
};

void MyUsbDevStart()
{
	  // USB features a dedicated RAM of 1.25 Kbytes (320 слов = 128+128+64 слов) with advanced FIFO control

	  HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 128);   // Пишет прямо в OTG_FS_GRXFSIZ в элементах по 32 бита
	  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 32); // тоже в словах по 32 бита
#if IsMidiUsbEn == 1
	  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 64); // тоже в словах по 32 бита
	  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 2, 64); // тоже в словах по 32 бита
	  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 3, 32); // тоже в словах по 32 бита
#else
	  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 64);
	  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 2, 64+32);
#endif

	  //Usart_Out(0xaa);
	  HAL_PCD_Start(&hpcd_USB_OTG_FS);  // запуск устр-ва
};



// см. USB_MAX_EP0_SIZE в usbd_def.h
#define MAX_EP0_PACKET_SIZE	32 // maximum packet size for low-speed peripherals is 8 bytes, for full-speed peripherals it can be 8, 16, 32, or 64 bytes
#define	NUM_CONFIGURATIONS	 1

#if IsMidiUsbEn != 1
#define	NUM_INTERFACES		 1
#endif
#if IsMidiUsbEn == 1
#define	NUM_INTERFACES		 3
#endif



//void OldHAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
	if (NDBG) Usart_Out(0x10);
    //OldHAL_PCD_ResetCallback(hpcd);

	HAL_PCD_EP_Open(hpcd, 0,    MAX_EP0_PACKET_SIZE, EP_TYPE_CTRL);  // OUT
	HAL_PCD_EP_Open(hpcd, 0x80, MAX_EP0_PACKET_SIZE, EP_TYPE_CTRL);  // IN

	m_EP1_AutoSendStatus = 0;
	m_OutUsartBufferStatus = 1; // запрет пытаться слать данные в неготовое устро-во
	m_UsbMidiOutBufStatus = 1;  // запрет посыла в неготовое устройство

	g_IsConfigSetAlready = 0;   // setconfig ещё не вызывался после сброса
};


// Standard descriptor types
//
#define	DEVICE_DESCRIPTOR_TYPE		    1
#define	CONFIGURATION_DESCRIPTOR_TYPE	2
#define	STRING_DESCRIPTOR_TYPE	     	3
#define	INTERFACE_DESCRIPTOR_TYPE	    4
#define	ENDPOINT_DESCRIPTOR_TYPE     	5
#define BOS_DESCRIPTOR_TYPE             (0x0F)


static const uint8_t MyDescriptor_Device[] __ALIGNED(4) =
{
  0x12, DEVICE_DESCRIPTOR_TYPE,  // bLength, bDescriptorType
  0x10, 0x02,                    // bcdUSB (low byte), bcdUSB (high byte)    // 2.10 надо для BOS и WebUsb
  0x00, 0x00,                    // bDeviceClass, bDeviceSubClass
  0x00, MAX_EP0_PACKET_SIZE,     // bDeviceProtocol, bMaxPacketSize
  0xD8, 0x04,                    // idVendor (low byte), idVendor (high byte)
  0xa2, 0xd7+IsMidiUsbEn,        // idProduct (low byte), idProduct (high byte)
  0x00, 0x01,                    // bcdDevice (low byte), bcdDevice (high byte)
  0x01, 0x02,                    // iManufacturer, iProduct (строки)
  0x03, NUM_CONFIGURATIONS       // iSerialNumber, bNumConfigurations
};

#pragma pack(push, 1)

typedef struct _USB_CONFIGURATION_DESCRIPTOR
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wTotalLength;
    //uint8_t wTotalLengths[2];
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t MaxPower;
} USB_CONFIGURATION_DESCRIPTOR;

typedef struct _USB_INTERFACE_DESCRIPTOR
{
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bInterfaceNumber;    // A zero-based value identifying the index in the array of concurrent interfaces supported by this configuration
	uint8_t bAlternateSetting;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;          // Index of a string descriptor that describes	this interface
} USB_INTERFACE_DESCRIPTOR;

typedef struct _USB_ENDPOINT_DESCRIPTOR
{
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
	uint8_t aaa, bbb;
} USB_ENDPOINT_DESCRIPTOR;



typedef struct
{
  USB_CONFIGURATION_DESCRIPTOR cfg;
  USB_INTERFACE_DESCRIPTOR intf;
  USB_ENDPOINT_DESCRIPTOR ep1, ep2, ep3, ep4;

#if IsMidiUsbEn == 1
  USB_INTERFACE_DESCRIPTOR intf0_audioCtrl;
  USB_AUDIOCLASS_INTERFACE_DESCRIPTOR intf0_audioCtrl__spec;

  USB_INTERFACE_DESCRIPTOR intf1_midiStream;
  USB_MIDISTRIMING_INTERFACE_DESCRIPTOR intf2_midiStream_spec;

  USB_MIDI_IN_JACK_DESCRIPTOR Midi_In_Jack1;
  USB_MIDI_IN_JACK_DESCRIPTOR Midi_In_Jack2;

  USB_MIDI_OUT_JACK_DESCRIPTOR Midi_Out_Jack1;
  USB_MIDI_OUT_JACK_DESCRIPTOR Midi_Out_Jack2;

  USB_ENDPOINT_DESCRIPTOR ep1_out;
  USB_MIDISTREAM_BULK_OUT_EP_DESCRIPTOR ep1_out_sc;

  USB_ENDPOINT_DESCRIPTOR ep1_in;
  USB_MIDISTREAM_BULK_OUT_EP_DESCRIPTOR ep1_in_sc;
#endif

} TNikyDevCfg1;

const TNikyDevCfg1 NikyDevCfg1 __ALIGNED(4) =
{
 // *** Дескриптор конфигурации
 {
    sizeof(USB_CONFIGURATION_DESCRIPTOR), // 0x09,
	CONFIGURATION_DESCRIPTOR_TYPE,        // bDescriptorType
    sizeof(TNikyDevCfg1),                // сколько всего байт возврящается на запрос дескриптора конфигурации (с вложенными дескрипторами)
	//sizeof(TNikyDevCfg1)>>8,
	//sizeof(TNikyDevCfg1),
    NUM_INTERFACES, 0x01,  // bNumInterfaces, bConfigurationValue (Идентификатор конфигурации)
    0x00, 0xc0,            // iConfiguration (строка), bmAttributes (было 0xA0, D7=1 - резерв, D6 - собств.пит, D5 - пробуждение)
    0x32,                  // bMaxPower (100 mA)
 },

 // *** Мой хитрый интерфейс и его EP 4 шт

 {  sizeof(USB_INTERFACE_DESCRIPTOR),
    INTERFACE_DESCRIPTOR_TYPE, 0x00,	// bDescriptorType, bInterfaceNumber (от 0)
    0x00, 0x04,			// bAlternateSetting, bNumEndpoints (не считая EP0)
    0xFF, 0x00,			// bInterfaceClass (FF-vendor specific class code), bInterfaceSubClass
    0x00,               // bInterfaceProtocol (vendor specific protocol used),
	0x04			    // iInterface (строка)
  },

  {	 0x09, ENDPOINT_DESCRIPTOR_TYPE,  // bLength, bDescriptorType
 	 0x81, EP_TYPE_BULK,              // bEndpointAddress (EP1 IN), bmAttributes (передача данных)
 	 64,                              // wMaxPacketSize
 	 0x00,                            // bInterval
  },
  {	 0x09, ENDPOINT_DESCRIPTOR_TYPE,
	 0x01, EP_TYPE_BULK,
 	 64,                              // wMaxPacketSize (high byte),
 	 0x00,                            // bInterval
  },

  {  0x09, ENDPOINT_DESCRIPTOR_TYPE,  // bLength, bDescriptorType
 	 0x82, EP_TYPE_BULK,              // bEndpointAddress (EP2 OUT), bmAttributes
 	 64,             				  // wMaxPacketSize)
 	 0x00,                            // интервал
  },
  {	 0x09, ENDPOINT_DESCRIPTOR_TYPE,
	 0x02, EP_TYPE_BULK,
 	 64,                              // wMaxPacketSize (high byte),
 	 0x00,                            // bInterval
  },


#if IsMidiUsbEn == 1
  // *** Начитаем с описания дескриптора управляющего интерфейса для аудиокласса
  {
     sizeof(USB_INTERFACE_DESCRIPTOR),
     INTERFACE_DESCRIPTOR_TYPE, 0x01,  // bDescriptorType, bInterfaceNumber (от 0)
     0x00, // bAlternateSetting
 	 0x00, // bNumEndpoints (EP0, тут 0, ибо AudioControl interface не использует endpoints, кроме управляющей endpoint 0)
 	 USB_CLASS_AUDIO, USB_AUDIO_AUDIOCONTROL,	// bInterfaceClass, bInterfaceSubClass - это аудиокласс, интерфейс управления
     0x00, 0x05,	 	                        // bInterfaceProtocol (Unused), iInterface (Unused)
  },

  // далее следует класс-специфичный descriptor для аудиокласса
  //
  {
    sizeof(USB_AUDIOCLASS_INTERFACE_DESCRIPTOR),
    USB_SC_INTERFACE_DESCRIPTOR_TYPE, // bDescriptorType, 0x24 - CS_INTERFACE
    1,                                // 1 - HEADER subtype
    0x100,                            // Revision of  class specification - 1.0
    0x009,                            // Total size of class specific descriptors (в данном случае только сам этот дескриптор)
    1,                                // Number of streaming interfaces (кол-во интерфейсов MIDIStreaming)
    { 2 }                             // MIDIStreaming interface belongs to this AudioControl interface
  },


  // Второй дескриптор интерфейса - класс AUDIO, подкласс MIDISTREAMING
  {
     sizeof(USB_INTERFACE_DESCRIPTOR),
     INTERFACE_DESCRIPTOR_TYPE, 0x02,  // bDescriptorType, bInterfaceNumber (от 0, это уже второй после audiocontrol)
     0x00, // bAlternateSetting
 	 0x02, // bNumEndpoints (не считая управляющую endpoint 0)
 	 USB_CLASS_AUDIO, USB_AUDIO_MIDISTRIMING,	// bInterfaceClass, bInterfaceSubClass - это аудиокласс, MIDISTRIMING
     0x00, 0x00,	 	                        // bInterfaceProtocol (Unused), iInterface (Unused)
  },

  // и сразу класс-специфичный дескриптор для midistreaming
  {
    sizeof(USB_MIDISTRIMING_INTERFACE_DESCRIPTOR),
    USB_SC_INTERFACE_DESCRIPTOR_TYPE, // bDescriptorType = 0x24 - CS_INTERFACE
    MIDI_CS_IF_HEADER,                // 1 - HEADER subtype
    0x100,                            // Revision of  class specification - 1.0

    // Тут размер этого дескриптора + размеры всего специфического ниже: дескрипторов jack, EP и их классспецифичных расширений
    sizeof(USB_MIDISTRIMING_INTERFACE_DESCRIPTOR) +
	sizeof(USB_MIDI_IN_JACK_DESCRIPTOR)*2 +
	sizeof(USB_MIDI_OUT_JACK_DESCRIPTOR)*2 +
	sizeof(USB_ENDPOINT_DESCRIPTOR)*2 +
	sizeof(USB_MIDISTREAM_BULK_OUT_EP_DESCRIPTOR)*2
  },

  // EMB:  IN Jack #1 -----> EXT: OUT Jack #4
  // EMB: OUT Jack #3 <----- EXT:  IN Jack #2

  // Описание midi IN jack
  {
     sizeof(USB_MIDI_IN_JACK_DESCRIPTOR),
 	 USB_SC_INTERFACE_DESCRIPTOR_TYPE, MIDI_CS_IF_IN_JACK,
 	 MIDI_JACK_TYPE_EMB,
     1,  // bJackID
 	 0   // iJack, строка
  },
  {
     sizeof(USB_MIDI_IN_JACK_DESCRIPTOR),
 	 USB_SC_INTERFACE_DESCRIPTOR_TYPE, MIDI_CS_IF_IN_JACK,
 	 MIDI_JACK_TYPE_EXT,
     2,  // bJackID
   	 0   // iJack, строка
  },


  // Описание midi OUT jack

  {
     sizeof(USB_MIDI_OUT_JACK_DESCRIPTOR),
 	USB_SC_INTERFACE_DESCRIPTOR_TYPE, MIDI_CS_IF_OUT_JACK,
 	MIDI_JACK_TYPE_EMB,
    3,     // bJackID
 	1,     // bNrInputPins
 	2, 1,  // ID of the Entity to which this Pin is connected и Output Pin number of the Entity to which this Input Pin is connected
 	0
  },

  {
     sizeof(USB_MIDI_OUT_JACK_DESCRIPTOR),
 	USB_SC_INTERFACE_DESCRIPTOR_TYPE, MIDI_CS_IF_OUT_JACK,
 	MIDI_JACK_TYPE_EXT,
    4,     // bJackID
 	1,     // bNrInputPins
 	1, 1,  // ID of the Entity to which this Pin is connected и Output Pin number of the Entity to which this Input Pin is connected
 	0
  },

  // EP1 OUT
  //
  {		 0x09,
 		 ENDPOINT_DESCRIPTOR_TYPE,
 		 0x03,  // bEndpointAddress
 		 EP_TYPE_BULK,  // bulk
 		 64,    // wMaxPacketSize (high byte),
 		 0x01*0,  // bInterval (1 ms)
 		 0,0
  },

  // EP1 OUT class специфичный
  {
     sizeof(USB_MIDISTREAM_BULK_OUT_EP_DESCRIPTOR),
 	 USB_MIDI_CS_EP_DESCRIPTOR, USB_MIDI_CS_EP_MS_GENERAL,
 	 1,
 	 1  // 1 - ID of the Embedded MIDI IN Jack
  },



  // EP1 IN
  {		 0x09, ENDPOINT_DESCRIPTOR_TYPE,  // bLength, bDescriptorType
 		 0x83,                            // bEndpointAddress (EP1 IN)
 		 EP_TYPE_BULK,                    //  bmAttributes (2-bulk)
 		 64,                              // wMaxPacketSize
 		 0x01*0,                          // bInterval (1 ms)
 		 0,0
  },

  // EP1 IN class специфичный
  {
    sizeof(USB_MIDISTREAM_BULK_IN_EP_DESCRIPTOR),
 	USB_MIDI_CS_EP_DESCRIPTOR, USB_MIDI_CS_EP_MS_GENERAL,
 	1,
 	3  // 3 - ID of the Embedded MIDI OUT Jack
  },


#endif

};

#pragma pack(pop)


void * GetMyDevCfg()
{
	return (void *) &NikyDevCfg1;
};
int GetMyDevCfgSz()
{
	return sizeof(NikyDevCfg1);
};


const int8_t MyDescriptor_String0[] =
{   4, STRING_DESCRIPTOR_TYPE, //bLength, bDescriptorType
	0x09, 0x04,  // Англ (США) wLANGID[0] (low byte), wLANGID[0] (high byte)
	//0x19, 0x04   // Русский    wLANGID[1] (low byte), wLANGID[1] (high byte)
};

const uint16_t MyDescriptor_String1_Manufacturer[] =
{
  2 + 18*2 + 256*STRING_DESCRIPTOR_TYPE,
  L'z',
  L'v',
  L'z',
  L'd',
  L'3',
  L'd',
  L'.',
  L'r',
  L'u',
  L' ',
  L'N',
  L'i',
  L'k',
  L'y',
  L'2',
  L'0',
  L'2',
  L'5'
};

const uint16_t MyDescriptor_String2_Product[] =
{
  2 + 10*2 + 256*STRING_DESCRIPTOR_TYPE,
  L'N',
  L'i',
  L'k',
  L'y',
  L'V',
  L'V',
  L'3',
  L'k',
  L'V',
  L'2',
};

const uint16_t MyDescriptor_String3_Serial[] =
{
  2 + 14*2 + 256*STRING_DESCRIPTOR_TYPE,
  L'N',
  L'i',
  L'k',
  L'y',
  L'2',
  L'0',
  L'2',
  L'5',
  L'V',
  L'V',
  L'3',
  L'k',
  L'V',
  L'2',
};

const uint16_t MyDescriptor_String4_NikyProgInterfaceName[] =
{
  2 + 9*2 + 256*STRING_DESCRIPTOR_TYPE,
  L'V',
  L'V',
  L'3',
  L'k',
  L'V',
  L'P',
  L'r',
  L'o',
  L'g',
};

const uint16_t MyDescriptor_String5_NikyMidiInterfaceName[] =
{
  2 + 8*2 + 256*STRING_DESCRIPTOR_TYPE,
  L'N',
  L'i',
  L'k',
  L'y',
  L'M',
  L'i',
  L'd',
  L'i',
};

#if IsMidiUsbEn == 1
#define MS_OS_20_LENGTH (0xB2) // 0xB2 - для композитных
#else
#define MS_OS_20_LENGTH (0xA2) // 0xA2 - для не композитных
#endif

const uint8_t MyBOSDescriptor[] =   // BOS (Binary Device Object Store) Descriptor
{
    // BOS Descriptor
    0x05,                // bLength
	BOS_DESCRIPTOR_TYPE, // bDescriptorType (BOS=0xF)
    5+24+28, 0,          // wTotalLength = 5+24+28=57=0x39
	0x02,                // bNumDeviceCaps - число дескрипторов, идущих дальше

	// === Capability 1: WebUSB ===

	// Device Capability: Platform Descriptor
    0x18,        // bLength (24 bytes)
    0x10,        // bDescriptorType (Device Capability)
    0x05,        // bDevCapabilityType (Platform)
    0x00,        // bReserved

    // Platform Capability UUID: {3408b638-09a9-47a0-8bfd-a0768815b665}
    0x38, 0xB6, 0x08, 0x34,
    0xA9, 0x09, 0xA0, 0x47,
    0x8B, 0xFD, 0xA0, 0x76,
    0x88, 0x15, 0xB6, 0x65,

    0x00, 0x01,  // bcdVersion (1.0)
    0x01,        // bVendorCode  (используется для запроса URL, подставляется в поле bRequest запроса)
    0x01,        // iLandingPage (индекс URL дескриптора, подставляется в поле wValue запроса)


	// === Capability 2: Microsoft OS 2.0 ===

	0x1c,        // bLength 0x1c = 28
	0x10,        // bDescriptorType: Device Capability
	0x05,        // bDevCapabilityType: Platform
	0x00,        // bReserved

	// MS OS 2.0 UUID: {D8DD60DF-4589-4CC7-9CD2-659D9E648A9F}
	0xDF, 0x60, 0xDD, 0xD8,
	0x89, 0x45, 0xC7, 0x4C,
	0x9C, 0xD2, 0x65, 0x9D,
	0x9E, 0x64, 0x8A, 0x9F,

	0, 0, 0x03, 0x06,    // dwWindowsVersion это 6.3 (т.е. Win8.1)

	MS_OS_20_LENGTH, 0,  // wMSOSDescriptorSetTotalLength, см. структуру MS_OS_20_Descriptor_Set

	0x02,                // bMS_VendorCode - bRequest value for retrieving further Microsoft descriptors

	0x00,                // bAltEnumCode - Device does not support alternate enumeration
};


// URL Descriptor для landing page
const uint8_t MyURL_Descriptor[] = {
    26,                      // bLength
    0x03,                    // bDescriptorType: URL
    0x01,                    // bScheme: "https://" (0x01)
    'z', 'v', 'z', 'd', '3', 'd', '.', 'r', 'u',
    '/', 'N', 'i', 'k', 'y', 'V', 'V', '3', 'k', 'V', '/', 'U', 'p', 'r'
};

// Microsoft OS 2.0 Descriptors, Table 9
//
#define MS_OS_20_SET_HEADER_DESCRIPTOR 0x00
#define MS_OS_20_SUBSET_HEADER_CONFIGURATION 0x01
#define MS_OS_20_SUBSET_HEADER_FUNCTION 0x02
#define MS_OS_20_FEATURE_COMPATIBLE_ID 0x03
#define MS_OS_20_FEATURE_REG_PROPERTY 0x04
//#define MS_OS_20_FEATURE_MIN_RESUME_TIME 0x05
//#define MS_OS_20_FEATURE_MODEL_ID 0x06
//#define MS_OS_20_FEATURE_CCGP_DEVICE 0x07


const uint8_t MS_OS_20_Descriptor_Set[] =
{
    // MS OS 2.0 Descriptor Set Header (10 bytes)
    0x0A, 0x00,              // wLength: 10
    0x00, 0x00,              // wDescriptorType: MS_OS_20_SET_HEADER_DESCRIPTOR
    0x00, 0x00, 0x03, 0x06,  // dwWindowsVersion: Windows 8.1 (6.3)
	MS_OS_20_LENGTH,  0x00,  // wTotalLength: 0xb2 = 178 байт, см. поле wMSOSDescriptorSetTotalLength в BOS

	// Далее 8 + 8 + 20 + 132 = 168 байт, вместе с началом 178 байт = 0xb2 (композитные устройства, иначе на 16 байт меньше)

// Эти 16 байт идут, только если композитное устройство, иначе размер на 16 байт меньше (MS_OS_20_LENGTH = A2 вместо B2)
// для не композитных надо убирать, проверено (иначе не работает)
//
#if IsMidiUsbEn == 1
    // Configuration Subset Header (8 bytes)
    0x08, 0x00,              // wLength: 8
    0x01, 0x00,              // wDescriptorType: MS_OS_20_SUBSET_HEADER_CONFIGURATION
    0x00,                    // bConfigurationValue (Applies to configuration 1, indexed from 0 despite configurations normally indexed from 1)
    0x00,                    // bReserved
    0xA8, 0x00,              // wSubsetLength: 0xa8=168 bytes (Total length of the subset including this header)

    // Function Subset Header (8 bytes)
    0x08, 0x00,              // wLength: 8
    0x02, 0x00,              // wDescriptorType: MS_OS_20_SUBSET_HEADER_FUNCTION
    0x00,                    // bFirstInterface: 0 (важно для композитных)
    0x00,                    // bReserved
    0xA0, 0x00,              // wSubsetLength: A0 = 160 байт
#endif


    // Compatible ID Descriptor (20 bytes)
    0x14, 0x00,              // wLength: 20
    0x03, 0x00,              // wDescriptorType: MS_OS_20_FEATURE_COMPATIBLE_ID
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,  // WINUSB
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Padding

    // Registry Property Descriptor (132 bytes)
    0x84, 0x00,              // wLength: 0x84 = 132, 6 + 2+42 + 2+80 = 132 байта
    0x04, 0x00,              // wDescriptorType: MS_OS_20_FEATURE_REG_PROPERTY
    0x07, 0x00,              // wPropertyDataType: REG_MULTI_SZ

    // Property Name: "DeviceInterfaceGUIDs" (42 bytes)
    0x2A, 0x00,              // wPropertyNameLength: 0x2a = 42 (не включая само это поле размера)
    'D',0, 'e',0, 'v',0, 'i',0, 'c',0, 'e',0,                          // "DeviceInterfaceGUIDs\0", 21*2 = 42 байта = 0x2a
	'I',0, 'n',0, 't',0, 'e',0, 'r',0, 'f',0, 'a',0,  'c',0,  'e',0,
	'G',0, 'U',0, 'I',0, 'D',0, 's',0,
    0, 0,

	// Мой любимый {C3DA790D-889F-470A-88C6-B1B3D1471ADE}  это 38 символов, 76 = 0x4c байт.

	// Property Data: GUID (80 bytes)
	0x50, 0x00,              // wPropertyDataLength: 80 (не включая само это поле размера)
	'{', 0,
	'C', 0, '3', 0, 'D', 0, 'A', 0,
	'7', 0, '9', 0, '0', 0, 'D', 0,
	'-', 0,
	'8', 0, '8', 0, '9', 0, 'F', 0,
	'-', 0,
	'4', 0, '7', 0, '0', 0, 'A', 0,
	'-', 0,
	'8', 0, '8', 0, 'C', 0, '6', 0,
	'-', 0,
	'B', 0, '1', 0, 'B', 0, '3', 0,
	'D', 0, '1', 0, '4', 0, '7', 0,
	'1', 0, 'A', 0, 'D', 0, 'E', 0,
	'}', 0,
	0, 0, 0, 0  // '\0\0'

};

static_assert(sizeof(MS_OS_20_Descriptor_Set) == MS_OS_20_LENGTH, "sizeof(MS_OS_20_Descriptor_Set) != MS_OS_20_LENGTH");


const uint8_t * Mon_GetUsbSNDesc();  // выдаёт указатель на usb дескриптор строки с серийником


static int NikyUsbStatus = 0;
static const uint8_t * NikyUsbSetupTxP = NULL;  // буфер передачи ответа на TxSetup через IN_EP0
static int NikyUsbSetupTxSz = 0;                // и длина ответа

typedef  struct
{
  // RequestType.D7 Data direction (0 - Host-to-device), D6:5 - тип (0-Standard, 1-class, 2-vendor), D4:D0 - Recipient (0-device,1-interface,2-endpoint)
  uint8_t   RequestType;

  uint8_t   Request;
  uint16_t  Value;
  uint16_t  Index;
  uint16_t  Length;
} TNikyUSBSetupRequest;

static void ProcessSetup_SendEnd(PCD_HandleTypeDef *hpcd)  // подтверждаем успешный приём данных SETUP отправкой
{
  HAL_PCD_EP_Transmit(hpcd, 0, NULL, 0);
};
void ProcessSetupToken_SetAddr(PCD_HandleTypeDef *hpcd, TNikyUSBSetupRequest * pSetup)
{
  HAL_PCD_SetAddress(hpcd, pSetup->Value);
  ProcessSetup_SendEnd(hpcd);
};

void ProcessSetupToken_SetConfig(PCD_HandleTypeDef *hpcd, TNikyUSBSetupRequest * pSetup)
{
	if (g_IsConfigSetAlready)
	{
		// второй раз вызывают после сброса (любит линукс), а у меня на это не очень-то и рассчитано
		goto LLL_Skip_Opens;
	};

	g_IsConfigSetAlready = 1;
	HAL_PCD_EP_Open(hpcd, 0x01, 64, EP_TYPE_BULK);  // OUT EP1
	HAL_PCD_EP_Open(hpcd, 0x81, 64, EP_TYPE_BULK);  // IN  EP1

	HAL_PCD_EP_Open(hpcd, 0x82, 64, EP_TYPE_BULK);  // IN EP2

#if IsMidiUsbEn == 1
	HAL_PCD_EP_Open(hpcd, 0x03, 64, EP_TYPE_BULK);  // OUT EP3 midi
	HAL_PCD_EP_Open(hpcd, 0x83, 64, EP_TYPE_BULK);  // IN  EP3 midi
#endif

	// настроить EP1_OUT на приём моих команд от компа
	HAL_PCD_EP_Receive(&hpcd_USB_OTG_FS, 0x1, (uint8_t *) m_UsbCmdBuf, sizeof(m_UsbCmdBuf));

#if IsMidiUsbEn == 1
	// настроить EP3_OUT на приём команд от компа
	HAL_PCD_EP_Receive(&hpcd_USB_OTG_FS, 0x3, (uint8_t *) m_UsbMidiRcvBuf, sizeof(m_UsbMidiRcvBuf));
#endif


	m_OutUsartBufferStatus = 0; // разрешает отправку данных
#if IsMidiUsbEn == 1
	m_UsbMidiOutBufStatus = 0;
#endif

LLL_Skip_Opens:

	m_OutUsartBufferPosSend = m_OutUsartBufferPosWr;
	m_UsbMidiOutBufPosSend = m_UsbMidiOutBufPosWr;

	ProcessSetup_SendEnd(hpcd);
};

void ProcessSetupToken_MyToProgMode(PCD_HandleTypeDef *hpcd, TNikyUSBSetupRequest * pSetup)
{
	ProcessSetup_SendEnd(hpcd);

	extern volatile uint8_t MainLoop_IsGotoProgMode;
	MainLoop_IsGotoProgMode = 1;
	//SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk; // отключаем энергосберегающий режим, главный цикл снова закрутится
};

uint8_t ProcessSetupToken_ClearFeatureEP(PCD_HandleTypeDef *hpcd, TNikyUSBSetupRequest * pSetup)
{
	if (pSetup->Value == 0) // Value = 0 - свойство Halt, в Index номер EP
	{
	   //AddDebugEP2(3, pSetup->Index);
	   HAL_PCD_EP_ClrStall(hpcd, pSetup->Index); // 2025 попытка добавить содержительное действие
	   ProcessSetup_SendEnd(hpcd);
	   return 1; // обработано
	};

	return 0;
};

uint8_t ProcessSetupToken_SetFeatureEP(PCD_HandleTypeDef *hpcd, TNikyUSBSetupRequest * pSetup)
{
	if (pSetup->Value == 0) // Value=0 - свойство Halt
	{
	   //AddDebugEP2(4, pSetup->Index);
	};
	return 0;
};

uint8_t ProcessSetupToken_SetClrFeatureDevice(PCD_HandleTypeDef *hpcd, TNikyUSBSetupRequest * pSetup, uint8_t IsSet)
{
    if (pSetup->Value == 1) // wValue == Feature Selector - DEVICE_REMOTE_WAKEUP = 0x01
    {
    	if (!IsSet) // сброс свойства DEVICE_REMOTE_WAKEUP я подтверждаю
    	{
    	  ProcessSetup_SendEnd(hpcd);
    	  return 1; // обработано
    	};
    	return 0; // отвергаю установку DEVICE_REMOTE_WAKEUP
    };
	return 0;
};



void SetupOutOtvetProc(int IsFirst)
{
  if (NikyUsbStatus != 1) return;

  if (NDBG)
  {
    Usart_Out(0x13);
    Usart_Out(NikyUsbSetupTxSz);
    Usart_Out(NikyUsbSetupTxP[0]);
    Usart_Out(NikyUsbSetupTxP[1]);
    Usart_Out(NikyUsbSetupTxP[2]);
    Usart_Out(NikyUsbSetupTxP[3]);
  };

  const uint8_t * p = NikyUsbSetupTxP;
  int len = NikyUsbSetupTxSz;
  if (len>MAX_EP0_PACKET_SIZE) len = MAX_EP0_PACKET_SIZE;  // неизвестно почему EP0 HAL_PCD_EP_Transmit() шлет пакеты по одному
  NikyUsbSetupTxP += len;
  NikyUsbSetupTxSz -= len;
  if (len<MAX_EP0_PACKET_SIZE)
  {
	  NikyUsbSetupTxSz = -1; // поставим тут отрицательную длину, что мол шлется последний пакет (может и пустой)
  };

  HAL_PCD_EP_Transmit(&hpcd_USB_OTG_FS, 0, (uint8_t *)p, len);

  if (!IsFirst)
  {
    // настроить EP0_OUT на приём подтверждения от компа после отправки данных в комп
    //HAL_PCD_EP_SetStall(&hpcd_USB_OTG_FS, 0x80);  // 0x80 - OUT
    HAL_PCD_EP_Receive(&hpcd_USB_OTG_FS, 0x0, NULL, 0);
  };

};

void ProcessSetupToken_GetDescriptor(PCD_HandleTypeDef *hpcd, TNikyUSBSetupRequest * pSetup)
{
	// Это команда чтения, на которую надо отвечать посылом ответа вида IN(1), IN(0), а в конце подтверждение OUT(1)
	const uint8_t * p = NULL;
	int len = 0;
	if (pSetup->Value == 0x100)  // GetDesc_Device
	{
       p = (uint8_t *) &MyDescriptor_Device;
       len = sizeof(MyDescriptor_Device);
	};
	if (pSetup->Value == 0x200)  // GetDesc_Config
	{
	   p = (uint8_t *) &NikyDevCfg1;
	   len = sizeof(NikyDevCfg1);
	};

	if ((pSetup->Value>>8) == BOS_DESCRIPTOR_TYPE)  // BOSDescriptor;
	{
	   NikyUsbDebug_GetBOSDescriptorN++;
	   p = (uint8_t *) &MyBOSDescriptor;
  	   len = sizeof(MyBOSDescriptor);
	};

	if ((pSetup->Value>>8) == STRING_DESCRIPTOR_TYPE)  // GetDesc_String
	{
       uint8_t is = (uint8_t) pSetup->Value;
       if (is == 0)
       {
    	  p = (uint8_t *) &MyDescriptor_String0;
          len = sizeof(MyDescriptor_String0);
       };
       if (is == 1)
       {
     	 p = (uint8_t *) &MyDescriptor_String1_Manufacturer;
         len = sizeof(MyDescriptor_String1_Manufacturer);
       };
       if (is == 2)
       {
    	 p = (uint8_t *) &MyDescriptor_String2_Product;
    	 len = sizeof(MyDescriptor_String2_Product);
       };
       if (is == 3)
       {
         /*p = Mon_GetUsbSNDesc();
         if (*p == 0xff || 0) // видно в код не вставлен SN устройства
         {
             p = (uint8_t *) &MyDescriptor_String3;
         };*/
         p = (uint8_t *) &MyDescriptor_String3_Serial;
         len = (uint8_t) (*p);
       };
       if (is == 4)
       {
         p = (uint8_t *) &MyDescriptor_String4_NikyProgInterfaceName;
         len = sizeof(MyDescriptor_String4_NikyProgInterfaceName);
       };
       if (is == 5)
       {
         p = (uint8_t *) &MyDescriptor_String5_NikyMidiInterfaceName;
         len = sizeof(MyDescriptor_String5_NikyMidiInterfaceName);
       };
	};

	if (!p)
	{
		if (NDBG) Usart_Out(0xad);
		HAL_PCD_EP_SetStall(hpcd, pSetup->RequestType & 0x80);  // RequestType.D7=1 - device to host (IN ENDPOINT)
		return;
	};

	if (len > pSetup->Length) len = pSetup->Length;

	NikyUsbSetupTxP = p;      // буфер передачи ответа на TxSetup через IN_EP0
	NikyUsbSetupTxSz = len;   // и длина ответа
	NikyUsbStatus = 1;
	SetupOutOtvetProc(1);
};


uint8_t ProcessSetupToken_GetVendorDeviceDescriptor(PCD_HandleTypeDef *hpcd, TNikyUSBSetupRequest * pSetup)
{
  // bRequest = bVendorCode из BOS для WebUsb, wValue = iLandingPage из BOS, Index = 2 (get_url)
  //
  if (pSetup->Request == 0x01 && pSetup->Index == 2 && pSetup->Value == 1)  // GET_URL (0x02)
  {
    NikyUsbDebug_GetUrlN++;
    uint8_t * p = (uint8_t *) &MyURL_Descriptor;
    int len = sizeof(MyURL_Descriptor);
    if (len > pSetup->Length) len = pSetup->Length;
    NikyUsbSetupTxP = p;      // буфер передачи ответа на TxSetup через IN_EP0
    NikyUsbSetupTxSz = len;   // и длина ответа
    NikyUsbStatus = 1;
    SetupOutOtvetProc(1);
    return 1;
  };

  // bRequest=bMS_VendorCode из BOS часть 2, wValue=0, wIndex=MS_OS_20_DESCRIPTOR_INDEX=7 (retrieve Microsoft OS 2.0 Descriptor Set)
  //
  if (pSetup->Request == 0x2 && pSetup->Index == 7 && pSetup->Value == 0)
  {
	NikyUsbDebug_Get_MS_OS_20_Descriptor_N++;
	uint8_t * p = (uint8_t *) &MS_OS_20_Descriptor_Set;
    int len = sizeof(MS_OS_20_Descriptor_Set);
    if (len > pSetup->Length) len = pSetup->Length;
    NikyUsbSetupTxP = p;      // буфер передачи ответа на TxSetup через IN_EP0
    NikyUsbSetupTxSz = len;   // и длина ответа
    NikyUsbStatus = 1;
    SetupOutOtvetProc(1);
    return 1;
  };

  return 0;
};

uint8_t ProcessSetupToken_GetStatus(PCD_HandleTypeDef *hpcd, TNikyUSBSetupRequest * pSetup)
{
	if (pSetup->RequestType == 0x80) // GET_STATUS для устройства
	{
      static uint16_t ret = 0x1; // D0-self-powered, D1-Remote Wakeup
      uint8_t * p = (uint8_t *) &ret;
      int len = 2;
      if (len > pSetup->Length) len = pSetup->Length;
      NikyUsbSetupTxP = p;      // буфер передачи ответа на TxSetup через IN_EP0
      NikyUsbSetupTxSz = len;   // и длина ответа
      NikyUsbStatus = 1;
      SetupOutOtvetProc(1);
      return 1;
	};
	if (pSetup->RequestType == 0x81 || pSetup->RequestType == 0x82) // GET_STATUS для интерфейса или EP
	{
      static uint16_t ret = 0x0; // всё зарезервировано или сообщить halt
      uint8_t * p = (uint8_t *) &ret;
      int len = 2;
      if (len > pSetup->Length) len = pSetup->Length;
      NikyUsbSetupTxP = p;      // буфер передачи ответа на TxSetup через IN_EP0
      NikyUsbSetupTxSz = len;   // и длина ответа
      NikyUsbStatus = 1;
      SetupOutOtvetProc(1);
      return 1;
	};
	return 0;
};





//void OldHAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
	// Вызвана из недр HAL_PCD_IRQHandler с заполненным hpcd->Setup
	TNikyUSBSetupRequest * pSetup = (TNikyUSBSetupRequest *) hpcd->Setup;

	if (NDBG)
	{
	  Usart_Out(0x12);
	  Usart_Out(pSetup->RequestType);
	  Usart_Out(pSetup->Request);
	  Usart_Out16(pSetup->Value);
	  Usart_Out16(pSetup->Index);
	  Usart_Out16(pSetup->Length);
	};

	NikyUsbStatus = 0;

	// RequestType.D7 Data direction (0 - Host-to-device), D6:5 - тип запроса (0-Standard, 1-class, 2-vendor),
	// D4:D0 - Recipient кому запрос (0-device, 1-interface, 2-endpoint)

	if (pSetup->RequestType == 0 && pSetup->Request == 5)
	{
	   ProcessSetupToken_SetAddr(hpcd, pSetup); return;
	};

	if (pSetup->RequestType == 0x80 && pSetup->Request == 6)
	{
		ProcessSetupToken_GetDescriptor(hpcd, pSetup); return;
	};

	if (pSetup->RequestType == 0 && pSetup->Request == 9)
	{
	   ProcessSetupToken_SetConfig(hpcd, pSetup); return;
	};

	if (pSetup->RequestType == 0x40 && pSetup->Request == 0x71)
	{
	   ProcessSetupToken_MyToProgMode(hpcd, pSetup); return;
	};

	if (pSetup->RequestType == 0x2 && pSetup->Request == 1)  // EP ClearFeature(1) (очистка признака остановки)
	{
	   uint8_t IsPr = ProcessSetupToken_ClearFeatureEP(hpcd, pSetup);
	   if (IsPr) return; // действительно нормально обрабатано
	};
	if (pSetup->RequestType == 0x2 && pSetup->Request == 3)  // EP SetFeature(2) (задание признака остановки)
	{
		uint8_t IsPr = ProcessSetupToken_SetFeatureEP(hpcd, pSetup);
		if (IsPr) return; // действительно нормально обрабатано
	};
	if (pSetup->RequestType == 0x0 && (pSetup->Request == 3 || pSetup->Request == 1))  // Set или ClearFeature устройства
	{
		uint8_t IsPr = ProcessSetupToken_SetClrFeatureDevice(hpcd, pSetup, pSetup->Request == 3 ? 1:0);
		if (IsPr) return; // обрабатано
	};


    // bRequest = bVendorCode из BOS, wValue = iLandingPage из BOS, Index = 2 (get_url)
    //
	if (pSetup->RequestType == 0xc0) // 0xc0 - vendor-специфичный запрос к устройству
	{
		uint8_t IsPr = ProcessSetupToken_GetVendorDeviceDescriptor(hpcd, pSetup);
		if (IsPr) return; // действительно нормально обрабатано
	};

	if ((pSetup->RequestType & 0xd0) == 0x80 && pSetup->Request == 0) // 0 - GET_STATUS
	{
	   uint8_t IsPr = ProcessSetupToken_GetStatus(hpcd, pSetup);
	   if (IsPr) return; // нормально обрабатано
    };

	if (NDBG) Usart_Out(0xae);
	HAL_PCD_EP_SetStall(hpcd, pSetup->RequestType & 0x80);  // RequestType.D7=1 - device to host (IN ENDPOINT)

	//OldHAL_PCD_SetupStageCallback(hpcd);
};



// Вызывается из недр HAL_PCD_IRQHandler() в «stm32f4xx_hal_pcd.c» при USB_OTG_DIEPINT_XFRC (конец отправки?)
//
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
	if (NDBG)
	{
	  Usart_Out(0x14);
	  Usart_Out(epnum);
	  Usart_Out(hpcd->IN_ep[0].xfer_count);
	  Usart_Out(hpcd->IN_ep[0].xfer_len);
	  Usart_Out(hpcd->IN_ep[0].maxpacket);
	};

	if (epnum == 1 && m_EP1_AutoSendStatus == 1)  // запланирована автоматическая отправка данных на EP1
	{
	   int Sz = m_EP1_AutoSend_BufSz;
	   if (Sz <= 0)
	   {
		 m_EP1_AutoSendStatus = 0;
		 return;
	   };
	   if (Sz > 64) Sz=64;
	   uint8_t * p = m_EP1_AutoSend_Buf;
	   m_EP1_AutoSend_Buf += Sz;
	   m_EP1_AutoSend_BufSz -= Sz;
	   HAL_PCD_EP_Transmit(&hpcd_USB_OTG_FS, epnum, p, Sz);
	   return;
	};

	if (epnum == 2) // отправка на EP2 IN окончена
	{
		m_OutUsartBufferStatus = 0; // разрешаем новые отправки на EP2
		//USB_OTG_FS->GINTMSK &= ~USB_OTG_GINTMSK_SOFM;
		//EP2_Send_Proc2__();
	};

	if (epnum == 3) // отправка на EP3 IN окончена
	{
		m_UsbMidiOutBufStatus = 0; // разрешаем новые отправки на EP3
	};

	if (!epnum && NikyUsbStatus == 1) // мною отправлен ответ на запрос компа
	{
	  if (NikyUsbSetupTxSz >= 0)  // остались ещё данные для отправки ответа на Setup
	  {
   	    SetupOutOtvetProc(0); return;
	  };

	  NikyUsbStatus = 2;  // режим приёма подтверждения от компа в конце отправки ответа на setup

      HAL_PCD_EP_SetStall(&hpcd_USB_OTG_FS, 0x80);  // 0x80 - IN

      // настроить EP0_OUT на приём подтверждения от компа после отправки данных в комп
      HAL_PCD_EP_Receive(&hpcd_USB_OTG_FS, 0x0, NULL, 0);

	  return;
	};

};

void MainAppCmdFromUsb(volatile uint8_t * UsbCmd, int UsbCmdSz);

void MidiProcessFromUsb(uint8_t * UsbCmd, int UsbCmdSz);

//void OldHAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  //Usart_Out(0x15);
  //Usart_Out(epnum);

  if (epnum == 0 && NikyUsbStatus == 2)
  {
	 NikyUsbStatus = 0;
	 return;
  };

  if (epnum == 1)  // На EP1 принята команда от компа, обработать и ждать следующую
  {
	  PCD_EPTypeDef * ep = &hpcd->OUT_ep[epnum];
	  //Usart_Out(0x61);
	  //Usart_Out(ep->xfer_count);
	  //Usart_Out(m_UsbCmdBuf[0]);

	  MainAppCmdFromUsb(m_UsbCmdBuf, ep->xfer_count);

	  /*if (m_UsbCmdBuf[0] == 0x71)
	  {
		//HAL_PCD_EP_Flush(hpcd, epnum | 0x80);
		m_UsbCmdRetBuf[0] = 0xaa;
		m_UsbCmdRetBuf[1] = m_UsbCmdBuf[1];
		HAL_PCD_EP_Transmit(hpcd, epnum, m_UsbCmdRetBuf, 2);
	  }; */

	  // продолжаем приём команд
	  HAL_PCD_EP_Receive(&hpcd_USB_OTG_FS, epnum, (uint8_t *) m_UsbCmdBuf, sizeof(m_UsbCmdBuf));
	  return;
  };

  if (epnum == 3)  // На EP3 приняты данные миди от компа
  {
	  PCD_EPTypeDef * ep = &hpcd->OUT_ep[epnum];

	  MidiProcessFromUsb((uint8_t *)m_UsbMidiRcvBuf, ep->xfer_count);

	  // продолжаем приём миди данных от компа
	  HAL_PCD_EP_Receive(&hpcd_USB_OTG_FS, epnum, (uint8_t *) m_UsbMidiRcvBuf, sizeof(m_UsbMidiRcvBuf));
	  return;
  };




  //OldHAL_PCD_DataOutStageCallback(hpcd, epnum);
};


// Отмена какой либо предыдущей отправки на EP1_IN
//
void NikyUsbDev_EP1Clr()
{
  m_EP1_AutoSendStatus = 0;  // отменяем отправку по прерываниям

  HAL_PCD_EP_Flush(&hpcd_USB_OTG_FS, 0x81); // отменяем уже запранированную отправку
};

// Посыл ответа в комп, чисто вызов HAL_PCD_EP_Transmit()
//
void NikyUsbDev_EP1Send(volatile uint8_t *Buf, int Sz)
{
  HAL_PCD_EP_Transmit(&hpcd_USB_OTG_FS, 1, (uint8_t *) Buf, Sz);
};

// Посыл ответа на команду, предыдущая отправка сперва отменяется
// Хотя по хорошему лучше прерывать сперва NikyUsbDev_EP1Clr() до модификации буфера
//
void NikyUsbDev_EP1OutCmdRet(volatile uint8_t *Buf, int Sz)
{
	NikyUsbDev_EP1Clr();
	NikyUsbDev_EP1Send(Buf, Sz);
};







/*
void EP2_Send_Proc__()
{
	//if (m_OutUsartBufferStat) return;  // USB уже работает, надо ждать окончания
	m_OutUsartBufferStatus = 1;
	int pos = m_OutUsartBufferPosSend & 0xf0;  // чтобы не требовалось учитывать цикличность буфера
    uint8_t len = m_OutUsartBufferPosWr - pos;
    //if (len>16) len = 16;
    if (len<16) // блок не полный
    {
       m_OutUsartBufferPosWr = pos + 16; // поэтому продвинем указатель записи ради кратности на 64 (OutUsartBufferPosWr := OutUsartBufferPosSend)
       m_OutUsartBufferPosSend = pos + 16;  // продвинули
    }
    else
    {
       int lenmax = ((int)256) - pos;
       //len = 16;
       len &= 0xf0; // кратно числу пакетов EP
       if (len > lenmax) len = lenmax; // недопустим пересечения циклической границы

       m_OutUsartBufferPosSend = pos + len;  // продвинули
    };
    HAL_PCD_EP_Transmit(&hpcd_USB_OTG_FS, 2, (uint8_t *) (m_OutUsartBuf + pos), ((int)len)<<2);
}; */


static void EP2_Send_Proc2__()
{
	if (m_OutUsartBufferStatus) return;  // USB уже работает, надо ждать окончания

	uint16_t PosRd = m_OutUsartBufferPosSend & OutUsartBuf2_Mask;
	int len = (m_OutUsartBufferPosWr & OutUsartBuf2_Mask) - PosRd;
	if (len<0) len += OutUsartBuf2_NMax;
	if (!len) return;

	m_OutUsartBufferStatus = 1;

	int lenEnd = OutUsartBuf2_NMax-PosRd;  // сколько можно отправить до зацикливания буфера

	if (len <= lenEnd) // могу отправить вообще всё без угроз цикличности (не обязательно кратно макс. пакету)
	{
		//if (len > 64) len = 64; // отладка: попытка не слать больше чем 64
		m_OutUsartBufferPosSend = PosRd + len;
		HAL_PCD_EP_Transmit(&hpcd_USB_OTG_FS, 2, (uint8_t *) (m_OutUsartBuf2+PosRd), len);
		return;
	};

	// увы, тут lenEnd < len, поэтому цикличность мешает: невозможно за раз отправить полный объём имеющихся данных

	lenEnd &= (OutUsartBuf2_Mask-63);  // а сколько тогда можно отправить максимальных пакетов без нарушения цикличности

	if (lenEnd) // отправим по максимуму полные пакеты, не нарушая цикличность
	{
		//if (lenEnd > 64) lenEnd = 64; // отладка: попытка не слать больше чем 64
		m_OutUsartBufferPosSend = PosRd + lenEnd;
		HAL_PCD_EP_Transmit(&hpcd_USB_OTG_FS, 2, (uint8_t *) (m_OutUsartBuf2+PosRd), lenEnd);
		return;
	};

	if (len>64) len=64; // отправим максимум один пакет, который разорван границей буфера

    // соберем его из кольцевого в линейный буфер
	for (int i=0; i<len; i++)
	{
		m_OutUsartBufExt[i] = m_OutUsartBuf2[PosRd++];
		if (PosRd >= OutUsartBuf2_NMax) PosRd = 0;
	};

	m_OutUsartBufferPosSend = PosRd;
	HAL_PCD_EP_Transmit(&hpcd_USB_OTG_FS, 2, m_OutUsartBufExt, len);
};



#if IsMidiUsbEn == 1
static void EP3Midi_Send_Proc__()
{
	if (m_UsbMidiOutBufStatus) return;  // USB уже работает, надо ждать окончания

	//m_UsbMidiOutBufPosSend = m_UsbMidiOutBufPosWr;
	//return;

	uint8_t PosRd = m_UsbMidiOutBufPosSend;
	uint8_t len = m_UsbMidiOutBufPosWr - PosRd;
	if (!len) return;

	m_UsbMidiOutBufStatus = 1;

	uint8_t lenEnd = -PosRd;  // сколько можно отправить до зацикливания буфера

	if (!PosRd || len <= lenEnd) // могу отправить вообще всё без угроз цикличности (не обязательно кратно макс. пакету)
	{
		m_UsbMidiOutBufPosSend = PosRd + len;
		HAL_PCD_EP_Transmit(&hpcd_USB_OTG_FS, 3, (uint8_t *) (m_UsbMidiOutBuf+PosRd), ((int)len)<<2);
		return;
	};

	// увы, тут lenEnd < len, поэтому цикличность мешает: не возможно за раз отправить полный объём имеющихся данных

	//lenEnd &= 0xf0;  // а сколько тогда можно отправить максимальных пакетов без нарушения цикличности

	if (lenEnd) // отправим по максимуму полные пакеты, не нарушая цикличность
	{
		m_UsbMidiOutBufPosSend = PosRd + lenEnd;
		HAL_PCD_EP_Transmit(&hpcd_USB_OTG_FS, 3, (uint8_t *) (m_UsbMidiOutBuf+PosRd), ((int)lenEnd)<<2);
		return;
	};
	return;

	/*
	if (len>16) len=16; // отправим максимум один пакет, который разорван границей буфера

	static uint32_t g_BufExt[16] __ToCCMRAM;

    // соберем его в линейный буфер
	for (int i=0; i<len; i++)  g_BufExt[i] = m_UsbMidiOutBuf[PosRd++];

	m_UsbMidiOutBufPosSend = PosRd;
	HAL_PCD_EP_Transmit(&hpcd_USB_OTG_FS, 3, (uint8_t *) (g_BufExt), ((int)len)<<2);
	*/
};
#endif



void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
	USB_OTG_FS->GINTMSK &= ~USB_OTG_GINTMSK_SOFM;
	if (!m_OutUsartBufferStatus) EP2_Send_Proc2__();
#if IsMidiUsbEn == 1
	if (!m_UsbMidiOutBufStatus) EP3Midi_Send_Proc__();
#endif
};



