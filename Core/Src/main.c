/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  https://zvzd3d.ru/NikyVV3kV/
  Для Журнала Радио 2025.12.06
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "NikyUsbDev.h"
//#include "UsartMidi.h"
#include "Oled96.h"
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_CRC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct
{
	  int CurFM;
	  int DiaFM1;
	  int DiaFM2;

	  uint8_t IsOutOn;

	  // режим сканирования при старте МК. Не тоже самое, что текущий режим (который может прекратиться)
	  // биты 0-3 - режим сканирования; 4-6 - критерий поиска (исп. только в режиме сканирования или долгом нажатии на кнопку)
	  //
	  uint8_t StartScanMode;

	  uint8_t IsOffOnScan;
	  uint8_t Signature;   // 55 если норма из flash, иначе надо по умолчанию

	  float Cal_VV_RH;
	  float Cal_VV_RL;
	  float Cal_VV_ShuntR;

	  float Cal_Upit_RH;
	  float Cal_Upit_RL;
	  float Cal_Ipit_Chui;
	  float Cal_Ipit_Zero;

	  // 1 - использовать экран, 2 - переворот, 4 - средние вместо пиковых, 8 - показ тока (acs712 подкл), 16 - погасить led на PC13
	  // 32 - инверсия яркости
	  uint8_t OledOptions;
	  uint8_t Cal_VV_IsFixZeroAdcCode;
	  uint16_t OffTime10MKS; // Длительность отключения в единицах по 10 мкс

	  float Cal_VV_ZeroAdcCode; // если Cal_VV_IsFixZeroAdcCode, то это код для ноля тока


} TProgOptions;

volatile TProgOptions g_VVInfo;

void CalFromVVParams();

void VVInfoInitDef(volatile TProgOptions * const p)
{
	// Делители не должны быть кратны 61, иначе на будет сверхразрешения
    //
	p->CurFM = 2100;   // 84e6 / 2100 = 40 кгц
	p->DiaFM1 = 4200;  // 84e6 / 4200 = 20 кгц
	p->DiaFM2 = 420;   // 84e6 / 420 = 200 кгц

	p->IsOutOn = 0xff;   // выход активен
	p->StartScanMode = 0;     // сканирования нет
	p->Signature = 0xff;

	p->IsOffOnScan = 0;  // и не выключать выход
	p->OffTime10MKS = 300; // в единицах по 10 мкс

	 // коэффициенты калибровки для номиналов деталей, указанных на схеме

	//p->Cal_CodeToVV_U = 3.3 / 4095 * (10000+4.7) / 4.7;  // делитель 4.7 пФ : 10000 пФ
	//p->Cal_CodeToVV_I = 3.3 / 4095 / 120 * 1000;         // шунт 120 Ом

	//p->Cal_PitCodeToU = 3.3 / 4095 * (620+2400) / 620;   // делитель 2400 : 620 Ом
	//p->Cal_PitCodeToI = -3.3 / 4095 * 1000 / 185;        // чуйка датчика тока 185 мв на 1 Ампер, при росте тока напряжение должно убывать
	//p->Cal_PitCodeOfsI = 0;

	p->Cal_VV_RH = 10000;
	p->Cal_VV_RL = 4.7;
	p->Cal_VV_ShuntR = 120;

	p->Cal_Upit_RH = 2700;
	p->Cal_Upit_RL = 680;
	p->Cal_Ipit_Chui = 185;
	p->Cal_Ipit_Zero = 0;

	p->OledOptions = 1 + 8;

	p->Cal_VV_IsFixZeroAdcCode = 0;
	p->Cal_VV_ZeroAdcCode = 2048;

	CalFromVVParams();
};

void VVInfoTestKorrect(volatile TProgOptions * const p) // Защита от дурака
{
    if (p->CurFM < 240) p->CurFM = 240;
    if (p->CurFM > 5600) p->CurFM = 5600;
    if (p->CurFM % 61 == 0) p->CurFM += 2;

    if (p->DiaFM1 < 240) p->DiaFM1 = 240;
    if (p->DiaFM1 > 5600) p->DiaFM1 = 5600;
    if (p->DiaFM1 % 61 == 0) p->DiaFM1 += 2;

    if (p->DiaFM2 < 240) p->DiaFM2 = 240;
    if (p->DiaFM2 > 5600) p->DiaFM2 = 5600;
    if (p->DiaFM2 % 61 == 0) p->DiaFM2 += 2;

    if (p->DiaFM1 < p->DiaFM2)
    {
      int tmp = p->DiaFM1; p->DiaFM1 = p->DiaFM2; p->DiaFM2 = tmp;
    };

    uint8_t mode = p->StartScanMode & 15;
    if (mode > 3) mode = 0;
    uint8_t kriteriy = (p->StartScanMode >> 4) & 7;
    if (kriteriy > 4) kriteriy = 0;
    p->StartScanMode = (p->StartScanMode & 0x80) | (kriteriy<<4) | mode;

    if (!(p->Cal_VV_ZeroAdcCode>=0 && p->Cal_VV_ZeroAdcCode<=4096)) p->Cal_VV_ZeroAdcCode = 2048;
};

/*void VVInfoReadFromFlash(volatile TProgOptions * const p)
{
	  // 128 кб под прошивку, 128 кб под конфигурацию

	  uint8_t * pRd = (uint8_t *) 0x08020000;
	  if (*pRd++ != 0x5e) return;
	  if (*pRd++ != 0x81) return;
	  if (*pRd++ != 0x3c) return;
	  if (*pRd++ != 0xd9) return;

      *p = * (TProgOptions *) pRd; // берём настрйоки из flash памяти

      VVInfoTestKorrect(p);
};*/

float Cal_CodeToVV_U;
float Cal_CodeToVV_I;

float Cal_PitCodeToU;
float Cal_PitCodeToI;
float Cal_PitCodeOfsI;

void CalFromVVParams()
{
	Cal_CodeToVV_U = 3.3 / 4095 * (g_VVInfo.Cal_VV_RH+g_VVInfo.Cal_VV_RL) / g_VVInfo.Cal_VV_RL;
	Cal_CodeToVV_I = 3.3 / 4095 * 1000 / g_VVInfo.Cal_VV_ShuntR;

	Cal_PitCodeToI = -3.3 / 4095 * 1000 / g_VVInfo.Cal_Ipit_Chui;
	Cal_PitCodeOfsI = 2.5 * 1000 / g_VVInfo.Cal_Ipit_Chui - g_VVInfo.Cal_Ipit_Zero;

	Cal_PitCodeToU = 3.3 / 4095 * (g_VVInfo.Cal_Upit_RH + g_VVInfo.Cal_Upit_RL) / g_VVInfo.Cal_Upit_RL;
};

void OledShowVV(float VV_PikI, float VV_PikV, float VV_Pwr, int OutPeriod, int PereFlags);


static inline void W25SelectProc(uint8_t CS)
{
  /*if (CS & 1) LL_GPIO_SetOutputPin(NikyCS0_GPIO_Port, NikyCS0_Pin);
         else LL_GPIO_ResetOutputPin(NikyCS0_GPIO_Port, NikyCS0_Pin);
  if (CS & 2) LL_GPIO_SetOutputPin(NikyCS1_GPIO_Port, NikyCS1_Pin);
    	 else LL_GPIO_ResetOutputPin(NikyCS1_GPIO_Port, NikyCS1_Pin);
  //if (CS & 4) LL_GPIO_SetOutputPin(NikyCS2_GPIO_Port, NikyCS2_Pin);
  //  	 else LL_GPIO_ResetOutputPin(NikyCS2_GPIO_Port, NikyCS2_Pin);
   */

  GPIOA->ODR = (GPIOA->ODR & (0xffff-0x3f)) | (CS & 0x3f);

  const uint32_t t0 = TIM2->CNT;
  while (TIM2->CNT - t0 < 8) continue;
};

static inline void W25Deselect()
{
  /*LL_GPIO_SetOutputPin(NikyCS0_GPIO_Port, NikyCS0_Pin);
  LL_GPIO_SetOutputPin(NikyCS1_GPIO_Port, NikyCS1_Pin);
  //LL_GPIO_SetOutputPin(NikyCS2_GPIO_Port, NikyCS2_Pin);
   *
   */

  GPIOA->ODR = GPIOA->ODR | 0x3f;

  const uint32_t t0 = TIM2->CNT;
  while (TIM2->CNT - t0 < 8) continue;
};




inline static uint8_t SSP_WriteRead(uint8_t data)
{
	while (!(SPI1->SR & SPI_SR_TXE)) ;   // SR.TXE уст, если Tx buffer is empty (и можно писать)
	*((volatile uint8_t*)&SPI1->DR) = data;
	while (!(SPI1->SR & SPI_SR_RXNE)) ;  // SR.RXNE уст, если Rx buffer is not empty (и можно читать)
	return (uint8_t) SPI1->DR;
};
inline static void SSP_Write(uint8_t data)
{
	while (!(SPI1->SR & SPI_SR_TXE)) ; // SR.TXE уст, если Tx buffer is empty (и можно писать)
	*((volatile uint8_t*)&SPI1->DR) = data;
};
inline static volatile uint8_t SSP_Read()
{
	while (!(SPI1->SR & SPI_SR_RXNE)) ;  // SR.RXNE уст, если Rx buffer is not empty (и можно читать)
	return (uint8_t) SPI1->DR;
};

#define StartAdcType_VV_I  (1)
#define StartAdcType_VV_U  (2)
#define StartAdcType_VV_IU (3)
#define StartAdcType_U   (128)
#define StartAdcType_I   (129)

int g_AdcDmaIsWork = 0;
uint32_t g_AdcStartT;
uint32_t g_AdcEndT;
uint16_t g_AdcUseLen = 0;

uint16_t g_Adc1ResBuf[16384+8192];
uint32_t g_Adc1Time;
uint32_t g_Adc1Len;
int g_Adc1OutPeriod;
uint16_t g_Adc2ResBuf[128*4];

volatile int g_IsMustStartIzmVV = 0;
volatile uint8_t g_IsMustLockVV = 0;
//volatile int g_AutoScanMode = 0;
volatile int ScanMode = 0;
volatile float FindValue = 0;
volatile int FindOutPeriod = 0;
volatile int g_ScanDir = 1;





//volatile uint8_t g_OffScanMode = 0;


void StartAdc(int Type, uint16_t AdcLen) // AdcLen - кол-во 16-битных выборок
{
	g_AdcDmaIsWork = 1;

	LL_ADC_Enable(ADC1);

	int GroupNminus1 = 0;  // число каналов в группе - 1
	if (Type == StartAdcType_VV_IU) GroupNminus1 = 1;

	uint32_t scan_en = 0;
	if (GroupNminus1 & 1) scan_en |= ADC_SQR1_L_0;
	if (GroupNminus1 & 2) scan_en |= ADC_SQR1_L_1;
	if (GroupNminus1 & 4) scan_en |= ADC_SQR1_L_2;
	if (GroupNminus1 & 8) scan_en |= ADC_SQR1_L_3;
	LL_ADC_REG_SetSequencerLength(ADC1, scan_en); // LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS);

	if (Type == StartAdcType_VV_I)	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
	if (Type == StartAdcType_VV_U)	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_2);
	if (Type == StartAdcType_U)	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_6);
	if (Type == StartAdcType_I)	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_7);

	if (Type == StartAdcType_VV_IU)
	{
		LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
        LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_2);
	};


	/*
	int aa = LL_ADC_SAMPLINGTIME_3CYCLES; //LL_ADC_SAMPLINGTIME_15CYCLES; //LL_ADC_SAMPLINGTIME_84CYCLES;
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, aa);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, aa);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, aa);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3, aa);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, aa);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, aa);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6, aa);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7, aa);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_8, aa);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_9, aa);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_10, aa);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_11, aa);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_12, aa);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_13, aa);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_14, aa);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_15, aa);
    */

    //LL_DMA_ClearFlag_HT4(DMA2);  // Clear Stream half transfer flag
    //LL_DMA_ClearFlag_TC4(DMA2);  // Clear Stream transfer complete flag
    //LL_DMA_ClearFlag_TE4(DMA2);  // Clear Stream transfer error flag
    //LL_DMA_ClearFlag_DME4(DMA2); // Clear Stream direct mode error flag.
    //LL_DMA_ClearFlag_FE4(DMA2);  // Clear Stream FIFO error flag.

    DMA2->HIFCR |= DMA_HIFCR_CFEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CTEIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTCIF4;

    //LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_4, (uint32_t)&ADC1->DR, (uint32_t)(&g_Adc1ResBuf), LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    //LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_4, sizeof(g_Adc1ResBuf)/2);

    void * Adr = &g_Adc1ResBuf;
    //int Len = sizeof(g_Adc1ResBuf)/2; // NDTR - в элементах периферии
    if (Type >= 128)
    {
      Adr = &g_Adc2ResBuf;
      //Len = sizeof(g_Adc2ResBuf)/2;
    };

    uint32_t Len = AdcLen;
	g_AdcUseLen = AdcLen;

    DMA2_Stream4->PAR = (uint32_t)&ADC1->DR;
    DMA2_Stream4->M0AR = (uint32_t)(Adr);
    DMA2_Stream4->M1AR = (uint32_t)(Adr);  // для двойной буферизации
    DMA2_Stream4->NDTR = Len;              // NDTR - в элементах периферии
    //DMA2_Stream4->CR |= DMA_SxCR_DBM;
    //DMA2_Stream4->CR &= ~DMA_SxCR_CT;  // текущий буфер нулевой для двойной буферизации


    // см. также ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED, ещё есть
    // LL_ADC_REG_DMA_TRANSFER_LIMITED    (              ADC_CR2_DMA) ADC conversion data are transferred by DMA, in limited mode (one shot mode): DMA transfer requests are stopped when number of DMA data transfers (number of ADC conversions) is reached. This ADC mode is intended to be used with DMA mode non-circular.
    // LL_ADC_REG_DMA_TRANSFER_UNLIMITED  (ADC_CR2_DDS | ADC_CR2_DMA) ADC conversion data are transferred by DMA, in unlimited mode: DMA transfer requests are unlimited, whatever number of DMA data transferred (number of ADC conversions). This ADC mode is intended to be used with DMA mode circular.
    // Как я понимаю, в cube опция DMA_CONTINIUS_REQUEST = enable включает LL_ADC_REG_DMA_TRANSFER_UNLIMITED
    ADC1->CR2 |= ADC_CR2_DMA;



    //LL_DMA_EnableIT_HT(DMA2, LL_DMA_STREAM_4);
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_4);



    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_4);
    //DMA2_Stream4->CR |= DMA_SxCR_EN;



    //LL_ADC_EnableIT_EOCS(ADC1);
    //LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_4);

  // Тут нельзя включать АЦП, ибо TIM5 ещё не инициализирован

  //LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_4);

  //LL_TIM_EnableCounter(TIM5); // который запускает АЦП
  g_AdcStartT = TIM2->CNT;

  //LL_TIM_EnableCounter(TIM3);
  TIM3->CNT = TIM3->ARR-1;
  //LL_TIM_EnableMasterSlaveMode(TIM3);
  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);

};


int GetAdcPeriod()
{
	return TIM3->ARR + 1;
};

void SetOutPeriod(int OutPeriod)  // OutPeriod - период соответствует желаемой частоте, должен быть четным
{
  // у нас не шим, а компаратор с последующим триггером, период таймера должен быть вдвое короче
  OutPeriod /= 2;
  TIM1->ARR = OutPeriod - 1;
};

int GetOutPeriod()
{
  return (TIM1->ARR + 1) * 2; // у нас не шим, а компаратор с последующим триггером на выходе, считаем итоговый период
};


// *********************************************************************************************************

// s0: [0x08000000 - 0x08003FFF], 16 Kbytes - тут прошивка или загрузчик
// s1: [0x08004000 - 0x08007FFF], 16 Kbytes
// s2: [0x08008000 - 0x0800BFFF], 16 Kbytes
// s3: [0x0800C000 - 0x0800FFFF], 16 Kbytes
// s4: [0x08010000 - 0x0801FFFF], 64 Kbytes
// s5: [0x08020000 - 0x0803FFFF], 128 Kbytes - тут опции (хотя конечно лучше бы загрузщик и s1)

#define Options_BaseAdr (0x08020000)
#define Options_Sector  (5)

volatile uint8_t IsSavingParamsToFlash = 0;

static inline void ProgFlashByte(uint8_t * p, uint8_t DD)
{
  uint8_t * const pBaseAdr = (uint8_t *) Options_BaseAdr;

  if (p < pBaseAdr || p >= (pBaseAdr+16384)) return; // только область сохранения опций
  *p = DD;
  while (FLASH->SR & FLASH_SR_BSY) ; // ждем завершения операции
};

static inline void ProgFlashUnlock()
{
  while (FLASH->SR & FLASH_SR_BSY) ; // ждем завершения пред. операции

  FLASH->SR |= FLASH_SR_SOP | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR; // сброс флагов ошибки их уст. в 1
  FLASH->SR |= FLASH_SR_EOP; // очистка флага End of operation установкой в 1

  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;  // снять бит FLASH_CR_LOCK

  FLASH->CR = FLASH_CR_PG; // по умолчанию 0x80000000, нам надо включить бит PG, а остальные опции - нулевые (запреты прерываний, PSIZE=x8)
};

static inline void ProgFlashLock()
{
  FLASH->CR = FLASH_CR_LOCK; // ибо по умолчанию 0x80000000
};

static inline void EraseFlashPage(uint32_t PageN)
{
  if (PageN != Options_Sector) return;

  while (FLASH->SR & FLASH_SR_BSY) ; // ждем завершения пред. операции

  FLASH->SR |= FLASH_SR_SOP | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR; // сброс флагов ошибки их уст. в 1
  FLASH->SR |= FLASH_SR_EOP; // очистка флага End of operation установкой в 1

  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;  // снять бит FLASH_CR_LOCK

  //обнуляем FLASH_CR_SNB; // по умолчанию 0x80000000, остальные опции - нулевые (запреты прерываний, PSIZE=10 - x32)
  FLASH->CR = FLASH_CR_SER | FLASH_CR_PSIZE_1;
  FLASH->CR |= (PageN << FLASH_CR_SNB_Pos) & FLASH_CR_SNB;

  FLASH->CR |= FLASH_CR_STRT;  // стирание сектора

  while (FLASH->SR & FLASH_SR_BSY) ; // ждем завершения операции
  while (FLASH->SR & FLASH_SR_BSY) ; // ждем завершения операции
  while (FLASH->SR & FLASH_SR_BSY) ; // ждем завершения операции
  while (FLASH->SR & FLASH_SR_BSY) ; // ждем завершения операции

  FLASH->CR = FLASH_CR_LOCK; // ибо по умолчанию 0x80000000
};

static void NikyFlashDataCashReset()
{
  // точный порядок сброса кэша данных флэш-памяти МК нигде не прописан
  //LL_FLASH_DisableDataCache();
  //LL_FLASH_EnableDataCache();

  FLASH->ACR &= ~FLASH_ACR_DCEN;          // Data cache is disabled
  while (FLASH->ACR & FLASH_ACR_DCEN) ;

  FLASH->ACR |= FLASH_ACR_DCRST;          // Data cache is reset
  //while (!(FLASH->ACR & FLASH_ACR_DCRST)) ;
  //FLASH->ACR &= ~FLASH_ACR_DCRST;         // Data cache reset снять
  //while (FLASH->ACR & FLASH_ACR_DCRST) ;

  FLASH->ACR |= FLASH_ACR_DCEN;           // Data cache включить
};



static inline void OptionsAct(int Act, uint8_t * const p, uint8_t * const pDst, uint8_t * pcmp)
{
  if (Act == 1)
  {
	*pDst = *p;
	return;
  };

  if (Act == 2)
  {
    const uint8_t mm = *p;
    if (*pDst != mm) *pcmp |= 1;

    const uint8_t mm_inv = ~mm;
    if (*pDst & mm_inv) *pcmp |= 2;

    return;
  };

  if (Act == 3)
  {
	ProgFlashByte(p, *pDst);
	return;
  };
};

uint8_t OptionsLoad(int Act) // Act = 1 - чтение из памяти в параметры
{
  uint8_t * p = (uint8_t *) Options_BaseAdr;

  uint8_t cmp = 0;

  uint8_t * pDst = (uint8_t *) &g_VVInfo;

  TProgOptions VVInfoRd;
  if (Act == 1) pDst = (uint8_t *) &VVInfoRd;

  const int Sz = sizeof(g_VVInfo);
  for (int i=0; i<Sz; i++)
  {
     OptionsAct(Act, p++, pDst+i, &cmp);
  };
  if (Act == 1)
  {
	  if (VVInfoRd.Signature == 0x55)
	  {
		VVInfoTestKorrect(&VVInfoRd);
		g_VVInfo = VVInfoRd;
		CalFromVVParams();
	  };
  }

  return cmp;
};

volatile uint8_t MainLoop_StopOutOled = 0;



volatile uint8_t MainLoop_IsGotoProgMode = 0;
volatile uint32_t * GetLockMonValue()
{
	// внимание, хотя бы для загрузчика в .ld используемый объём RAM ДОЛЖЕН БЫТЬ уменьшен, и соотв. тут должен быть адр из неиспользуемого блока
	return (uint32_t *) (0x2000fff8);
};

void MidiProcessFromUsb(uint8_t * UsbCmd, int UsbCmdSz)
{
};




volatile uint8_t * m_EP1_RvcDataBuf = NULL;  // для приёма данных от EP1, инкрементится по мере приёма
volatile int m_EP1_RvcDataBufSz = 0; // сколько ещё осталось
volatile int m_EP1_RvcDataLen = 0;   // сколько байт скопировано

void MainAppCmdFromUsb(uint8_t * UsbCmd, int UsbCmdSz)
{
	if (UsbCmdSz == 64)  // не команда, а данные, будем копировать в заранее заданный (другой командой) буфер
	{
		  if (m_EP1_RvcDataBufSz <= 0) return;  // конец приёма
		  volatile uint32_t * pSrc = (volatile uint32_t *) UsbCmd;
		  volatile uint32_t * pDst = (volatile uint32_t *) m_EP1_RvcDataBuf;
		  int Sz = m_EP1_RvcDataBufSz;
		  if (Sz > 64) Sz=64;
		  Sz >>= 2;
	      for (int i=0; i<Sz; i++) *pDst++ = *pSrc++;
	      m_EP1_RvcDataBufSz -= 64;
	      m_EP1_RvcDataBuf += 64;
	      m_EP1_RvcDataLen += 64;
	      return;
	};

    m_EP1_RvcDataBufSz = 0; // отменяем заполнение буфера на всякий случай

    if (UsbCmd[0] == 0x10)  // команда перехода в штатный режим устройства
    {
    	MainLoop_IsGotoProgMode = 128; // перезагрузка
    	return;
    };

    if (UsbCmd[0] == 0x11)  // Spi_Out
    {
       const int n = UsbCmdSz-1;
       for (int i=0; i<n; i++) SSP_WriteRead(UsbCmd[1+i]);
       return;
    };

    if (UsbCmd[0] == 0x12)  // Spi_In
    {
       const int n = UsbCmd[1];
       for (int i=0; i<n; i++) m_UsbCmdRetBuf[i] = SSP_WriteRead(0xff);
       NikyUsbDev_EP1OutCmdRet(m_UsbCmdRetBuf, n);
       return;
    };

    if (UsbCmd[0] == 0x13)  // OutPortB
    {
      W25SelectProc(UsbCmd[1]);
      return;
    };


    if (UsbCmd[0] == 0x14)  // GetMKProgVersion
    {
    	m_UsbCmdRetBuf[0] = 0xa7;  // сигнатуры ответа
    	m_UsbCmdRetBuf[1] = 0x91;
    	m_UsbCmdRetBuf[2] = 0x00;     // код продукта Коли Lo
    	m_UsbCmdRetBuf[3] = 0xf0;     // код продукта Коли Hi
    	m_UsbCmdRetBuf[4] = 1;        // ProgVer (версия прошивки МК, для познавания протокола обмена)
    	m_UsbCmdRetBuf[5] = 0;        // ProgSubVer
    	m_UsbCmdRetBuf[6] = 0;        // TypeMK (тип МК - для опознавания координат и размеров таблиц. 0 - stm32f401ccu6)
    	m_UsbCmdRetBuf[7] = 0;        // MKSuppFlags
    	m_UsbCmdRetBuf[8] = 0;        //
    	NikyUsbDev_EP1OutCmdRet(m_UsbCmdRetBuf, 9);
     	return;
    };

    if (UsbCmd[0] == 0x15)  // Spi_InCrc32
    {
    	int n = UsbCmd[1];
    	n<<=8;

    	CRC->CR = 1;
        while (CRC->CR & 1) ;

        //SSP_Write(0xff); // попробуем конвеерно

        for (int i=0; i<n; i+=4)
        {
          uint8_t bb[4] = { SSP_WriteRead(0xff), SSP_WriteRead(0xff), SSP_WriteRead(0xff), SSP_WriteRead(0xff) };
          CRC->DR = * (uint32_t *)bb;
        };

        //SSP_Read(); // конец конвеера (и лишний прочитанный байт)

        uint32_t crc = CRC->DR;
        m_UsbCmdRetBuf[0] = crc;
        m_UsbCmdRetBuf[1] = crc>>8;
        m_UsbCmdRetBuf[2] = crc>>16;
        m_UsbCmdRetBuf[3] = crc>>24;
        NikyUsbDev_EP1OutCmdRet(m_UsbCmdRetBuf, 4);

        return;
    };

    /*
    if (UsbCmd[0] == 0x17)  // Spi_In в BigBuffer
    {
      int nn = * (int *) (UsbCmd+1);
      if (nn < 0) nn = 0;
      if (nn > sizeof(m_SoundBuffer)) nn = sizeof(m_SoundBuffer);

      uint8_t * const BigBuffer = (uint8_t *) &m_SoundBuffer;
      for (int i=0; i<nn; i++) BigBuffer[i] = SSP_WriteRead(0xff);

      NikyUsbDev_EP1Clr();

      m_EP1_AutoSend_Buf = (uint8_t *) BigBuffer;
      m_EP1_AutoSend_BufSz = nn;
      m_EP1_AutoSendStatus = 1;  // настройка автоотправки по окончанию отправки из HAL_PCD_DataInStageCallback()

      m_UsbCmdRetBuf[0] = nn; m_UsbCmdRetBuf[1] = nn>>8; m_UsbCmdRetBuf[2] = nn>>16; m_UsbCmdRetBuf[3] = nn>>24;
      NikyUsbDev_EP1Send(m_UsbCmdRetBuf, 4);
      return;
    };
    */

    if (UsbCmd[0] == 0x18)  // переход в режим управления с компа
    {
    	if (!MainLoop_StopOutOled)
    	{
    	   if (UsbCmd[1] == 0x55) MainLoop_StopOutOled = 2; else MainLoop_StopOutOled = 1;
    	};
    	m_UsbCmdRetBuf[0] = 0xa8; m_UsbCmdRetBuf[1] = 0xf5; m_UsbCmdRetBuf[2] = 0xe9; m_UsbCmdRetBuf[3] = 0x3d;
    	NikyUsbDev_EP1OutCmdRet(m_UsbCmdRetBuf, 4);
    	return;
    };

    if (UsbCmd[0] == 0x29)  // DevCmdSaveEEPROMAllParams(), команда сохранения во флэш параметров всех сразу
    {
    	uint8_t cmp = 128;
    	if (IsSavingParamsToFlash) goto LL_SkipSaveParamsToFlash;

    	cmp = OptionsLoad(2);
    	if ((cmp & 1) || (UsbCmd[1] & 1))  // надо сохранять, есть изменения!
    	{
     	   if ((cmp & 2) || (UsbCmd[1] & 2)) // и необходимо стирание
     	   {
     		 // сама деблокирует флэшку и блокирует потом
     		 EraseFlashPage(Options_Sector);
     	   };

     	   ProgFlashUnlock();
    	   OptionsLoad(3);  // запись во флэш побайтно
    	   ProgFlashLock();

    	   NikyFlashDataCashReset();
    	};

    	LL_SkipSaveParamsToFlash:
        m_UsbCmdRetBuf[0] = cmp;
    	NikyUsbDev_EP1OutCmdRet(m_UsbCmdRetBuf, 1);
    	return;
    };

    if (UsbCmd[0] == 0x2a)  // стереть сектор с настройками
    {
   	  EraseFlashPage(Options_Sector);
      return;
    };



    if (UsbCmd[0] == 0x37)  // Тесты GSM COM
    {
    	if (UsbCmd[1] == 0x9c)
    	{
   		  uint16_t PosWr = m_OutUsartBufferPosWr;
   		  m_OutUsartBuf2[PosWr++] = 0xcd;      if (PosWr >= OutUsartBuf2_NMax) PosWr = 0;
   		  m_OutUsartBuf2[PosWr++] = 0xe0;      if (PosWr >= OutUsartBuf2_NMax) PosWr = 0;
   		  m_OutUsartBuf2[PosWr++] = UsbCmd[2]; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0;
   		  m_OutUsartBuf2[PosWr++] = UsbCmd[3]; if (PosWr >= OutUsartBuf2_NMax) PosWr = 0;
   		  m_OutUsartBufferPosWr = PosWr;
   		  EP2_Send();
     	  return;
    	};
    	return;
    };

    if (UsbCmd[0] == 0x38)  // adc test
    {
    	if (UsbCmd[1] == 0x1)
    	{
    		g_IsMustStartIzmVV = UsbCmd[2];
    		return;
    	};
    	if (UsbCmd[1] == 0x2)
    	{
    		g_IsMustLockVV = 0;
    		return;
    	};
    	if (UsbCmd[1] == 0x3)
    	{
    		g_VVInfo.StartScanMode = UsbCmd[2];

    		FindValue = 0;
    		FindOutPeriod = 0;
    		if (ScanMode==3 && (UsbCmd[2] & 7) == 3) g_ScanDir = !g_ScanDir;
    		ScanMode = UsbCmd[2] & 7;
    		return;
    	};
    	if (UsbCmd[1] == 0x4)
    	{
    		g_VVInfo.IsOffOnScan = UsbCmd[2];
    		g_VVInfo.OffTime10MKS = * (uint16_t*) (UsbCmd+32);
    	 	return;
    	};
    	if (UsbCmd[1] == 0x5)
    	{
    	  int fm1 = * (int *) (UsbCmd+2);
    	  int fm2 = * (int *) (UsbCmd+2+4);
    	  g_VVInfo.DiaFM1 = fm1;
    	  g_VVInfo.DiaFM2 = fm2;
    	  VVInfoTestKorrect(&g_VVInfo);
    	};

    	if (UsbCmd[1] == 0x6)
    	{
    	  const uint8_t * const p = (uint8_t *) &g_VVInfo;
          for (int i=0; i<sizeof(g_VVInfo); i++) m_UsbCmdRetBuf[i] = p[i];
          NikyUsbDev_EP1OutCmdRet(m_UsbCmdRetBuf, sizeof(g_VVInfo));
          return;
    	};
    	if (UsbCmd[1] == 0x7)
    	{
    	  uint8_t * const p = (uint8_t *) &g_VVInfo;
          for (int i=0; i<sizeof(g_VVInfo); i++) p[i] = UsbCmd[2+i];
          VVInfoTestKorrect(&g_VVInfo);
          CalFromVVParams();
          return;
    	};

    	if (UsbCmd[1] == 0x8)
    	{
    	  g_VVInfo.OledOptions = UsbCmd[2];
    	  return;
    	};


    	if (UsbCmd[1] == 0x80) // инфа
    	{
    	   m_UsbCmdRetBuf[0] = g_AdcDmaIsWork;
    	   m_UsbCmdRetBuf[1] = ADC1->SR;
    	   *(uint32_t*)(m_UsbCmdRetBuf+2) = TIM3->CNT;
    	   *(uint32_t*)(m_UsbCmdRetBuf+6) = ADC1->CR1;
    	   *(uint32_t*)(m_UsbCmdRetBuf+10) = ADC1->CR2;
    	   *(uint32_t*)(m_UsbCmdRetBuf+14) = DMA2->HISR;
    	   *(uint32_t*)(m_UsbCmdRetBuf+18) = DMA2_Stream4->CR;
    	   *(uint32_t*)(m_UsbCmdRetBuf+22) = DMA2_Stream4->NDTR;
    	   *(uint32_t*)(m_UsbCmdRetBuf+26) = TIM2->CNT;
    	   *(uint32_t*)(m_UsbCmdRetBuf+30) = g_AdcEndT - g_AdcStartT;
    	   NikyUsbDev_EP1OutCmdRet(m_UsbCmdRetBuf, 34);
    	   return;
    	};

    	if (UsbCmd[1] == 0x81)
    	{
   		   ADC1->CR2 &= ~ADC_CR2_DMA;  // передергивание позволяет возобновить работу после завершения передачи данных по ДМА
    	   ADC1->CR2 |= ADC_CR2_DMA;   // (если не стоит бит DDS)
     	   return;
    	};

    	if (UsbCmd[1] == 0x9a) // время выборки
    	{
    		//int aa = LL_ADC_SAMPLINGTIME_3CYCLES;
    		//if (UsbCmd[4] == 1) aa = LL_ADC_SAMPLINGTIME_15CYCLES;
    		//if (UsbCmd[4] == 2) aa = LL_ADC_SAMPLINGTIME_28CYCLES;
    		//if (UsbCmd[4] == 3) aa = LL_ADC_SAMPLINGTIME_56CYCLES;
    		//if (UsbCmd[4] == 4) aa = LL_ADC_SAMPLINGTIME_84CYCLES;
    	    //NikyAdcSetST(aa);
    	    return;
    	};

    	if (UsbCmd[1] == 0xa9)
    	{
    	  int nn = * (int *) (UsbCmd+2);
    	  if (nn < 0) nn = 0;
    	  if (nn > g_Adc1Len*2) nn = g_Adc1Len*2;

    	  if (g_IsMustStartIzmVV) nn = 0;

  	      NikyUsbDev_EP1Clr();

   	      m_EP1_AutoSend_Buf = (uint8_t *) g_Adc1ResBuf;
   	      m_EP1_AutoSend_BufSz = nn;
   	      m_EP1_AutoSendStatus = 1;  // настройка автоотправки по окончанию отправки из HAL_PCD_DataInStageCallback()

    	  m_UsbCmdRetBuf[0] = nn; m_UsbCmdRetBuf[1] = nn>>8; m_UsbCmdRetBuf[2] = nn>>16; m_UsbCmdRetBuf[3] = nn>>24;
    	  m_UsbCmdRetBuf[4] = g_IsMustLockVV;
    	  m_UsbCmdRetBuf[5] = g_IsMustStartIzmVV;
    	  m_UsbCmdRetBuf[6] = g_Adc1Time; m_UsbCmdRetBuf[7] = g_Adc1Time>>8; m_UsbCmdRetBuf[8] = g_Adc1Time>>16; m_UsbCmdRetBuf[9] = g_Adc1Time>>24;
    	  m_UsbCmdRetBuf[10] = g_Adc1OutPeriod;
    	  m_UsbCmdRetBuf[11] = g_Adc1OutPeriod>>8;
    	  m_UsbCmdRetBuf[12] = GetAdcPeriod();
    	  NikyUsbDev_EP1Send(m_UsbCmdRetBuf, 4+2+4+2+1);

    	  //g_IsMustLockVV = 0; // тут нельзя, ещё не отправлено
    	  return;
     	};

    	if (UsbCmd[1] == 0xaa)
    	{
    	  //g_VVInfo.StartScanMode = 0;
    	  ScanMode = 0;
    	  //g_VVInfo.IsOffOnScan = 0;
    	  int nn = * (int *) (UsbCmd+2);  // период соответствует желаемой частоте, должен быть четным
    	  g_VVInfo.CurFM = nn;
    	  VVInfoTestKorrect(&g_VVInfo);
    	  SetOutPeriod(g_VVInfo.CurFM);
    	  if (UsbCmd[2+4] == 0x55) g_IsMustLockVV = 1; // и заблокировать опрос
    	};
    	if (UsbCmd[1] == 0xab)
    	{
    		//int nn = * (int *) (UsbCmd+2);
      	    //TIM1->CCR1 = nn;
    	};
    	if (UsbCmd[1] == 0xac)
    	{
   		  if (UsbCmd[2])
   		  {
   			  g_VVInfo.IsOutOn = 0xff;
   			  //LL_TIM_EnableAllOutputs(TIM1);  // установка бита TIM_BDTR_MOE - MOE: Main output enable
   			  LL_TIM_EnableAutomaticOutput(TIM1);
   		  }
   		  else
   		  {
   			 g_VVInfo.IsOutOn = 0x0;
   	         LL_TIM_DisableAutomaticOutput(TIM1); // сброс бита AOE
   			 LL_TIM_DisableAllOutputs(TIM1); // сброс бита TIM_BDTR_MOE
   		  };
    	};

    	return;
    };

    if (UsbCmd[0] == 0x5e)  // Тесты oled
    {
    	if (UsbCmd[1] == 0) OledClear();
    	if (UsbCmd[1] == 1) OledPrintChar(UsbCmd[2], UsbCmd[3], UsbCmd[4]);
    	if (UsbCmd[1] == 2) OledPrintCharX1_5(UsbCmd[2], UsbCmd[3], UsbCmd[4]);
    	//if (UsbCmd[1] == 3) OledPrintCharX2(UsbCmd[2], UsbCmd[3], UsbCmd[4]);
    	if (UsbCmd[1] == 4) OledPrintCharX3(UsbCmd[2], UsbCmd[3], UsbCmd[4]);
    	if (UsbCmd[1] == 5 && MainLoop_StopOutOled == 2) MainLoop_StopOutOled = 3;
    	return;
    };

    if (UsbCmd[0] == 0x5f)  // Отладка web usb
    {
  	  m_UsbCmdRetBuf[0] = NikyUsbDebug_GetUrlN;
  	  m_UsbCmdRetBuf[1] = NikyUsbDebug_GetBOSDescriptorN;
  	  NikyUsbDev_EP1Send(m_UsbCmdRetBuf, 2);
    };

    //if (UsbCmd[0] == 0x38)  // Тесты аппаратного миди

};

unsigned int StepUI(unsigned int x, int step)
{
  unsigned int p = 1;
  while (step > 0)
  {
    if (step & 1) p*=x;
    x *= x;
    step >>= 1;
  };
  return p;
};

// Len - сколько всего знаков в строку должно быть выведено (включая точку и после запятой)
// Xvost - сколько знаков после запятой
//
void NikyPrintF(char * str, float Value, int LenAll, int Xvost)
{
  if (Value < 0)
  {
	  Value = -Value;
	  str[0] = '-';
	  str++; LenAll--;
  };
  if (LenAll < 1)
  {
     str[0] = 0;
	 return;
  };

  unsigned int Cel;
  char sDrob[16];

  if (Xvost < 1)
  {
	 Cel = (unsigned int) floor(Value + 0.5);
	 //sDrob[0] = 0;
  }
  else
  {
    const unsigned int Koo = StepUI(10, Xvost);
    unsigned int val = (unsigned int) floor(Value*Koo + 0.5);
    Cel = val / Koo;
    unsigned int Drob = val % Koo;
    for (int i=0; i<Xvost; i++)
    {
      unsigned int Cf = Drob % 10;
      Drob = Drob / 10;
      sDrob[Xvost-1-i] = '0' + Cf;
    };
    //sDrob[Xvost] = 0;
  };

  char sCel[16]; // цифры целой части с конца
  int n = 0;
  do
  {
	unsigned int Cf = Cel % 10;
	Cel = Cel / 10;
	sCel[n++] = '0' + Cf;
  } while (Cel);

  if (n > LenAll) // целая часть увы не влазит
  {
	  for (int i=0; i<LenAll; i++) str[i] = 'B';
	  str[LenAll] = 0;
	  return;
  };

  int n20 = LenAll - n; // сколько пробелов надо в начале перед числом
  if (Xvost > 0) n20 = n20 - 1 - Xvost; // с учетом .хвоста

  if (n20 < 0) n20 = 0;
  for (int i=0; i<n20; i++)
  {
	*str++ = ' '; // вписываем пробелы, если нужно
  };
  LenAll -= n20;

  for (int i=0; i<n; i++)
  {
	*str++ = sCel[n-1-i]; // вписываем цифры целой части числа
  };
  LenAll -= n;

  if (LenAll < 1)
  {
	*str = 0;
	return;
  };

  if (LenAll == 1) // влазит только точка
  {
	*str++ = ' ';  // но мы её заменяем на пробел
	*str = 0;
	return;
  };

  *str++ = '.'; LenAll--;
  for (int i=0; i<LenAll; i++)
  {
	 *str++ = sDrob[i];
  };
  *str = 0;
};


void PrintInt4Cif(int Value, char * str)
{
  if (Value < 0)
  {
    str[0] = str[1] = str[2] = str[3] = '?'; str[4] = 0;
    str[3] = 'M';
    return;
  };

  if (Value > 9999)
  {
	  str[0] = str[1] = str[2] = str[3] = '9'; str[4] = 0;
	  str[3] = 'B';
	  return;
  };

  int V = Value;

  int c4 = V / 1000;
  V -= c4*1000;

  int c3 = V / 100;
  V -= c3*100;

  int c2 = V / 10;
  V -= c2*10;

  str[0] = '0' + c4;
  str[1] = '0' + c3;
  str[2] = '0' + c2;
  str[3] = '0' + V;
  str[4] = 0;

  if (Value < 1000) str[0] = ' ';
  if (Value < 100) str[1] = ' ';
  if (Value < 10) str[2] = ' ';
};

void PrintInt3Cif(int Value, char * str)
{
  if (Value < 0)
  {
    str[0] = str[1] = str[2] = '?'; str[3] = 0;
    str[2] = 'M';
    return;
  };

  if (Value > 999)
  {
	  str[0] = str[1] = str[2] = '9'; str[3] = 0;
	  str[2] = 'B';
	  return;
  };

  int V = Value;
  int c3 = V / 100;
  V -= c3*100;

  int c2 = V / 10;
  V -= c2*10;

  str[0] = '0' + c3;
  str[1] = '0' + c2;
  str[2] = '0' + V;
  str[3] = 0;

  if (Value < 100) str[0] = ' ';
  if (Value < 10) str[1] = ' ';
};


void PrintInt2Cif(int Value, char * str)
{
  if (Value < 0)
  {
    str[0] = str[1] = '?'; str[2] = 0;
    str[1] = 'M';
    return;
  };

  if (Value > 99)
  {
	  str[0] = str[1] = '9'; str[2] = 0;
	  str[1] = 'B';
	  return;
  };

  int V = Value;
  int c2 = V / 10;
  V -= c2*10;

  str[0] = '0' + c2;
  str[1] = '0' + V;
  str[2] = 0;

  if (Value < 10) str[0] = ' ';
};


void OledShowVV(float VV_PikI, float VV_PikU, float VV_Pwr, int OutPeriod, int PereFlags)
{
  static uint32_t t0 = 0;
  if (ScanMode && TIM2->CNT - t0 < 84e6/3) return;

  float rU = VV_PikU * Cal_CodeToVV_U;

  char s[16];
  //int iU = (int) (rU + 0.5);
  //PrintInt4Cif(iU, s);

  if (rU > 9999)
  {
	NikyPrintF(s, rU*1e-3f, 4, 1);
    OledPrintX1_5(0, 0, s);
    OledPrintX1(15*4+4, 1, (PereFlags & 2) ? "К" : "к");
  }
  else
  {
	NikyPrintF(s, rU, 4, 0);
    OledPrintX1_5(0, 0, s);
    OledPrintX1(15*4+4, 1, (PereFlags & 2) ? "B" : "в");
  };

  float rI = VV_PikI * Cal_CodeToVV_I;
  /*int iI = (int) (rI*10 + 0.5);
  int nI = iI / 10;
  iI -= 10*nI;
  PrintInt2Cif(nI, s);
  s[2] = '.';
  s[3] = '0' + iI; */
  NikyPrintF(s, rI, 4, 2); //0.12 или 12.2 или 120_ или 9999
  OledPrintX1(128-10*4, 0, s);
  OledPrintX1(128-10*2, 2, (PereFlags & 1) ? "МА" : "мА");


  float rP = VV_Pwr * Cal_CodeToVV_U * Cal_CodeToVV_I * 1e-3;
  /*int iP = (int) (rP*1000 + 0.5);
  if (iP < 0) iP = 0;
  int nP = iP / 1000;
  iP -= 1000*nP;
  PrintInt2Cif(nP, s);
  s[2] = '.';
  s[3] = '0' + (iP/100);
  s[4] = '0' + (iP/10) % 10;
  s[5] = '0' + iP % 10;
  s[6] = 0; */
  NikyPrintF(s, rP, 6, 3);
  if (!LL_TIM_IsEnabledAllOutputs(TIM1))
  {
	strcpy(s, "ВЫКЛ  ");
  };
  OledPrintX1_5(0, 3, s);
  OledPrintX1(6*15+8, 3+1, "Вт");


  float FM = 84e3 / OutPeriod;
  NikyPrintF(s, FM, 5, 2);
  s[5] = 'к';
  s[6] = 0;
  OledPrintX1(0, 6, s);
  /*int iFM = (int)(FM*10 + 0.5);
  int nFM = iFM / 10;
  iFM -= nFM*10;
  if (nFM < 100)
  {
	  PrintInt2Cif(nFM, s);
	  s[2] = '.';
	  s[3] = '0' + iFM;
	  s[4] = 'к';
	  s[5] = 0;
	  OledPrintX1(0, 6, s);
  }
  else
  {
	  PrintInt3Cif(nFM, s);
	  s[3] = '.';
	  s[4] = '0' + iFM;
	  s[5] = 0;
	  OledPrintX1(0, 6, s);
  };*/

  char sFF[2] = " ";
  static uint8_t nn; nn++;
  if ((nn & 1) && (ScanMode == 1 || ScanMode == 2) && (g_VVInfo.StartScanMode & 0x70))
  {
	  if ((g_VVInfo.StartScanMode & 0x70) == 0x10) sFF[0] = 'I';
	  if ((g_VVInfo.StartScanMode & 0x70) == 0x20) sFF[0] = 'U';
	  if ((g_VVInfo.StartScanMode & 0x70) == 0x30) sFF[0] = 'P';
	  if ((g_VVInfo.StartScanMode & 0x70) == 0x40) sFF[0] = 'C';
  };
  OledPrintX1(6*10, 6, sFF);


  t0 = TIM2->CNT;
};


void OledShowPit(float PitI)
{
  static uint32_t t0 = 0;
  if (ScanMode && TIM2->CNT - t0 < 84e6/10) return;


  float rI = PitI * Cal_PitCodeToI + Cal_PitCodeOfsI;

  //int iI = (int) (rI*100 + 0.5);
  //uint8_t sgn = 0;
  //if (iI < 0) { sgn = 1; iI = -iI; };
  //int nI = iI / 100;
  //iI -= 100*nI;
  char s[16];
  //PrintInt2Cif(nI, s);
  //if (sgn)
  //{
  //if (s[0] == ' ') s[0] = '-'; else { s[0] = '-'; s[1] = 'B'; };
  //}
  //s[2] = '.';
  //s[3] = '0' + (iI/10) % 10;
  //s[4] = '0' + iI % 10;
  //s[5] = 0;
  NikyPrintF(s, rI, 4, 2);
  OledPrintX1(10*7, 6, s);

  OledPrintX1(128-10, 6, "А");

  t0 = TIM2->CNT;
};


void ProcessKbrd()
{
    static uint32_t OldKeyT0 = 0;
    static int OldKey = 0;
    if (GPIOA->IDR & 1) // кнопка не нажата
    {
  	  if (OldKey) // а раньше была нажата, т.е. отпускание
      {
  	    OldKey = 0;
      };
    }
    else // вижу кнопка нажата
    {
  	  if (!OldKey) // это фронт нажатия
  	  {
   	    OldKey = 1;
   	    OldKeyT0 = TIM2->CNT;
  	    if (!g_VVInfo.IsOutOn)
  	    {
  	      g_VVInfo.IsOutOn = 1;
  	  	  LL_TIM_EnableAutomaticOutput(TIM1);
  	    };
	    if (ScanMode) ScanMode = 0;
	    else
	    {
	    	ScanMode = 3;
	    	g_ScanDir = !g_ScanDir;
	    };
 	  };

	  uint32_t dT = TIM2->CNT - OldKeyT0;

	  if (OldKey == 1 && dT > 84e6) // обнаружено долгое нажатие
	  {
		  OldKey = 2;

		  ScanMode = 1; // запускаем интеллектуальный поиск вверх
		  FindValue = 0;
		  FindOutPeriod = 0;
		  SetOutPeriod(g_VVInfo.DiaFM1);
		  if ((g_VVInfo.StartScanMode & 0x70) < 0x10 || (g_VVInfo.StartScanMode & 0x70) > 0x40)
	  	  {
		    g_VVInfo.StartScanMode &= ~0x70;
		    g_VVInfo.StartScanMode |= 0x10;  // критерий ток
		  };
	    };

    };
};


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  if (0)
  {
	 __disable_irq();
	 SCB->VTOR = 0x8008000;  // включить только для работы под монитором
	 __DSB();
	 __enable_irq();    //разрешим прер., которые запретил загрузчик, иначе не раб.
  };

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_CRC_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  LL_TIM_EnableCounter(TIM2);
  //LL_TIM_EnableIT_UPDATE(TIM2);  // Enable update interrupt (UIE)

  //SPI1->CR1 |= SPI_CR1_SPE;
  //LL_SPI_Enable(SPI1);

  MyUsbDevInitGlobal();  // инициализацию глобальных переменных надо вызвать до запуска usb и использования отправки потоков по EP2
  MyUsbDevStart(); // включение usb устройства

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Отключение всех исключений FPU
  //FPU->FPCCR &= ~(FPU_FPCCR_IOE_Msk | FPU_FPCCR_DZE_Msk | FPU_FPCCR_OFE_Msk | FPU_FPCCR_UFE_Msk | FPU_FPCCR_IXE_Msk);

  VVInfoInitDef(&g_VVInfo);
  if (10)
  {
	 OptionsLoad(1); // прочесть конфирурацию из флеша, если её задал комп
  };
  VVInfoTestKorrect(&g_VVInfo);
  CalFromVVParams();


  int ShimValue = 0x3000;

  //InitUsartMidiDma();

  // CCER - capture/compare enable register, содержит биты TIM_CCER_CC1E, TIM_CCER_CC1NE и прочее по включению выходов
  // Там же ещё биты полярности вида TIM_CCER_CC1NP
  // Реальное поведение выхода (как вкл, так и выкл) зависит от бит MOE, OSSI, OSSR, OIS1, OIS1N и CC1E
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N); // необходимо

  //LL_TIM_EnableAllOutputs(TIM1);  // установка бита TIM_BDTR_MOE - MOE: Main output enable
  //LL_TIM_DisableAllOutputs(TIM1); // сброс бита TIM_BDTR_MOE

  // Бит BDTR.AOE (Automatic output enable) - если 1, то MOE сам установится при next update event. �?наче только софтово

  // Бит OSSR (Off-State Selection Run) - Выбор состояния в режиме RUN (для MOE=1), но когда канал отключен битами OC/OCN
  // Бит OSSI (Off-state selection for Idle mode) для MOE=0. Если 0, то выключаются (Z или как в GPIO).

  uint8_t OledOptions = g_VVInfo.OledOptions;
  if (g_VVInfo.OledOptions & 1)
  {
    OledInit(g_VVInfo.OledOptions & 2, g_VVInfo.OledOptions & 32);
    OledClear();
    OledPrintX1(0,0, "Привет");
    MainLoop_StopOutOled = 0;
  }
  else MainLoop_StopOutOled = 44; // блокировка навсегда



  SetOutPeriod(g_VVInfo.CurFM);
  TIM1->CNT = 0;
  LL_TIM_EnableCounter(TIM1); // tim1 - генератор импульсов

  if (g_VVInfo.IsOutOn)
  {
	  LL_TIM_EnableAutomaticOutput(TIM1);
	  //LL_TIM_EnableAllOutputs(TIM1);  // установка бита TIM_BDTR_MOE - MOE: Main output enable
  };


  //uint32_t T0 = LL_TIM_GetCounter(TIM2);

  int varadc = 0;

  uint8_t AdcIsStart = 0;
  int AdcStartType = 0;

  FindValue = 0;
  FindOutPeriod = 0;
  ScanMode = g_VVInfo.StartScanMode & 15;

  while (1)
  {
	    uint32_t a = LL_TIM_GetCounter(TIM2);

	  	if ((a & 0x3fff) < ShimValue || ((g_VVInfo.OledOptions & 16) && MainLoop_StopOutOled==0))
	   		LL_GPIO_SetOutputPin(NikyLed1_GPIO_Port, NikyLed1_Pin); else LL_GPIO_ResetOutputPin(NikyLed1_GPIO_Port, NikyLed1_Pin);

	   	ShimValue = (a / 5126 / 2) & 0x3fff;
	   	ShimValue *= ShimValue;
	   	ShimValue >>= 14;

	   	if (AdcIsStart && !g_AdcDmaIsWork) // запущенная ранее главным циклом операция оцифровки закончена
	   	{
	      const uint32_t dT = g_AdcEndT - g_AdcStartT;

	      if (g_IsMustStartIzmVV && g_IsMustStartIzmVV == AdcStartType && AdcStartType < 128) AdcIsStart = 2; // уже выполнено

          float VV_PikI = 0;  // для отображения
          float VV_PikU = 0;
          float VV_SredI = 0;
          float VV_SredU = 0;
          float VV_Pwr = 0;
          int VV_OutPeriod = 0;
          int VV_PereFlags = 0;

          static float PitI = 0;

	      if (AdcStartType >= 128)
          {
        	int sum = 0;
        	const int N = g_AdcUseLen;
        	for (int i=0; i<N; i++)
        	{
                uint16_t v = g_Adc2ResBuf[i];
        		sum += v;
        	};

        	uint16_t PosWr = m_OutUsartBufferPosWr;
            m_OutUsartBuf2[PosWr++] = 0xcd;      if (PosWr >= OutUsartBuf2_NMax) PosWr = 0;
        	m_OutUsartBuf2[PosWr++] = 0xe1;      if (PosWr >= OutUsartBuf2_NMax) PosWr = 0;
        	OutUsartBuf_WrB(PosWr, AdcStartType);
        	OutUsartBuf_WrW(PosWr, N);
	    	OutUsartBuf_WrD(PosWr, sum);
	    	OutUsartBuf_WrD(PosWr, dT);
	    	m_OutUsartBufferPosWr = PosWr;

	    	PitI = ((float)sum) / N;
          }
	      else
	      if (AdcStartType == StartAdcType_VV_IU)
	      {
	    	  g_Adc1Time = dT;
	    	  const int N = g_AdcUseLen & 0xfffe;
	    	  g_Adc1Len = N;
	    	  g_Adc1OutPeriod = GetOutPeriod();
	    	  uint32_t s1 = 0;
	    	  uint64_t d1 = 0;
	    	  uint32_t s2 = 0;
	    	  uint64_t d2 = 0;
        	  uint16_t vMin1 = 0xffff;
	          uint16_t vMax1 = 0;
        	  uint16_t vMin2 = 0xffff;
	          uint16_t vMax2 = 0;

	    	  for (int i=0; i<N; i++)
	    	  {
	    	    uint16_t v1 = g_Adc1ResBuf[i];
	    		if (vMin1 > v1) vMin1 = v1;
	    		if (vMax1 < v1) vMax1 = v1;
	    		s1 += v1;
	    	  	d1 += ((uint32_t)v1) * v1;
	    	  	i++;
	    	  	uint16_t v2 = g_Adc1ResBuf[i];
	    		if (vMin2 > v2) vMin2 = v2;
	    		if (vMax2 < v2) vMax2 = v2;
	    	    s2 += v2;
	    	    d2 += ((uint32_t)v2) * v2;
	    	  };

	    	  float ns1;
	    	  float ns2;
	    	  if (1)
	          {
	        	 VV_OutPeriod = g_Adc1OutPeriod;

	        	 ns1 = ((float)s1) / (N>>1);
	        	 VV_SredI = ((float)d1) / (N>>1) - ns1*ns1;
	        	 VV_PikI = vMax1-ns1;
	        	 if (VV_PikI < ns1-vMin1) VV_PikI = ns1-vMin1;

	        	 ns2 = ((float)s2) / (N>>1);
	             VV_SredU = ((float)d2) / (N>>1) - ns2*ns2;
	             VV_PikU = vMax2-ns2;
	             if (VV_PikU < ns2-vMin2) VV_PikU = ns2-vMin2;

	             if (g_VVInfo.OledOptions & 4)
	             {
	            	 VV_PikI = sqrt(VV_SredI);
	            	 VV_PikU = sqrt(VV_SredU);
	             };

	             if (vMin1 == 0 || vMax1==4095) VV_PereFlags |= 1;
	             if (vMin2 == 0 || vMax2==4095) VV_PereFlags |= 2;
	          };

	    	  const int OutPeriod = GetOutPeriod();
	    	  const int AdcPeriod = GetAdcPeriod();
	    	  int pos = 0;
	    	  int stp;
	    	  for (stp=0; stp<OutPeriod; stp++)
	    	  {
	    	    if (pos == 1) break;
	    	    pos += AdcPeriod;
	    	    if (pos >= OutPeriod) pos -= OutPeriod;
	    	  };

	    	  uint64_t P12 = 0;
	    	  uint16_t P12N = 0;

	    	  if (pos == 1) // сверхразрешение - считаем мощность используя stp, stp*AdcPeriod = 1 mod OutPeriod
	    	  {
	    		 for (int i0=0; i0+OutPeriod<=N; i0+=OutPeriod)
	    		 {
                    for (int di=0; di<OutPeriod; di+=2)
                    {
                       int j = di + stp;
                       if (j >= OutPeriod) j -= OutPeriod;
                       P12 += ((uint32_t)g_Adc1ResBuf[i0+di]) * g_Adc1ResBuf[i0+j];
                       P12N++;
                    };
	    		 };

		         if (1)
		         {
		        	VV_Pwr = ((float)P12) / P12N - ns1*ns2;
		         };
	    	  };

	    	  uint16_t PosWr = m_OutUsartBufferPosWr;
	    	  m_OutUsartBuf2[PosWr++] = 0xcd;       if (PosWr >= OutUsartBuf2_NMax) PosWr = 0;
	    	  m_OutUsartBuf2[PosWr++] = 0xe3;       if (PosWr >= OutUsartBuf2_NMax) PosWr = 0;
	    	  OutUsartBuf_WrB(PosWr, AdcStartType);
	    	  OutUsartBuf_WrW(PosWr, N);
	    	  OutUsartBuf_WrD(PosWr, s1);
	    	  OutUsartBuf_WrQ5(PosWr, d1);
	    	  OutUsartBuf_WrW(PosWr, vMin1);
	    	  OutUsartBuf_WrW(PosWr, vMax1);
	    	  OutUsartBuf_WrD(PosWr, s2);
	    	  OutUsartBuf_WrQ5(PosWr, d2);
	    	  OutUsartBuf_WrW(PosWr, vMin2);
	    	  OutUsartBuf_WrW(PosWr, vMax2);
	    	  OutUsartBuf_WrW(PosWr, P12N);
	    	  OutUsartBuf_WrQ5(PosWr, P12);
	    	  OutUsartBuf_WrB(PosWr, AdcIsStart);
	    	  OutUsartBuf_WrW(PosWr, GetOutPeriod());
	    	  OutUsartBuf_WrB(PosWr, ScanMode | (g_VVInfo.IsOutOn ? 0x80:0)); //(ScanMode & 0xf) | (g_VVInfo.IsOffOnScan ? 0x80:0) );
	    	  m_OutUsartBufferPosWr = PosWr;
	      }
	      else
	      {
	    	  g_Adc1Time = dT;
	    	  g_Adc1Len = g_AdcUseLen;
	    	  g_Adc1OutPeriod = GetOutPeriod();
	    	  uint32_t s = 0;
	    	  uint64_t d = 0;
        	  uint16_t vMin = 0xffff;
	          uint16_t vMax = 0;
	    	  const int N = g_AdcUseLen;
	    	  for (int i=0; i<N; i++)
	    	  {
	    		uint16_t v = g_Adc1ResBuf[i];
	    		if (vMin > v) vMin = v;
	    		if (vMax < v) vMax = v;
	    		s += v;
	    		d += ((uint32_t)v) * v;
	    	  };

	    	  uint16_t PosWr = m_OutUsartBufferPosWr;
	    	  m_OutUsartBuf2[PosWr++] = 0xcd;       if (PosWr >= OutUsartBuf2_NMax) PosWr = 0;
	    	  m_OutUsartBuf2[PosWr++] = 0xe2;       if (PosWr >= OutUsartBuf2_NMax) PosWr = 0;
	    	  OutUsartBuf_WrB(PosWr, AdcStartType);
	    	  OutUsartBuf_WrW(PosWr, N);
	    	  OutUsartBuf_WrD(PosWr, s);
	    	  OutUsartBuf_WrQ5(PosWr, d);
	    	  OutUsartBuf_WrW(PosWr, vMin);
	    	  OutUsartBuf_WrW(PosWr, vMax);
	    	  OutUsartBuf_WrB(PosWr, AdcIsStart);
	    	  OutUsartBuf_WrW(PosWr, GetOutPeriod());
	    	  OutUsartBuf_WrB(PosWr, ScanMode | (g_VVInfo.IsOutOn ? 0x80:0)); //(ScanMode & 0xf) | (g_VVInfo.IsOffOnScan ? 0x80:0) | (g_VVInfo.IsOutOn ? 0x40:0));
	    	  m_OutUsartBufferPosWr = PosWr;
	      };



	      if (AdcStartType == StartAdcType_VV_IU && ScanMode >= 1 && ScanMode <= 3 ) // сканирование
	      {
             int P = GetOutPeriod();
             if (ScanMode == 1 || ScanMode == 2)
             {
            	if ((g_VVInfo.StartScanMode & 0x70) == 0x10 && FindValue < VV_SredI)
            	{
            	  FindValue = VV_SredI; FindOutPeriod = P;
            	};
            	if ((g_VVInfo.StartScanMode & 0x70) == 0x20 && FindValue < VV_SredU)
            	{
            	  FindValue = VV_SredU; FindOutPeriod = P;
            	};
            	if ((g_VVInfo.StartScanMode & 0x70) == 0x30 && FindValue < VV_Pwr)
            	{
            	  FindValue = VV_Pwr; FindOutPeriod = P;
            	};
            	if ((g_VVInfo.StartScanMode & 0x70) == 0x40 && FindValue < 4096-PitI)
            	{
            	  FindValue = 4096-PitI; FindOutPeriod = P; // PitI - код, который убывает
            	};
             };

             if (ScanMode == 1)
             {
            	P -= 2;
            	if (P % 61 == 0) P -= 2;
            	if (P < g_VVInfo.DiaFM2)
            	{
            		ScanMode = 0;
            		if ((g_VVInfo.StartScanMode & 0x70) && FindOutPeriod)
            		{
            			SetOutPeriod(FindOutPeriod);

            			uint16_t PosWr = m_OutUsartBufferPosWr;
            			OutUsartBuf_WrB(PosWr, 0xcd);
            			OutUsartBuf_WrB(PosWr, 0xe7);
            			OutUsartBuf_WrB(PosWr, g_VVInfo.StartScanMode);
            			OutUsartBuf_WrD(PosWr, *(uint32_t *)&FindValue);
            			OutUsartBuf_WrD(PosWr, FindOutPeriod);
            			m_OutUsartBufferPosWr = PosWr;
            		};
            	}
            	else SetOutPeriod(P);
             };
             if (ScanMode == 2)
             {
            	P += 2;
            	if (P % 61 == 0) P += 2;
            	if (P > g_VVInfo.DiaFM1)
            	{
            	  ScanMode = 0;
          		  if ((g_VVInfo.StartScanMode & 0x70) && FindOutPeriod)
          		  {
          			SetOutPeriod(FindOutPeriod);

          			uint16_t PosWr = m_OutUsartBufferPosWr;
          			OutUsartBuf_WrB(PosWr, 0xcd);
          			OutUsartBuf_WrB(PosWr, 0xe7);
          			OutUsartBuf_WrB(PosWr, g_VVInfo.StartScanMode);
          			OutUsartBuf_WrD(PosWr, *(uint32_t *)&FindValue);
          			OutUsartBuf_WrD(PosWr, FindOutPeriod);
          			m_OutUsartBufferPosWr = PosWr;
          		  };
            	}
            	else SetOutPeriod(P);
             };
             if (ScanMode == 3)
             {
            	if (g_ScanDir)
                {
                  P -= 2;
                  if (P % 61 == 0) P -= 2;
                  if (P < g_VVInfo.DiaFM2) g_ScanDir=0; else SetOutPeriod(P);
                }
                else
                {
                	P += 2;
                	if (P % 61 == 0) P += 2;
                	if (P > g_VVInfo.DiaFM1) g_ScanDir=1; else SetOutPeriod(P);
                };
             };

             if (g_VVInfo.IsOffOnScan) // выключить, подождать и включить
             {
            	 LL_TIM_DisableAutomaticOutput(TIM1); // сброс бита AOE
            	 LL_TIM_DisableAllOutputs(TIM1); // сброс бита MOE

            	 uint32_t Delay = g_VVInfo.OffTime10MKS * 840;
            	 uint32_t t0 = TIM2->CNT;
                 while (TIM2->CNT - t0 < Delay) ;

                 if (g_VVInfo.IsOutOn) LL_TIM_EnableAutomaticOutput(TIM1); // уст. бита AOE, тогда MOE уст. при обновлении

                 if (!g_VVInfo.IsOutOn) // на случай, если успели выключить по usb
                 {
                	LL_TIM_DisableAutomaticOutput(TIM1);
                	LL_TIM_DisableAllOutputs(TIM1);
                 };

                 t0 = TIM2->CNT;
                 while (TIM2->CNT - t0 < 84e6*2/1000) ;
             };
	      };

	      if (AdcIsStart == 2)
	      {
			  g_IsMustStartIzmVV = 0; // отработал задание компа
			  g_IsMustLockVV = 1;     // и запретим портить буфер самостоятельными опросами
	      };
	      AdcIsStart = 0; // свободен
	      EP2_Send();

          if (MainLoop_StopOutOled == 0 && VV_OutPeriod)
          {
            // показ на экранчике высокого
            OledShowVV(VV_PikI, VV_PikU, VV_Pwr, VV_OutPeriod, VV_PereFlags);
          };

   	      if (MainLoop_StopOutOled == 0 && AdcStartType == StartAdcType_I)
   	      {
   	    	if (g_VVInfo.OledOptions & 8) OledShowPit(PitI);
   	      };

	   	};

	   	if (!AdcIsStart & !g_AdcDmaIsWork) // не запускался ещё - можно что-то запустить
	   	{
	   		if (g_IsMustStartIzmVV && (!ScanMode || g_IsMustStartIzmVV != StartAdcType_VV_IU))
	   		{
	   		  AdcStartType = g_IsMustStartIzmVV & (~0x40);
  	   		  uint16_t Len = sizeof(g_Adc1ResBuf) / 2;
	  		  if (AdcStartType < 128 && Len > GetOutPeriod()) Len = (Len / GetOutPeriod()) * GetOutPeriod();
	   		  StartAdc(AdcStartType, Len);
	     	  AdcIsStart = 2;
	     	  if (g_IsMustStartIzmVV & 0x40) // и просьба сразу включить выход
	     	  {
					g_VVInfo.IsOutOn = 1;
	     		 LL_TIM_EnableAutomaticOutput(TIM1);
	     	  };
	   		}
	   		else
	   		{
  	   		  varadc++;
	   		  if (varadc >= 4) varadc = 0;
	   		  if (varadc >= 2 && g_IsMustLockVV) varadc = 0;
	   		  if (varadc == 0) ProcessKbrd();
	   		  AdcStartType = StartAdcType_U;
	   		  if (varadc == 1) AdcStartType = StartAdcType_I;
	   		  if (varadc == 2) AdcStartType = StartAdcType_VV_I;
	   		  if (varadc == 3) AdcStartType = StartAdcType_VV_U;
	   		  if (varadc == 2) { AdcStartType = StartAdcType_VV_IU; varadc++; };
	   		  uint16_t Len = sizeof(g_Adc2ResBuf)/2;
	   		  if (AdcStartType < 128)
	   		  {
	   			// цифруем высокое напряжение
	   			Len = sizeof(g_Adc1ResBuf)/2;
	   			if (ScanMode)
	   		    {
	   				Len = 4200; // Урежем, 4200*61/84e6 = 3мс
	   				if (Len < GetOutPeriod()) Len = GetOutPeriod();
	   				if (Len > sizeof(g_Adc1ResBuf)/2) Len = sizeof(g_Adc1ResBuf)/2;
	   		    };
	   			if (Len > GetOutPeriod()) Len = (Len / GetOutPeriod()) * GetOutPeriod();
	   		  };
	   		  StartAdc(AdcStartType, Len);
	   		  AdcIsStart = 1;
	   		};
	   	};


	    if (MainLoop_IsGotoProgMode)
		{
		   const uint32_t T0 = TIM2->CNT;

		   if (MainLoop_StopOutOled == 0)
		   {
			   OledClear();
			   OledPrintX3(4,1, "ПЕРЕ");
		   };

		   while (TIM2->CNT - T0 < 84000000/10) ;  // подождем, пока подтверждение SETUP транзакции дойдет до компа

		   if (!(MainLoop_IsGotoProgMode & 128))
		   {
		     GetLockMonValue()[0] = 0xea39d2a1;  // в режим перепрошивки
		     GetLockMonValue()[1] = 0x99113344;
		   };
		   SCB->AIRCR = 0x05FA0004; // ресет
		   MainLoop_IsGotoProgMode = 0;
		};

	    if (MainLoop_StopOutOled == 0 && ((OledOptions ^ g_VVInfo.OledOptions) & (2+32))) // изменились настройки иниц. инд.
	    {
	      OledOptions = g_VVInfo.OledOptions;
	      OledInit(OledOptions & 2, OledOptions & 32);
	      OledClear();
	    };
	    if (MainLoop_StopOutOled == 1)
	    {
	    	MainLoop_StopOutOled = 2;
	    	OledClear();
	    	OledPrintX3(4,1,"КОМП");
	    };
	    if (MainLoop_StopOutOled == 3)
	    {
	    	OledClear();
	    	MainLoop_StopOutOled = 0;
	    };





    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 336, LL_RCC_PLLP_DIV_4);
  LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 336, LL_RCC_PLLQ_DIV_7);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(84000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA1   ------> ADC1_IN1
  PA2   ------> ADC1_IN2
  PA6   ------> ADC1_IN6
  PA7   ------> ADC1_IN7
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ADC1 DMA Init */

  /* ADC1 Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_4, LL_DMA_CHANNEL_0);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_4, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_4, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_4, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_4, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_4, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_4, LL_DMA_MDATAALIGN_WORD);

  LL_DMA_EnableFifoMode(DMA2, LL_DMA_STREAM_4);

  LL_DMA_SetFIFOThreshold(DMA2, LL_DMA_STREAM_4, LL_DMA_FIFOTHRESHOLD_FULL);

  LL_DMA_SetMemoryBurstxfer(DMA2, LL_DMA_STREAM_4, LL_DMA_MBURST_INC4);

  LL_DMA_SetPeriphBurstxfer(DMA2, LL_DMA_STREAM_4, LL_DMA_PBURST_SINGLE);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_ENABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM3_TRGO;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_5RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_SEQUENCE_CONV);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_3CYCLES);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_2);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_3CYCLES);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_6);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_56CYCLES);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_7);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_56CYCLES);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_5, LL_ADC_CHANNEL_TEMPSENSOR);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_56CYCLES);
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**SPI1 GPIO Configuration
  PB3   ------> SPI1_SCK
  PB4   ------> SPI1_MISO
  PB5   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1049;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_TOGGLE;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 5;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_OC3REF);
  LL_TIM_EnableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_ENABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_ENABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration
  PB15   ------> TIM1_CH3N
  PA10   ------> TIM1_CH3
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 4294967295;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 60;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR0);
  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_DISABLED);
  LL_TIM_DisableIT_TRIG(TIM3);
  LL_TIM_DisableDMAReq_TRIG(TIM3);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM4);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 12;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH2);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM4);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**TIM4 GPIO Configuration
  PB6   ------> TIM4_CH1
  PB7   ------> TIM4_CH2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  /* DMA interrupt init */
  /* DMA2_Stream4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(NikyLed1_GPIO_Port, NikyLed1_Pin);

  /**/
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5|NikyI2C_SDA_Pin
                          |NikyI2C_SCL_Pin);

  /**/
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2);

  /**/
  GPIO_InitStruct.Pin = NikyLed1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(NikyLed1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = NikyKey_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(NikyKey_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = NikyI2C_SDA_Pin|NikyI2C_SCL_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
