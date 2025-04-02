#include <stdlib.h>
#include "stm32f4xx.h"
#include "clock.h"
#include "audio.h"
#include "settings.h"


// SPK storage
const int16_t		spkNullBuffer[SPK_FRAME_SIZE*SPK_CHANNELS/8] = {0,}; // NullBuffer davam mensi (kratsie pauzy pri vypadkoch)
int16_t			spkBuffer[SPK_BUFFERS_NUM][SPK_FRAME_SIZE*SPK_CHANNELS];
uint16_t		spkBufferLen[SPK_BUFFERS_NUM];
volatile unsigned int	spkBufferReadIndex, spkBufferWriteIndex;
volatile int 		spkDmaRunning;

// MIC storage
int16_t 		micPdmBuffer[2][MIC_PDM_FRAME_SIZE];
const uint32_t 		micPdmBufferSize = MIC_PDM_FRAME_SIZE;
volatile uint32_t 	frameCount;
volatile uint32_t 	transferComplete;


void micGpioInit()
{
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);

    // Configure I2S2 SD pin - PC3
    GPIOC->MODER &= ~(GPIO_MODER_MODE3);
    GPIOC->MODER |= GPIO_MODER_MODE3_1; // AF mode
    GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED3_1; // high speed
    GPIOC->OTYPER |= (0x0U << GPIO_OTYPER_OT3_Pos);
    GPIOC->PUPDR |= (0x0U << GPIO_PUPDR_PUPD3_Pos);
    GPIOC->AFR[0] |= (0x5U << GPIO_AFRL_AFSEL3_Pos); // AF5 for PC3

    // Configure I2S2 CK - PB10
    GPIOB->MODER   &= ~GPIO_MODER_MODE10;
    GPIOB->MODER   |=  GPIO_MODER_MODE10_1; // PB10 in AF mode
    GPIOB->OSPEEDR |=  GPIO_OSPEEDR_OSPEED10_1; // high speed
    GPIOB->OTYPER  |=  0x0U << GPIO_OTYPER_OT10_Pos;
    GPIOB->PUPDR   |=  0x0U << GPIO_PUPDR_PUPD10_Pos;
    GPIOB->AFR[1]  |=  0x5U << GPIO_AFRH_AFSEL10_Pos; // AF5 for PB10
}


void spkGpioInit()
{
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN);

    // Configure reset pin - PD4
    GPIOD->MODER &= ~GPIO_MODER_MODE4;
    GPIOD->MODER |= GPIO_MODER_MODE4_0;
    //GPIOD->OSPEEDR |= (0x2U << GPIO_OSPEEDR_OSPEED4_Pos);
    GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEED4_1; // high speed
    //GPIOD->OTYPER |= (0x0U << GPIO_OTYPER_OT4_Pos);
    GPIOD->OTYPER &= ~GPIO_OTYPER_OT4;
    //GPIOD->PUPDR |= (0x0U << GPIO_PUPDR_PUPD4_Pos);
    GPIOD->PUPDR &= ~GPIO_PUPDR_PUPD4;

    // Configure I2C SCL and SDA pins - PB6,9
    GPIOB->MODER   &= ~(GPIO_MODER_MODE6              | GPIO_MODER_MODE9);
    GPIOB->MODER   |=  (GPIO_MODER_MODE6_1            | GPIO_MODER_MODE9_1); // PB6,9 in AF mode
    GPIOB->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED6_1        | GPIO_OSPEEDR_OSPEED9_1); // high speed
    GPIOB->OTYPER  |=  (0x1U << GPIO_OTYPER_OT6_Pos)  | (0x1U << GPIO_OTYPER_OT9_Pos);
    GPIOB->PUPDR   |=  (0x0U << GPIO_PUPDR_PUPD6_Pos) | (0x0U << GPIO_PUPDR_PUPD9_Pos);
    GPIOB->AFR[0]  |=  (0x4U << GPIO_AFRL_AFSEL6_Pos);                                  // AF4 for PB6
    GPIOB->AFR[1]  |=                                   (0x4U << GPIO_AFRH_AFSEL9_Pos); // AF4 for PB9

    // Configure I2S MCK, SCK, SD pins - PC7,10,12
    GPIOC->MODER &= ~(GPIO_MODER_MODE7 | GPIO_MODER_MODE10 | GPIO_MODER_MODE12);
    GPIOC->MODER |= GPIO_MODER_MODE7_1 | GPIO_MODER_MODE10_1 | GPIO_MODER_MODE12_1; // AF mode
    GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED7_1 | GPIO_OSPEEDR_OSPEED10_1 | GPIO_OSPEEDR_OSPEED12_1; // high speed
    GPIOC->OTYPER |= (0x0U << GPIO_OTYPER_OT7_Pos) | (0x0U << GPIO_OTYPER_OT10_Pos) | (0x0U << GPIO_OTYPER_OT12_Pos);
    GPIOC->PUPDR |= (0x0U << GPIO_PUPDR_PUPD7_Pos) | (0x0U << GPIO_PUPDR_PUPD10_Pos) | (0x0U << GPIO_PUPDR_PUPD12_Pos);
    GPIOC->AFR[0] |= (0x6U << GPIO_AFRL_AFSEL7_Pos); // AF6 for PC7
    GPIOC->AFR[1] |= (0x6U << GPIO_AFRH_AFSEL10_Pos) | (0x6U << GPIO_AFRH_AFSEL12_Pos); // AF6 for PC10,12

    // Configure I2S WS pin - PA4
    GPIOA->MODER &= ~GPIO_MODER_MODE4;
    GPIOA->MODER |= GPIO_MODER_MODE4_1;
    //GPIOA->OSPEEDR |= (0x2U << GPIO_OSPEEDR_OSPEED4_Pos);
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED4_1; // high speed
    //GPIOA->OTYPER |= (0x0U << GPIO_OTYPER_OT4_Pos);
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT4;
    //GPIOA->PUPDR |= (0x0U << GPIO_PUPDR_PUPD4_Pos);
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD4;
    GPIOA->AFR[0] |= (0x6U << GPIO_AFRL_AFSEL4_Pos); // AF6 for PA4

    // Reset the codec.
    GPIOD->BSRR = GPIO_BSRR_BR4;  // PD4 low
    delay_ms(10);
    GPIOD->BSRR = GPIO_BSRR_BS4;  // PD4 high
}


void audioClkInit(int plln, int pllr, int i2s2div, int i2s2odd, int i2s3div, int i2s3odd)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; // DMA muselo byt tu !!!
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;


    /* Disable I2S_PLL */
    RCC->CR &= ~RCC_CR_PLLI2SON;
    while((RCC->CR & RCC_CR_PLLI2SRDY) == RCC_CR_PLLI2SRDY);

    /*  Configure I2S PLL
     * F_plli2s_clk_input - je vstup z krystalu
     * F_vco_clk           = F_plli2s_clk_input * (PLLI2SN/PLLM) (malo by byt 100 - 432 MHz) - VCO frequency
     * F_plli2s_clk_output = F_vco_clk / PLLI2SR                 (malo by byt     <= 192 MHz) - I2S clock frequency

     * PLLI2SN - 2,3,...432,434,...510                           (asi musi platit : 50 <= PLLI2SN <= 432)
                                                                 (50 <= PLLI2SN <= 99 je mozne len pre F_plli2s_clk_input > 1Mhz)
     * PLLI2SR - 2...7
    */

    RCC->PLLI2SCFGR &= ~( RCC_PLLI2SCFGR_PLLI2SN_Msk |
                          RCC_PLLI2SCFGR_PLLI2SR_Msk );   /* clear affected bits */
    RCC->PLLI2SCFGR |= ( plln << RCC_PLLI2SCFGR_PLLI2SN_Pos |
                         pllr << RCC_PLLI2SCFGR_PLLI2SR_Pos );

    /* Enable I2S_PLL */
    RCC->CR |= RCC_CR_PLLI2SON;
    while((RCC->CR & RCC_CR_PLLI2SRDY) == 0);

    // Configure I2S2 - Microphone
    // 258,6,42,0: (2*258/6) = 86.0 / (2*42 + 0) = 1023.8095 kHz
    // co je 1.024 [Msamples/s] (1sample=1bit) -> na 10ms zaznamu treba int16_t data[640]
    SPI2->I2SPR = ( i2s2div << SPI_I2SPR_I2SDIV_Pos |
                    i2s2odd << SPI_I2SPR_ODD_Pos );    // Master CLK disabled

    // Master receive, Phillips mode, 16 bit values, clock polarity low, enable
    SPI2->I2SCFGR = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_0 | SPI_I2SCFGR_I2SCFG_1 /*| SPI_I2SCFGR_CKPOL*/;
    SPI2->I2SCFGR |= SPI_I2SCFGR_I2SE; // enable periph

    // Configure I2S3 - Speaker
    // 256,5,12,1: (2*256/5) = 102.4 / 32*(2*12+1)*8 = 16.000 kHz
    // 258,6,10,1: (2*258/6) = 86.0  / 32*(2*10+1)*8 = 15.997 kHz
    SPI3->I2SPR = ( i2s3div << SPI_I2SPR_I2SDIV_Pos |
                    i2s3odd << SPI_I2SPR_ODD_Pos    |
                       0x01 << SPI_I2SPR_MCKOE_Pos );  // Master CLK enable

    // Master transmitter, Phillips mode, 16 bit values, clock polarity low, enable.
    SPI3->I2SCFGR = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_1;
    SPI3->I2SCFGR |= SPI_I2SCFGR_I2SE; // enable periph
}


void spkCodecWriteI2cRegister(uint8_t address, uint8_t value)
{
    while (I2C1->SR2 & I2C_SR2_BUSY );

    I2C1->CR1 |= I2C_CR1_START;           // send start
    while (!(I2C1->SR1 & I2C_SR1_SB ));

    I2C1->DR = 0x94;
    while (!(I2C1->SR1 & I2C_SR1_ADDR )); // wait for master mode
    I2C1->SR2;

    I2C1->DR = address;                   // send address
    while (!(I2C1->SR1 & I2C_SR1_TXE ));

    I2C1->DR = value;                     // send value

    while (!(I2C1 ->SR1 & I2C_SR1_BTF )); // wait for finish
    I2C1 ->CR1 |= I2C_CR1_STOP;           // send stop
}


void spkCodecInit(void* callBackFunction)
{
    const uint32_t pclk1 = 42000000;
    const uint32_t i2c_speed = 100000;

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Reset I2C.
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    // Configure I2C.
    I2C1->CR2 = pclk1 / 1000000; // Configure frequency and disable interrupts and DMA.
    I2C1->OAR1 = I2C_OAR1_ADDMODE | 0x33;

    // Configure I2C speed in standard mode.
    int ccrspeed = pclk1 / (i2c_speed * 2);
    if (ccrspeed < 4)
        ccrspeed = 4;

    I2C1->CCR = ccrspeed;
    I2C1->TRISE = pclk1 / 1000000 + 1;
    I2C1->CR1 = I2C_CR1_ACK | I2C_CR1_PE; // Enable and configure the I2C peripheral.

    // Configure codec.
    spkCodecWriteI2cRegister(0x02, 0x01); // Keep codec powered off.
    spkCodecWriteI2cRegister(0x04, 0xaf); // SPK always off and HP always on.
    spkCodecWriteI2cRegister(0x05, 0x81); // Clock configuration: Auto detection.
    spkCodecWriteI2cRegister(0x06, 0x04); // Set slave mode and Philips audio standard.
    spkCodecWriteI2cRegister(0x02, 0x9e); // Power on the codec

    // Configure codec for fast shutdown.
    spkCodecWriteI2cRegister(0x0a, 0x00); // Disable the analog soft ramp.
    spkCodecWriteI2cRegister(0x0e, 0x04); // Disable the digital soft ramp.
    spkCodecWriteI2cRegister(0x27, 0x00); // Disable the limiter attack level.
    spkCodecWriteI2cRegister(0x1f, 0x0f); // Adjust bass and treble levels.
    spkCodecWriteI2cRegister(0x1a, 0x0a); // Adjust PCM volume level.
    spkCodecWriteI2cRegister(0x1b, 0x0a);

    spkBufferReadIndex = spkBufferWriteIndex = 0;  // set empty buffers
    spkCallBackFunction = callBackFunction;
    spkDmaRunning = -1;  // codec initialized but not running
}


void spkCodecSetVolume(int volume)
{
    spkCodecWriteI2cRegister(0x20, (volume + 0x19) & 0xff);
    spkCodecWriteI2cRegister(0x21, (volume + 0x19) & 0xff);
}


void spkCodecOn()
{
    spkCodecWriteI2cRegister(0x02, 0x9e);
    // Master transmitter, Phillips mode, 16 bit values, clock polarity low, enable.
    SPI3->I2SCFGR = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_1 | SPI_I2SCFGR_I2SE;
}


void spkCodecOff()
{
    spkCodecWriteI2cRegister(0x02, 0x01);
    SPI3->I2SCFGR = 0;
}



int16_t* spkGetSamplesBuffer()
{
    if (spkBufferWriteIndex - spkBufferReadIndex == SPK_BUFFERS_NUM)
        return NULL; // fifo full

    return spkBuffer[spkBufferWriteIndex % SPK_BUFFERS_NUM];
}


void spkReturnSamplesBuffer(uint16_t spkBufferFilledLen)
{
    spkBufferLen[spkBufferWriteIndex % SPK_BUFFERS_NUM] = spkBufferFilledLen;
    spkBufferWriteIndex++;
}


int spkGetNumFreeSamplesBuffers()
{
    return SPK_BUFFERS_NUM - (spkBufferWriteIndex - spkBufferReadIndex);
}


void micDmaInit()
{
    DMA1_Stream3 ->CR = 0<<DMA_SxCR_CHSEL_Pos | // Channel 0
                        DMA_SxCR_PL_0         | // Priority 1
                        DMA_SxCR_PSIZE_0      | // PSIZE = 16 bit
                        DMA_SxCR_MSIZE_0      | // MSIZE = 16 bit
                        DMA_SxCR_MINC         | // Increase memory address
                        DMA_SxCR_DBM          | // double buffer mode
                        0;                      // Peripherial to Memory

    DMA1_Stream3 ->PAR = (uint32_t)&SPI2->DR;
    DMA1_Stream3->M0AR = (uint32_t)&micPdmBuffer[0][0];
    DMA1_Stream3->M1AR = (uint32_t)&micPdmBuffer[1][0];
    DMA1_Stream3->NDTR = micPdmBufferSize;
    DMA1_Stream3 ->FCR = DMA_SxFCR_DMDIS;
    DMA1_Stream3 ->CR &= ~DMA_SxCR_CT; // start transfer into M0AR
    DMA1_Stream3 ->CR |= DMA_SxCR_TCIE;
    DMA1->HIFCR |= DMA_LIFCR_CTCIF3; // Clear interrupt flag.
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    NVIC_SetPriority(DMA1_Stream3_IRQn, 4);
    SPI2->CR2 |= SPI_CR2_RXDMAEN; // Enable I2S RX DMA request.
}


void micDmaStart()
{
    DMA1_Stream3->CR  &= ~DMA_SxCR_CT; // start transfer into M0AR
    DMA1_Stream3->CR  |= DMA_SxCR_EN;  // start DMA

    frameCount = 0;
    transferComplete = 0;
}


void micDmaStop()
{
    DMA1_Stream3->CR &= ~DMA_SxCR_EN;      // Disable DMA stream.
    while(DMA1_Stream3->CR & DMA_SxCR_EN); // Wait for DMA stream to stop.
}


void spkDmaInit()
{
    DMA1_Stream7 ->CR = 0<<DMA_SxCR_CHSEL_Pos | // Channel 0
                        DMA_SxCR_PL_0         | // Priority 1
                        DMA_SxCR_PSIZE_0      | // PSIZE = 16 bit
                        DMA_SxCR_MSIZE_0      | // MSIZE = 16 bit
                        DMA_SxCR_MINC         | // Increase memory address
                        DMA_SxCR_DIR_0;         // Memory to peripheral

    DMA1_Stream7 ->PAR = (uint32_t)&SPI3->DR;
    DMA1_Stream7 ->FCR = DMA_SxFCR_DMDIS;
    DMA1_Stream7 ->CR |= DMA_SxCR_TCIE;
    DMA1->HIFCR |= DMA_HIFCR_CTCIF7; // Clear interrupt flag.
    NVIC_EnableIRQ(DMA1_Stream7_IRQn);
    NVIC_SetPriority(DMA1_Stream7_IRQn, 4);
    SPI3->CR2 |= SPI_CR2_TXDMAEN; // Enable I2S TX DMA request.
}


void spkDmaStart()
{
    DMA1_Stream7->NDTR = spkBufferLen[spkBufferReadIndex % SPK_BUFFERS_NUM];
    DMA1_Stream7->M0AR = (uint32_t)&spkBuffer[spkBufferReadIndex % SPK_BUFFERS_NUM][0];
    DMA1_Stream7->CR |= DMA_SxCR_EN;            // start DMA
    spkDmaRunning = 1;
}


void spkDmaStop()
{
    spkBufferReadIndex = spkBufferWriteIndex = 0;  // set empty buffers
    spkDmaRunning = -1;                                // set spkDma inactive flag
}


int16_t* micGetPdmBuffer()
{
    uint32_t currentTarget = DMA1_Stream3->CR & DMA_SxCR_CT;
    return (int16_t*)&micPdmBuffer[!currentTarget][0];
}


int isSpkDmaRunning()
{
    return spkDmaRunning;
}


// SPK
void DMA1_Stream7_IRQHandler()
{
    DMA1->HIFCR |= (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7);  // Clear interrupt flag + other flags

    if (spkDmaRunning == 1)  // audio buffer played so increment fifo readIndex
        spkBufferReadIndex++;
    else if (spkDmaRunning == -1)  // spkDmaStop() called
        return;

    if (spkBufferReadIndex == spkBufferWriteIndex)  // fifo empty -> play silence
        spkDmaRunning = 0;

    if (spkDmaRunning == 1) {  // play next audio buffer from fifo
        spkDmaStart();

        if (spkCallBackFunction) {  // call fifo buffer fill function if set
            spkCallBackFunction(&spkBuffer[spkBufferWriteIndex % SPK_BUFFERS_NUM][0],
                                &spkBufferLen[spkBufferWriteIndex % SPK_BUFFERS_NUM]);
            spkBufferWriteIndex++;
        }
    } else if (spkDmaRunning == 0) {  // fifo empty -> play silence
        DMA1_Stream7->NDTR = sizeof(spkNullBuffer)/sizeof(spkNullBuffer[0]);
        DMA1_Stream7->M0AR = (uint32_t)&spkNullBuffer[0];
        DMA1_Stream7->CR |= DMA_SxCR_EN;            // start spkNullBuffer DMA transfer
    }
}


// MIC
void DMA1_Stream3_IRQHandler()
{
    DMA1->LIFCR |= (DMA_LIFCR_CTCIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3);  // Clear interrupt flag + other flags
    frameCount++;
    transferComplete = 1;
}
