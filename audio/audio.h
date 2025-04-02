#include <stdint.h>


void audioClkInit(int plln, int pllr, int i2s2div, int i2s2odd, int i2s3div, int i2s3odd);

void micGpioInit();
void micDmaInit();
void micDmaStart();
void micDmaStop();
int16_t* micGetPdmBuffer();

void spkGpioInit();
void spkDmaInit();
void spkDmaStart();
void spkDmaStop();
int16_t* spkGetSamplesBuffer();
void spkReturnSamplesBuffer(uint16_t audioBufferFilledLen);
int spkGetNumFreeSamplesBuffers();
int isSpkDmaRunning();
void spkCodecInit(void* callBackFunction);
void spkCodecSetVolume(int volume);
void spkCodecOn();
void spkCodecOff();
void (*spkCallBackFunction)(int16_t* audioBuffer, uint16_t* audioBufferLen);
