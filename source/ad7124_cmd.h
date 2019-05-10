#ifndef _AD7124_CMD_
#define _AD7124_CMD_

typedef void (*adc_callback)(void);
int32_t getAdcRaw(uint8_t ch);
void setAdcOffset(int32_t x, int32_t y);
int8_t ad7124cmd_startConversion(adc_callback f, int32_t* b);

#endif