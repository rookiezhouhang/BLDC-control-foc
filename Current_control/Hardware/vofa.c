#include "vofa.h"
//#include "main.h"
#include "usart.h"
#include "string.h"

void Vofa_JustFloat(float *_data, uint8_t _num)
{
    uint8_t tempData[100];
    uint8_t temp_end[4] = {0, 0, 0x80, 0x7F};
    float temp_copy[_num];

    memcpy(&temp_copy, _data, sizeof(float) * _num);

    memcpy(tempData, (uint8_t *)&temp_copy, sizeof(temp_copy));
    memcpy(&tempData[_num * 4], &temp_end[0], 4);

    //....在此替换你的串口发送函数...........
    //HAL_UART_Transmit_DMA(&huart2, tempData, (_num + 1) * 4);
	HAL_UART_Transmit(&huart2,tempData,(_num + 1) * 4,10);
    //......................................
}
