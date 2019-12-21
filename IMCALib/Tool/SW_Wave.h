#ifndef __SHANWAI_WAVE_H
#define __SHANWAI_WAVE_H

#include "mytype.h"



/*串口发送数据*/
void UsartSendData(uint8_t *tx_buf, uint8_t len);
/*发送数据到山外多功能调试助手中的虚拟示波器进行波形显示*/
void SwSendWare(uint8_t *wareaddr, int16_t waresize);
/*更新发送到山外多功能调试助手的数据*/
void SwDataWaveUpdate(void);

/*********************************************************不同数据类型发送函数***********************************************************/
/*发送四位float型数据到山外多功能调试助手*/
void SwDataSendFloat(float data1, float data2, float data3, float data4);
/*发送八位float型数据到山外多功能调试助手*/
void SwDataSendFloatPro(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8);
/*发送四位int8_t型数据到山外多功能调试助手*/
void SwDataSendInt8(int8_t data1, int8_t data2, int8_t data3, int8_t data4);
/*发送四位uint8_t型数据到山外多功能调试助手*/
void SwDataSendUint8(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4);
/*发送四位int16_t型数据到山外多功能调试助手*/
void SwDataSendInt16(int16_t data1, int16_t data2, int16_t data3, int16_t data4);
/*发送四位uint16_t型数据到山外多功能调试助手*/
void SwDataSendUint16(uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4);
/*发送四位int32_t型数据到山外多功能调试助手*/
void SwDataSendInt32(int32_t data1, int32_t data2, int32_t data3, int32_t data4);
/*发送四位uint32_t型数据到山外多功能调试助手*/
void SwDataSendUint32(uint32_t data1, uint32_t data2, uint32_t data3, uint32_t data4);

/*****************************************************************END********************************************************************/


#endif
