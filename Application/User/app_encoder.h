#ifndef APP_ENCODER_SINGLE_L151_H
#define APP_ENCODER_SINGLE_L151_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef struct
{
    TIM_HandleTypeDef *htim;
    int32_t position;
    uint16_t last_count;
} EncoderAxis_t;

void APP_Encoders_Init(void);
void APP_Encoders_Update(void);
void APP_Encoders_ZeroFromXIndex(void);
void APP_Encoders_BuildRel16Packet(uint8_t *packet, uint16_t packet_len);
void APP_BuildTimestampPacket(uint8_t *packet, uint16_t packet_len, uint32_t tick_ms);
int32_t APP_Encoder_GetX(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_ENCODER_SINGLE_L151_H */
