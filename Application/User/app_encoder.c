#include "app_encoder.h"

extern TIM_HandleTypeDef htim2;

static EncoderAxis_t enc_x = {0};
static int32_t zero_ref_x = 0;

static void Encoder_InitAxis(EncoderAxis_t *axis, TIM_HandleTypeDef *htim)
{
    if ((axis == NULL) || (htim == NULL)) return;

    axis->htim = htim;
    axis->position = 0;

    HAL_TIM_Encoder_Start(axis->htim, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(axis->htim, 0);
    axis->last_count = (uint16_t)__HAL_TIM_GET_COUNTER(axis->htim);
}

static void Encoder_UpdateAxis(EncoderAxis_t *axis)
{
    uint16_t now;
    int16_t delta;

    if ((axis == NULL) || (axis->htim == NULL)) return;

    now = (uint16_t)__HAL_TIM_GET_COUNTER(axis->htim);
    delta = (int16_t)(now - axis->last_count);

    axis->position += (int32_t)delta;
    axis->last_count = now;
}

static int16_t ClampToInt16(int32_t value)
{
    if (value > 32767) return 32767;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

void APP_Encoders_Init(void)
{
    Encoder_InitAxis(&enc_x, &htim2);
    zero_ref_x = 0;
}

void APP_Encoders_Update(void)
{
    Encoder_UpdateAxis(&enc_x);
}

void APP_Encoders_ZeroFromXIndex(void)
{
    APP_Encoders_Update();
    zero_ref_x = enc_x.position;
}

/*
 * One-encoder relative-count packet:
 *   [0] 0xA5
 *   [1] E1 low byte
 *   [2] E1 high byte
 *   [3] XOR checksum over bytes 0..2
 */
void APP_Encoders_BuildRel16Packet(uint8_t *packet, uint16_t packet_len)
{
    uint8_t checksum = 0U;
    uint8_t i;
    int16_t x16;
    int32_t x32;

    if ((packet == NULL) || (packet_len < 4U)) return;

    x32 = enc_x.position - zero_ref_x;
    x16 = ClampToInt16(x32);

    packet[0] = 0xA5;
    packet[1] = (uint8_t)(x16 & 0xFF);
    packet[2] = (uint8_t)((x16 >> 8) & 0xFF);

    for (i = 0U; i < 3U; i++) checksum ^= packet[i];
    packet[3] = checksum;
}

void APP_BuildTimestampPacket(uint8_t *packet, uint16_t packet_len, uint32_t tick_ms)
{
    uint8_t checksum = 0U;
    uint8_t i;

    if ((packet == NULL) || (packet_len < 6U)) return;

    packet[0] = 0xAA;
    packet[1] = (uint8_t)(tick_ms & 0xFFU);
    packet[2] = (uint8_t)((tick_ms >> 8) & 0xFFU);
    packet[3] = (uint8_t)((tick_ms >> 16) & 0xFFU);
    packet[4] = (uint8_t)((tick_ms >> 24) & 0xFFU);

    for (i = 0U; i < 5U; i++) checksum ^= packet[i];
    packet[5] = checksum;
}

int32_t APP_Encoder_GetX(void)
{
    return enc_x.position - zero_ref_x;
}
