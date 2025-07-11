#ifndef __ENCODER_H__
#define __ENCODER_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern int32_t encoder1_raw_count;
extern int32_t encoder2_raw_count;
extern int32_t encoder1_base_count;
extern int32_t encoder2_base_count;
extern int32_t encoder1_wrap_count;
extern int32_t encoder2_wrap_count;
extern int32_t x_total_ticks;
extern int32_t y_total_ticks;
extern int32_t x_total_ticks_shadow;
extern float x_position_units;
extern float y_position_units;

/* Exported functions prototypes ---------------------------------------------*/
void EncoderInit(void);
void EncoderTask(void *argument);

#endif /* __ENCODER_H__ */