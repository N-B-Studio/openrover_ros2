#ifndef INC_OLED_H_
#define INC_OLED_H_

#include "main.h"
#include <stdint.h>

/*
 * ============================================================
 * SSD1315 / SSD1306 compatible 128x64 OLED
 * ============================================================
 */

#define OLED_WIDTH              128U
#define OLED_HEIGHT             64U

/*
 * 7-bit address normally 0x3C.
 *
 * STM32 HAL expects the address shifted left by one.
 */
#define OLED_I2C_ADDR           (0x3CU << 1)


/*
 * OLED driver API
 */
HAL_StatusTypeDef OLED_Init(
    I2C_HandleTypeDef *hi2c
);

HAL_StatusTypeDef OLED_Update(void);

void OLED_Clear(void);

void OLED_SetPixel(
    uint8_t x,
    uint8_t y,
    uint8_t on
);

void OLED_SetCursor(
    uint8_t x,
    uint8_t y
);

void OLED_WriteChar(
    char c
);

void OLED_WriteString(
    const char *str
);

#endif /* INC_OLED_H_ */
