#include "oled.h"

#include <string.h>


/*
 * ============================================================
 * PRIVATE VARIABLES
 * ============================================================
 */

static I2C_HandleTypeDef *oled_i2c = NULL;

static uint8_t oled_buffer[
    OLED_WIDTH * OLED_HEIGHT / 8U
];

static uint8_t cursor_x = 0U;
static uint8_t cursor_y = 0U;


/*
 * ============================================================
 * 5x7 ASCII FONT
 *
 * ASCII 32 -> 90
 *
 * Covers:
 *
 * space
 * + - . :
 * 0-9
 * A-Z
 *
 * Enough for this robot status display.
 * ============================================================
 */

static const uint8_t font5x7[][5] =
{
    /* 32 ' ' */
    {0x00,0x00,0x00,0x00,0x00},

    /* 33 ! */
    {0x00,0x00,0x5F,0x00,0x00},

    /* 34 " */
    {0x00,0x07,0x00,0x07,0x00},

    /* 35 # */
    {0x14,0x7F,0x14,0x7F,0x14},

    /* 36 $ */
    {0x24,0x2A,0x7F,0x2A,0x12},

    /* 37 % */
    {0x23,0x13,0x08,0x64,0x62},

    /* 38 & */
    {0x36,0x49,0x55,0x22,0x50},

    /* 39 ' */
    {0x00,0x05,0x03,0x00,0x00},

    /* 40 ( */
    {0x00,0x1C,0x22,0x41,0x00},

    /* 41 ) */
    {0x00,0x41,0x22,0x1C,0x00},

    /* 42 * */
    {0x14,0x08,0x3E,0x08,0x14},

    /* 43 + */
    {0x08,0x08,0x3E,0x08,0x08},

    /* 44 , */
    {0x00,0x50,0x30,0x00,0x00},

    /* 45 - */
    {0x08,0x08,0x08,0x08,0x08},

    /* 46 . */
    {0x00,0x60,0x60,0x00,0x00},

    /* 47 / */
    {0x20,0x10,0x08,0x04,0x02},

    /* 48 0 */
    {0x3E,0x51,0x49,0x45,0x3E},

    /* 49 1 */
    {0x00,0x42,0x7F,0x40,0x00},

    /* 50 2 */
    {0x42,0x61,0x51,0x49,0x46},

    /* 51 3 */
    {0x21,0x41,0x45,0x4B,0x31},

    /* 52 4 */
    {0x18,0x14,0x12,0x7F,0x10},

    /* 53 5 */
    {0x27,0x45,0x45,0x45,0x39},

    /* 54 6 */
    {0x3C,0x4A,0x49,0x49,0x30},

    /* 55 7 */
    {0x01,0x71,0x09,0x05,0x03},

    /* 56 8 */
    {0x36,0x49,0x49,0x49,0x36},

    /* 57 9 */
    {0x06,0x49,0x49,0x29,0x1E},

    /* 58 : */
    {0x00,0x36,0x36,0x00,0x00},

    /* 59 ; */
    {0x00,0x56,0x36,0x00,0x00},

    /* 60 < */
    {0x08,0x14,0x22,0x41,0x00},

    /* 61 = */
    {0x14,0x14,0x14,0x14,0x14},

    /* 62 > */
    {0x00,0x41,0x22,0x14,0x08},

    /* 63 ? */
    {0x02,0x01,0x51,0x09,0x06},

    /* 64 @ */
    {0x32,0x49,0x79,0x41,0x3E},

    /* 65 A */
    {0x7E,0x11,0x11,0x11,0x7E},

    /* 66 B */
    {0x7F,0x49,0x49,0x49,0x36},

    /* 67 C */
    {0x3E,0x41,0x41,0x41,0x22},

    /* 68 D */
    {0x7F,0x41,0x41,0x22,0x1C},

    /* 69 E */
    {0x7F,0x49,0x49,0x49,0x41},

    /* 70 F */
    {0x7F,0x09,0x09,0x09,0x01},

    /* 71 G */
    {0x3E,0x41,0x49,0x49,0x7A},

    /* 72 H */
    {0x7F,0x08,0x08,0x08,0x7F},

    /* 73 I */
    {0x00,0x41,0x7F,0x41,0x00},

    /* 74 J */
    {0x20,0x40,0x41,0x3F,0x01},

    /* 75 K */
    {0x7F,0x08,0x14,0x22,0x41},

    /* 76 L */
    {0x7F,0x40,0x40,0x40,0x40},

    /* 77 M */
    {0x7F,0x02,0x0C,0x02,0x7F},

    /* 78 N */
    {0x7F,0x04,0x08,0x10,0x7F},

    /* 79 O */
    {0x3E,0x41,0x41,0x41,0x3E},

    /* 80 P */
    {0x7F,0x09,0x09,0x09,0x06},

    /* 81 Q */
    {0x3E,0x41,0x51,0x21,0x5E},

    /* 82 R */
    {0x7F,0x09,0x19,0x29,0x46},

    /* 83 S */
    {0x46,0x49,0x49,0x49,0x31},

    /* 84 T */
    {0x01,0x01,0x7F,0x01,0x01},

    /* 85 U */
    {0x3F,0x40,0x40,0x40,0x3F},

    /* 86 V */
    {0x1F,0x20,0x40,0x20,0x1F},

    /* 87 W */
    {0x7F,0x20,0x18,0x20,0x7F},

    /* 88 X */
    {0x63,0x14,0x08,0x14,0x63},

    /* 89 Y */
    {0x03,0x04,0x78,0x04,0x03},

    /* 90 Z */
    {0x61,0x51,0x49,0x45,0x43}
};


/*
 * ============================================================
 * LOW LEVEL
 * ============================================================
 */

static HAL_StatusTypeDef OLED_Command(
    uint8_t command)
{
    uint8_t packet[2];

    packet[0] = 0x00U;
    packet[1] = command;

    return HAL_I2C_Master_Transmit(
        oled_i2c,
        OLED_I2C_ADDR,
        packet,
        2U,
        100U
    );
}


/*
 * ============================================================
 * INITIALIZATION
 * ============================================================
 */

HAL_StatusTypeDef OLED_Init(
    I2C_HandleTypeDef *hi2c)
{
    oled_i2c = hi2c;

    HAL_Delay(100U);


    /*
     * Display OFF
     */
    if (OLED_Command(0xAEU) != HAL_OK)
        return HAL_ERROR;


    /*
     * Clock divide
     */
    OLED_Command(0xD5U);
    OLED_Command(0x80U);


    /*
     * Multiplex ratio = 64
     */
    OLED_Command(0xA8U);
    OLED_Command(0x3FU);


    /*
     * Display offset
     */
    OLED_Command(0xD3U);
    OLED_Command(0x00U);


    /*
     * Start line = 0
     */
    OLED_Command(0x40U);


    /*
     * Charge pump
     */
    OLED_Command(0x8DU);
    OLED_Command(0x14U);


    /*
     * Horizontal addressing mode
     */
    OLED_Command(0x20U);
    OLED_Command(0x00U);


    /*
     * Segment remap
     */
    OLED_Command(0xA1U);


    /*
     * COM scan direction
     */
    OLED_Command(0xC8U);


    /*
     * COM pins
     */
    OLED_Command(0xDAU);
    OLED_Command(0x12U);


    /*
     * Contrast
     */
    OLED_Command(0x81U);
    OLED_Command(0x7FU);


    /*
     * Pre-charge
     */
    OLED_Command(0xD9U);
    OLED_Command(0xF1U);


    /*
     * VCOM detect
     */
    OLED_Command(0xDBU);
    OLED_Command(0x40U);


    /*
     * Entire display follows RAM
     */
    OLED_Command(0xA4U);


    /*
     * Normal display
     */
    OLED_Command(0xA6U);


    /*
     * Display ON
     */
    OLED_Command(0xAFU);


    OLED_Clear();

    return OLED_Update();
}


/*
 * ============================================================
 * FRAME BUFFER
 * ============================================================
 */

void OLED_Clear(void)
{
    memset(
        oled_buffer,
        0,
        sizeof(oled_buffer)
    );

    cursor_x = 0U;
    cursor_y = 0U;
}


void OLED_SetPixel(
    uint8_t x,
    uint8_t y,
    uint8_t on)
{
    if ((x >= OLED_WIDTH) ||
        (y >= OLED_HEIGHT))
    {
        return;
    }

    uint16_t index =
        x +
        ((uint16_t)(y / 8U) *
         OLED_WIDTH);

    uint8_t mask =
        (uint8_t)(
            1U << (y & 7U)
        );

    if (on)
    {
        oled_buffer[index] |= mask;
    }
    else
    {
        oled_buffer[index] &= ~mask;
    }
}


void OLED_SetCursor(
    uint8_t x,
    uint8_t y)
{
    cursor_x = x;
    cursor_y = y;
}


/*
 * ============================================================
 * FONT
 * ============================================================
 */

void OLED_WriteChar(
    char c)
{
    /*
     * Lowercase -> uppercase.
     */
    if ((c >= 'a') &&
        (c <= 'z'))
    {
        c =
            (char)(
                c - 'a' + 'A'
            );
    }

    if ((c < 32) ||
        (c > 90))
    {
        c = '?';
    }


    uint8_t font_index =
        (uint8_t)(c - 32);


    for (uint8_t column = 0U;
         column < 5U;
         column++)
    {
        uint8_t bits =
            font5x7[
                font_index
            ][column];

        for (uint8_t row = 0U;
             row < 7U;
             row++)
        {
            if (bits &
                (1U << row))
            {
                OLED_SetPixel(
                    cursor_x + column,
                    cursor_y + row,
                    1U
                );
            }
        }
    }


    /*
     * 5 pixels character +
     * 1 pixel spacing.
     */
    cursor_x += 6U;
}


void OLED_WriteString(
    const char *str)
{
    if (str == NULL)
    {
        return;
    }

    while (*str != '\0')
    {
        OLED_WriteChar(*str);

        str++;
    }
}


/*
 * ============================================================
 * SCREEN UPDATE
 * ============================================================
 */

HAL_StatusTypeDef OLED_Update(void)
{
    /*
     * Set column address:
     * 0 -> 127
     */
    OLED_Command(0x21U);
    OLED_Command(0x00U);
    OLED_Command(0x7FU);


    /*
     * Set page address:
     * 0 -> 7
     */
    OLED_Command(0x22U);
    OLED_Command(0x00U);
    OLED_Command(0x07U);


    /*
     * HAL I2C transaction buffer.
     *
     * 1 control byte +
     * 1024 framebuffer bytes.
     */
    static uint8_t tx_buffer[
        1U +
        (OLED_WIDTH * OLED_HEIGHT / 8U)
    ];

    tx_buffer[0] = 0x40U;

    memcpy(
        &tx_buffer[1],
        oled_buffer,
        sizeof(oled_buffer)
    );


    return HAL_I2C_Master_Transmit(
        oled_i2c,
        OLED_I2C_ADDR,
        tx_buffer,
        sizeof(tx_buffer),
        100U
    );
}
