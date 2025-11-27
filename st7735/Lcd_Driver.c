/******************************************************************************
//STM32F103RB
//              GND   - 0VDC
//              VCC   - 5V or 3.3V
//              SCL   - PC8
//              SDA   - PC9
//              RES   - PC10
//              DC    - PB7
//              CS    - PB8
//			    BL	  - PB11
*******************************************************************************/

#include "main.h"
#include "Lcd_Driver.h"
// #include "LCD_Config.h"

// GPIO Initialsing
void LCD_GPIO_Init(void)
{
    /* Init in the GPIO.C */
}
/* Write Data via SPI Bus */
void SPI_WriteData(uint8_t Data)
{
    unsigned char i = 0;
    for (i = 8; i > 0; i--)
    {
        if (Data & 0x80)
            LCD_SDA_SET; /* Output data */
        else
            LCD_SDA_CLR;

        LCD_SCL_CLR;
        LCD_SCL_SET;
        Data <<= 1;
    }
}

/* Write Command to LCD */
void Lcd_WriteIndex(uint8_t Index)
{

    LCD_CS_CLR;
    LCD_RS_CLR;
    SPI_WriteData(Index);
    LCD_CS_SET;
}

/* Write Data to LCD */
void Lcd_WriteData(uint8_t Data)
{
    LCD_CS_CLR;
    LCD_RS_SET;
    SPI_WriteData(Data);
    LCD_CS_SET;
}
/* Write 16 Bits Data to LCD */
void LCD_WriteData_16Bit(uint16_t Data)
{
    LCD_CS_CLR;
    LCD_RS_SET;
    SPI_WriteData(Data >> 8); // High byte
    SPI_WriteData(Data);      // Low byte
    LCD_CS_SET;
}

void Lcd_WriteReg(uint8_t Index, uint8_t Data)
{
    Lcd_WriteIndex(Index);
    Lcd_WriteData(Data);
}

void Lcd_Reset(void)
{
    LCD_RST_CLR;
    HAL_Delay(100);
    LCD_RST_SET;
    HAL_Delay(50);
}

// LCD Init For 1.44Inch LCD Panel with ST7735R.
void Lcd_Init(void)
{
    LCD_GPIO_Init();
    Lcd_Reset(); /*  Reset before LCD Init. */

    /*    Power Consumption Mode
      6 level modes are defined they are in order of Maximum Power consumption to Minimum Power Consumption
        1. Normal Mode On (full diodesplay), Idle Mode Off, Sleep Out.
            In this mode, the display is able to show maximum 262,144 colors.
        2. Partial Mode On, Idle Mode Off, Sleep Out.
            In this mode part of the display is used with maximum 262,144 colors.
        3. Normal Mode On (full display), Idle Mode On, Sleep Out.
            In this mode, the full display area is used but with 8 colors.
        4. Partial Mode On, Idle Mode On, Sleep Out.
            In this mode, part of the display is used but with 8 colors.
        5. Sleep In Mode
            In this mode, the DC: DC converter, internal oscillator and panel driver circuit are stopped. Only the MCU
            interface and memory works with VDDI power supply. Contents of the memory are safe.
        6. Power Off Mode
            In this mode, both VDD and VDDI are removed
    */

    // LCD Init For 1.44Inch LCD Panel with ST7735R.
    //  SLPIN (10h): Sleep In
    //  SLPOUT (11h): Sleep Out
    //  IDMOFF (38h)  :Idle Mode Off
    //  IDMON (39h)   :Idle Mode On

    Lcd_WriteIndex(0x11); // Sleep exit
    HAL_Delay(120);

    // ST7735R Frame Rate
    /*  -Set the frame frequency of the full colors normal mode.
     *   - Frame rate=fosc/((RTNA x 2 + 40) x (LINE + FPA + BPA +2))
     *   - fosc = 850kHz
     *   - FPA > 0, BPA > 0
     *   - LINE = 128
     */
    //  FRMCTR1 (B1h): Frame Rate Control (In normal mode/ Full colors)
    Lcd_WriteIndex(0xB1);
    // Frame rate=90HHz
    // Lcd_WriteData(0x01); // RTNA
    // Lcd_WriteData(0x2C); // 44,  FPA
    // Lcd_WriteData(0x2D); // 45,  BPA

    // Frame rate=80HHz
    Lcd_WriteData(0x02); // RTNA
    Lcd_WriteData(0x35); // FPA
    Lcd_WriteData(0x36); // BPA

    // FRMCTR2 (B2h): Frame Rate Control (In Idle mode/ 8-colors)
    Lcd_WriteIndex(0xB2);
    Lcd_WriteData(0x02); // RTNA
    Lcd_WriteData(0x35); // FPA
    Lcd_WriteData(0x36); // BPA

    // FRMCTR3 (B3h): Frame Rate Control (In Partial mode/ full colors)
    Lcd_WriteIndex(0xB3);
    Lcd_WriteData(0x02); // RTNA
    Lcd_WriteData(0x35); // FPA
    Lcd_WriteData(0x36); // BPA
    Lcd_WriteData(0x02); // RTNA
    Lcd_WriteData(0x35); // FPA
    Lcd_WriteData(0x36); // BPA

    // INVCTR (B4h): Display Inversion Control
    Lcd_WriteIndex(0xB4);
    Lcd_WriteData(0x07);

    // ST7735R Power Sequence
    //  PWCTR1 (C0h): Power Control 1
    Lcd_WriteIndex(0xC0);
    Lcd_WriteData(0xA2);
    Lcd_WriteData(0x02);
    Lcd_WriteData(0x84);

    // PWCTR2 (C1h): Power Control 2
    Lcd_WriteIndex(0xC1);
    Lcd_WriteData(0xC5);

    // PWCTR3 (C2h): Power Control 3 (in Normal mode/ Full colors)
    Lcd_WriteIndex(0xC2);
    Lcd_WriteData(0x0A);
    Lcd_WriteData(0x00);

    // PWCTR4 (C3h): Power Control 4 (in Idle mode/ 8-colors)
    Lcd_WriteIndex(0xC3);
    Lcd_WriteData(0x8A);
    Lcd_WriteData(0x2A);

    //  PWCTR5 (C4h): Power Control 5 (in Partial mode/ full-colors)
    Lcd_WriteIndex(0xC4);
    Lcd_WriteData(0x8A);
    Lcd_WriteData(0xEE);

    //  VMCTR1 (C5h): VCOM Control 1
    Lcd_WriteIndex(0xC5); // VCOM
    Lcd_WriteData(0x0E);

    // MADCTL (36h): Memory Data Access Control
    /*   D7  D6  D5  D4  D3  D2  D1  D0
     *    MY  MX  MV  ML  RGB MH  -   -
     *    1   1   0   0   1   0   0   0     <= 0xC8
     *
     *  MY   : Row Address Order
     *  MX   : Column Address Order
     *  MV   : Row/Column exchange
     *  ML   : Vertical Refresh Order      0= top to buttom
     *  RGB  : RGB-BGR ORDER
     *  MH   : Horizontal Refresh Order    0= left to right
     */
    Lcd_WriteIndex(0x36); // MX, MY, RGB mode
    Lcd_WriteData(0xC8);

    // ST7735R Gamma Sequence
    //   GMCTRP1 (E0h): Gamma ('+'polarity) Correction Characteristics Setting
    Lcd_WriteIndex(0xe0);
    Lcd_WriteData(0x0f);
    Lcd_WriteData(0x1a);
    Lcd_WriteData(0x0f);
    Lcd_WriteData(0x18);
    Lcd_WriteData(0x2f);
    Lcd_WriteData(0x28);
    Lcd_WriteData(0x20);
    Lcd_WriteData(0x22);
    Lcd_WriteData(0x1f);
    Lcd_WriteData(0x1b);
    Lcd_WriteData(0x23);
    Lcd_WriteData(0x37);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x07);
    Lcd_WriteData(0x02);
    Lcd_WriteData(0x10);
    //  GMCTRN1 (E1h): Gamma '-' polarity Correction Characteristics Setting
    Lcd_WriteIndex(0xe1);
    Lcd_WriteData(0x0f);
    Lcd_WriteData(0x1b);
    Lcd_WriteData(0x0f);
    Lcd_WriteData(0x17);
    Lcd_WriteData(0x33);
    Lcd_WriteData(0x2c);
    Lcd_WriteData(0x29);
    Lcd_WriteData(0x2e);
    Lcd_WriteData(0x30);
    Lcd_WriteData(0x30);
    Lcd_WriteData(0x39);
    Lcd_WriteData(0x3f);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x07);
    Lcd_WriteData(0x03);
    Lcd_WriteData(0x10);

    // CASET (2Ah): Column Address Set
    Lcd_WriteIndex(0x2a);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x7f);
    // RASET (2Bh): Row Address Set
    Lcd_WriteIndex(0x2b);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x9f);

    Lcd_WriteIndex(0xF0); // Enable test command
    Lcd_WriteData(0x01);

    Lcd_WriteIndex(0xF6); // Disable ram power save mode
    Lcd_WriteData(0x00);

    //  COLMOD (3Ah): Interface Pixel Format
    Lcd_WriteIndex(0x3A); // 65k mode
    Lcd_WriteData(0x05);

    Lcd_WriteIndex(0x39); // Idle Mode On
    // DISPON (28h): Display Off
    // DISPON (29h): Display On
    Lcd_WriteIndex(0x29); // Display on

    LCD_LED_SET;
    Lcd_Clear(GRAY0);
}

/*************************************************
函数名：LCD_Set_Region
功能：设置lcd显示区域，在此区域写点数据自动换行
入口参数：xy起点和终点
返回值：无
*************************************************/
void Lcd_SetRegion(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{
    // CASET (2Ah): Column Address Set
    Lcd_WriteIndex(0x2a);
    Lcd_WriteData(0x00);
    Lcd_WriteData(x_start + 1);
    Lcd_WriteData(0x00);
    Lcd_WriteData(x_end + 1);
    // RASET (2Bh): Row Address Set
    Lcd_WriteIndex(0x2b);
    Lcd_WriteData(0x00);
    Lcd_WriteData(y_start + 1);
    Lcd_WriteData(0x00);
    Lcd_WriteData(y_end + 1);
    // RAMWR (2Ch): Memory Write
    Lcd_WriteIndex(0x2c);
}

/*************************************************
函数名：LCD_Set_XY
功能：设置lcd显示起始点
入口参数：xy坐标
返回值：无
*************************************************/
void Lcd_SetXY(uint16_t x, uint16_t y)
{
    Lcd_SetRegion(x, y, x, y);
}

/*************************************************
LCD_DrawPoint
*************************************************/
void Lcd_DrawPoint(uint16_t x, uint16_t y, uint16_t Data)
{
    if ((x >= LV_HOR_RES_MAX) || (y >= LV_VER_RES_MAX))
        return;
    Lcd_SetRegion(x, y, x + 1, y + 1);
    LCD_WriteData_16Bit(Data);
}

/*****************************************
 Read the point return the Color value
******************************************/
unsigned int Lcd_ReadPoint(uint16_t x, uint16_t y)
{
    unsigned int Data;
    Lcd_SetXY(x, y);

    // Lcd_ReadData();//丢掉无用字节
    // Data=Lcd_ReadData();
    Lcd_WriteData(Data);
    return Data;
}
/*************************************************
Name：Lcd_Clear
Function：Fill the Color in Full screen
Para：Fill the COLOR
Ret：none
*************************************************/
void Lcd_Clear(uint16_t Color)
{
    unsigned int i, m;
    Lcd_SetRegion(0, 0, LV_HOR_RES_MAX - 1, LV_VER_RES_MAX - 1);
    // RAMWR (2Ch): Memory Write
    Lcd_WriteIndex(0x2C);
    for (i = 0; i < LV_HOR_RES_MAX; i++)
        for (m = 0; m < LV_VER_RES_MAX; m++)
        {
            LCD_WriteData_16Bit(Color);
        }
}

/*************************************************
LCD_Color_Fill
*************************************************/
void Lcd_Color_Fill(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, lv_color_t *color)
{
    uint32_t y = 0;
    uint16_t height, width;
    width = x_end - x_start + 1;
    height = y_end - y_start + 1;

    Lcd_SetRegion(x_start, y_start, x_end, y_end);

    for (y = 0; y < width * height; y++)
    {
        LCD_WriteData_16Bit(color->full);
        color++;
    }
}