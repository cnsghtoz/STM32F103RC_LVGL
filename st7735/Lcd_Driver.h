
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
#include "lvgl.h"


#define USE_HORIZONTAL 0 

#define RED        0xf800
#define GREEN      0x07e0
#define BLUE       0x001f
#define WHITE      0xffff
#define BLACK      0x0000
#define YELLOW     0xFFE0
#define VIOLET     0xF81F
#define ORANGE     0xFD20
#define BROWN      0x8200
#define PINK       0xF81F
#define DARKGREEN  0x01E0
#define LIGHTGREEN 0x87F0
#define GRAY0      0xEF7D // 3165 00110 001011 00101
#define GRAY1      0x8410 //      00000 000000 00000
#define GRAY2      0x4208 //      11111 111110 11111

# if 0
#define LCD_CTRLA  GPIOA // ����TFT���ݶ˿�
#define LCD_CTRLB  GPIOB // ����TFT���ݶ˿�
#define LCD_CTRLC  GPIOC // ����TFT���ݶ˿�

#define LCD_SCL    GPIO_Pin_8 // PC8      --->>TFT -- SCL/SCK
#define LCD_SDA    GPIO_Pin_9 // PC9 MOSI --->>TFT -- SDA/DIN
#define LCD_CS     GPIO_Pin_8 // MCU_PB8  --->>TFT -- CS/CE

#define LCD_LED    GPIO_Pin_11 // MCU_PB11--->> TFT --BL
#define LCD_RS     GPIO_Pin_7  // PB7     --->> TFT --RS/DC
#define LCD_RST    GPIO_Pin_10 // PC10    --->> TFT --RST

// #define LCD_CS_SET(x) LCD_CTRL->ODR=(LCD_CTRL->ODR&~LCD_CS)|(x ? LCD_CS:0)

// Һ�����ƿ���1�������궨��
#define LCD_SCL_SET LCD_CTRLC->BSRR = LCD_SCL
#define LCD_SDA_SET LCD_CTRLC->BSRR = LCD_SDA
#define LCD_CS_SET  LCD_CTRLB->BSRR = LCD_CS

#define LCD_LED_SET LCD_CTRLB->BSRR = LCD_LED
#define LCD_RS_SET  LCD_CTRLB->BSRR = LCD_RS
#define LCD_RST_SET LCD_CTRLC->BSRR = LCD_RST
// Һ�����ƿ���0�������궨��
#define LCD_SCL_CLR    LCD_CTRLC->BRR = LCD_SCL
#define LCD_SDA_CLR    LCD_CTRLC->BRR = LCD_SDA
#define LCD_CS_CLR     LCD_CTRLB->BRR = LCD_CS

#define LCD_LED_CLR    LCD_CTRLB->BRR = LCD_LED
#define LCD_RST_CLR    LCD_CTRLC->BRR = LCD_RST
#define LCD_RS_CLR     LCD_CTRLB->BRR = LCD_RS
# endif

#define LCD_SCL_SET HAL_GPIO_WritePin(LCD_SCL_GPIO_Port,LCD_SCL_Pin, GPIO_PIN_SET)   
#define LCD_SCL_CLR HAL_GPIO_WritePin(LCD_SCL_GPIO_Port,LCD_SCL_Pin, GPIO_PIN_RESET)   
#define LCD_SDA_SET HAL_GPIO_WritePin(LCD_SDA_GPIO_Port,LCD_SDA_Pin, GPIO_PIN_SET)   
#define LCD_SDA_CLR HAL_GPIO_WritePin(LCD_SDA_GPIO_Port,LCD_SDA_Pin, GPIO_PIN_RESET)   
#define LCD_CS_SET HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin, GPIO_PIN_SET)   
#define LCD_CS_CLR HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin, GPIO_PIN_RESET)   
#define LCD_LED_SET HAL_GPIO_WritePin(LCD_LED_GPIO_Port,LCD_LED_Pin, GPIO_PIN_SET)   
#define LCD_LED_CLR HAL_GPIO_WritePin(LCD_LED_GPIO_Port,LCD_LED_Pin, GPIO_PIN_RESET)   
#define LCD_RST_SET HAL_GPIO_WritePin(LCD_RES_GPIO_Port,LCD_RES_Pin, GPIO_PIN_SET)   
#define LCD_RST_CLR HAL_GPIO_WritePin(LCD_RES_GPIO_Port,LCD_RES_Pin, GPIO_PIN_RESET)   
#define LCD_RS_SET HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin, GPIO_PIN_SET)   
#define LCD_RS_CLR HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin, GPIO_PIN_RESET)   

#define LCD_DATAOUT(x) LCD_DATA->ODR = x; // �������
#define LCD_DATAIN     LCD_DATA->IDR;     // ��������

#define LCD_WR_DATA(data)  \
    {                      \
        LCD_RS_SET;        \
        LCD_CS_CLR;        \
        LCD_DATAOUT(data); \
        LCD_WR_CLR;        \
        LCD_WR_SET;        \
        LCD_CS_SET;        \
    }

void LCD_GPIO_Init(void);
void Lcd_WriteIndex(uint8_t Index);
void Lcd_WriteData(uint8_t Data);
void Lcd_WriteReg(uint8_t Index, uint8_t Data);
uint16_t Lcd_ReadReg(uint8_t LCD_Reg);
void Lcd_Reset(void);
void Lcd_Init(void);
void Lcd_Clear(uint16_t Color);
void Lcd_SetXY(uint16_t x, uint16_t y);
void Lcd_DrawPoint(uint16_t x, uint16_t y, uint16_t Data);
unsigned int Lcd_ReadPoint(uint16_t x, uint16_t y);
void Lcd_SetRegion(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end);
void LCD_WriteData_16Bit(uint16_t Data);
void Lcd_Color_Fill(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end,  lv_color_t *color);
