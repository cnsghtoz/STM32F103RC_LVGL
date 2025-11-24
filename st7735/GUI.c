#include "main.h"
#include "Lcd_Driver.h"
#include "GUI.h"
#include "font.h"
#include "math.h"

// ��ILI93xx����������ΪGBR��ʽ��������д���ʱ��ΪRGB��ʽ��
// ͨ���ú���ת��
// c:GBR��ʽ����ɫֵ
// ����ֵ��RGB��ʽ����ɫֵ
uint16_t LCD_BGR2RGB(uint16_t c)
{
    uint16_t r, g, b, rgb;
    b = (c >> 0) & 0x1f;
    g = (c >> 5) & 0x3f;
    r = (c >> 11) & 0x1f;
    rgb = (b << 11) + (g << 5) + (r << 0);
    return (rgb);
}

void Gui_Circle(uint16_t X, uint16_t Y, uint16_t R, uint16_t fc)
{ // Bresenham�㷨
    unsigned short a, b;
    int c;
    a = 0;
    b = R;
    c = 3 - 2 * R;
    while (a < b)
    {
        Lcd_DrawPoint(X + a, Y + b, fc); //        7
        Lcd_DrawPoint(X - a, Y + b, fc); //        6
        Lcd_DrawPoint(X + a, Y - b, fc); //        2
        Lcd_DrawPoint(X - a, Y - b, fc); //        3
        Lcd_DrawPoint(X + b, Y + a, fc); //        8
        Lcd_DrawPoint(X - b, Y + a, fc); //        5
        Lcd_DrawPoint(X + b, Y - a, fc); //        1
        Lcd_DrawPoint(X - b, Y - a, fc); //        4

        if (c < 0)
            c = c + 4 * a + 6;
        else
        {
            c = c + 4 * (a - b) + 10;
            b -= 1;
        }
        a += 1;
    }
    if (a == b)
    {
        Lcd_DrawPoint(X + a, Y + b, fc);
        Lcd_DrawPoint(X + a, Y + b, fc);
        Lcd_DrawPoint(X + a, Y - b, fc);
        Lcd_DrawPoint(X - a, Y - b, fc);
        Lcd_DrawPoint(X + b, Y + a, fc);
        Lcd_DrawPoint(X - b, Y + a, fc);
        Lcd_DrawPoint(X + b, Y - a, fc);
        Lcd_DrawPoint(X - b, Y - a, fc);
    }
}
// ���ߺ�����ʹ��Bresenham �����㷨
void Gui_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t Color)
{
    int dx,  // difference in x's
        dy,  // difference in y's
        dx2, // dx,dy * 2
        dy2,
        x_inc, // amount in pixel space to move during drawing
        y_inc, // amount in pixel space to move during drawing
        error, // the discriminant i.e. error i.e. decision variable
        index; // used for looping

    Lcd_SetXY(x0, y0);
    dx = x1 - x0; // ����x����
    dy = y1 - y0; // ����y����

    if (dx >= 0)
    {
        x_inc = 1;
    }
    else
    {
        x_inc = -1;
        dx = -dx;
    }

    if (dy >= 0)
    {
        y_inc = 1;
    }
    else
    {
        y_inc = -1;
        dy = -dy;
    }

    dx2 = dx << 1;
    dy2 = dy << 1;

    if (dx > dy) // x�������y���룬��ôÿ��x����ֻ��һ���㣬ÿ��y���������ɸ���
    {            // ���ߵĵ�������x���룬��x���������
        // initialize error term
        error = dy2 - dx;

        // draw the line
        for (index = 0; index <= dx; index++) // Ҫ���ĵ������ᳬ��x����
        {
            // ����
            Lcd_DrawPoint(x0, y0, Color);

            // test if error has overflowed
            if (error >= 0) // �Ƿ���Ҫ����y����ֵ
            {
                error -= dx2;

                // move to next line
                y0 += y_inc; // ����y����ֵ
            } // end if error overflowed

            // adjust the error term
            error += dy2;

            // move to the next pixel
            x0 += x_inc; // x����ֵÿ�λ���󶼵���1
        } // end for
    } // end if |slope| <= 1
    else // y�����x�ᣬ��ÿ��y����ֻ��һ���㣬x�����ɸ���
    {    // ��y��Ϊ��������
        // initialize error term
        error = dx2 - dy;

        // draw the line
        for (index = 0; index <= dy; index++)
        {
            // set the pixel
            Lcd_DrawPoint(x0, y0, Color);

            // test if error overflowed
            if (error >= 0)
            {
                error -= dy2;

                // move to next line
                x0 += x_inc;
            } // end if error overflowed

            // adjust the error term
            error += dx2;

            // move to the next pixel
            y0 += y_inc;
        } // end for
    } // end else |slope| > 1
}

void Gui_box(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t bc)
{
    /*     Gui_DrawLine(x, y, x + w, y, 0xEF7D);
        Gui_DrawLine(x + w - 1, y + 1, x + w - 1, y + 1 + h, 0x2965);
        Gui_DrawLine(x, y + h, x + w, y + h, 0x2965);
        Gui_DrawLine(x, y, x, y + h, 0xEF7D);
        Gui_DrawLine(x + 1, y + 1, x + 1 + w - 2, y + 1 + h - 2, bc); */

    Gui_DrawLine(x, y, x + w, y, bc);
    Gui_DrawLine(x + w - 1, y + 1, x + w - 1, y + 1 + h, bc);
    Gui_DrawLine(x, y + h, x + w, y + h, bc);
    Gui_DrawLine(x, y, x, y + h, bc);
    Gui_DrawLine(x + 1, y + 1, x + 1 + w - 2, y + 1 + h - 2, bc);
}

void Gui_box2(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t mode)
{
    if (mode == 0)
    {
        Gui_DrawLine(x, y, x + w, y, 0xEF7D);
        Gui_DrawLine(x + w - 1, y + 1, x + w - 1, y + 1 + h, 0x2965);
        Gui_DrawLine(x, y + h, x + w, y + h, 0x2965);
        Gui_DrawLine(x, y, x, y + h, 0xEF7D);
    }
    if (mode == 1)
    {
        Gui_DrawLine(x, y, x + w, y, 0x2965);
        Gui_DrawLine(x + w - 1, y + 1, x + w - 1, y + 1 + h, 0xEF7D);
        Gui_DrawLine(x, y + h, x + w, y + h, 0xEF7D);
        Gui_DrawLine(x, y, x, y + h, 0x2965);
    }
    if (mode == 2)
    {
        Gui_DrawLine(x, y, x + w, y, 0xffff);
        Gui_DrawLine(x + w - 1, y + 1, x + w - 1, y + 1 + h, 0xffff);
        Gui_DrawLine(x, y + h, x + w, y + h, 0xffff);
        Gui_DrawLine(x, y, x, y + h, 0xffff);
    }
}

/**************************************************************************************
��������: ����Ļ��ʾһ͹��İ�ť��
��    ��: uint16_t x1,y1,x2,y2 ��ť�����ϽǺ����½�����
��    ��: ��
**************************************************************************************/
void DisplayButtonDown(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    Gui_DrawLine(x1, y1, x2, y1, GRAY2);             // H
    Gui_DrawLine(x1 + 1, y1 + 1, x2, y1 + 1, GRAY1); // H
    Gui_DrawLine(x1, y1, x1, y2, GRAY2);             // V
    Gui_DrawLine(x1 + 1, y1 + 1, x1 + 1, y2, GRAY1); // V
    Gui_DrawLine(x1, y2, x2, y2, WHITE);             // H
    Gui_DrawLine(x2, y1, x2, y2, WHITE);             // V
}

/**************************************************************************************
��������: ����Ļ��ʾһ���µİ�ť��
��    ��: uint16_t x1,y1,x2,y2 ��ť�����ϽǺ����½�����
��    ��: ��
**************************************************************************************/
void DisplayButtonUp(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    Gui_DrawLine(x1, y1, x2, y1, WHITE); // H
    Gui_DrawLine(x1, y1, x1, y2, WHITE); // V

    Gui_DrawLine(x1 + 1, y2 - 1, x2, y2 - 1, GRAY1); // H
    Gui_DrawLine(x1, y2, x2, y2, GRAY2);             // H
    Gui_DrawLine(x2 - 1, y1 + 1, x2 - 1, y2, GRAY1); // V
    Gui_DrawLine(x2, y1, x2, y2, GRAY2);             // V
}

void Gui_DrawFont_ASC8(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *s)
{
    unsigned char i, j;
    unsigned short k, x0;
    x0 = x;

    while (*s)
    {
        if ((*s) < 128)
        { // Check the char is ASCII code
            k = *s;
            if (k == 13) // ASCII(13) -> Carriage Return
            {
                x = x0;
                y += 16;
            }
            else
            {
                if (k > 32) // ASCII(32) -> Space
                    k -= 32;
                else
                    k = 0;

                for (i = 0; i < 16; i++)
                    for (j = 0; j < 8; j++)
                    {
                        if (asc8x16[k * 16 + i] & (0x80 >> j))
                            Lcd_DrawPoint(x + j, y + i, fc);
                        else
                        {
                            if (fc != bc)
                                Lcd_DrawPoint(x + j, y + i, bc);
                        }
                    }
                x += 8;
            }
            s++;
        }

        else
        {

            for (k = 0; k < hz16_num; k++)
            {
                if ((hz16[k].Index[0] == *(s)) && (hz16[k].Index[1] == *(s + 1)))
                {
                    for (i = 0; i < 16; i++)
                    {
                        for (j = 0; j < 8; j++)
                        {
                            if (hz16[k].Msk[i * 2] & (0x80 >> j))
                                Lcd_DrawPoint(x + j, y + i, fc);
                            else
                            {
                                if (fc != bc)
                                    Lcd_DrawPoint(x + j, y + i, bc);
                            }
                        }
                        for (j = 0; j < 8; j++)
                        {
                            if (hz16[k].Msk[i * 2 + 1] & (0x80 >> j))
                                Lcd_DrawPoint(x + j + 8, y + i, fc);
                            else
                            {
                                if (fc != bc)
                                    Lcd_DrawPoint(x + j + 8, y + i, bc);
                            }
                        }
                    }
                }
            }
            s += 2;
            x += 16;
        }
    }
}

void Gui_DrawFont_GBK16(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *s)
{
    unsigned char i, j;
    unsigned short k, x0;
    x0 = x;

    while (*s)
    {
        if ((*s) < 128)
        {
            k = *s;
            if (k == 13) // ASCII(13) -> Carriage Return
            {
                x = x0;
                y += 16;
            }
            else
            {
                if (k > 32) // ASCII(32) -> Space
                    k -= 32;
                else
                    k = 0;

                for (i = 0; i < 16; i++)
                    for (j = 0; j < 8; j++)
                    {
                        if (asc16[k * 16 + i] & (0x80 >> j))
                            Lcd_DrawPoint(x + j, y + i, fc);
                        else
                        {
                            if (fc != bc)
                                Lcd_DrawPoint(x + j, y + i, bc);
                        }
                    }
                x += 8;
            }
            s++;
        }

        else
        {

            for (k = 0; k < hz16_num; k++)
            {
                if ((hz16[k].Index[0] == *(s)) && (hz16[k].Index[1] == *(s + 1)))
                {
                    for (i = 0; i < 16; i++)
                    {
                        for (j = 0; j < 8; j++)
                        {
                            if (hz16[k].Msk[i * 2] & (0x80 >> j))
                                Lcd_DrawPoint(x + j, y + i, fc);
                            else
                            {
                                if (fc != bc)
                                    Lcd_DrawPoint(x + j, y + i, bc);
                            }
                        }
                        for (j = 0; j < 8; j++)
                        {
                            if (hz16[k].Msk[i * 2 + 1] & (0x80 >> j))
                                Lcd_DrawPoint(x + j + 8, y + i, fc);
                            else
                            {
                                if (fc != bc)
                                    Lcd_DrawPoint(x + j + 8, y + i, bc);
                            }
                        }
                    }
                }
            }
            s += 2;
            x += 16;
        }
    }
}

void Gui_DrawFont_GBK24(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t *s)
{
    unsigned char i, j;
    unsigned short k;

    while (*s)
    {
        if (*s < 0x80)
        {
            k = *s;
            if (k > 32)
                k -= 32;
            else
                k = 0;

            for (i = 0; i < 16; i++)
                for (j = 0; j < 8; j++)
                {
                    if (asc16[k * 16 + i] & (0x80 >> j))
                        Lcd_DrawPoint(x + j, y + i, fc);
                    else
                    {
                        if (fc != bc)
                            Lcd_DrawPoint(x + j, y + i, bc);
                    }
                }
            s++;
            x += 8;
        }
        else
        {

            for (k = 0; k < hz24_num; k++)
            {
                if ((hz24[k].Index[0] == *(s)) && (hz24[k].Index[1] == *(s + 1)))
                {
                    for (i = 0; i < 24; i++)
                    {
                        for (j = 0; j < 8; j++)
                        {
                            if (hz24[k].Msk[i * 3] & (0x80 >> j))
                                Lcd_DrawPoint(x + j, y + i, fc);
                            else
                            {
                                if (fc != bc)
                                    Lcd_DrawPoint(x + j, y + i, bc);
                            }
                        }
                        for (j = 0; j < 8; j++)
                        {
                            if (hz24[k].Msk[i * 3 + 1] & (0x80 >> j))
                                Lcd_DrawPoint(x + j + 8, y + i, fc);
                            else
                            {
                                if (fc != bc)
                                    Lcd_DrawPoint(x + j + 8, y + i, bc);
                            }
                        }
                        for (j = 0; j < 8; j++)
                        {
                            if (hz24[k].Msk[i * 3 + 2] & (0x80 >> j))
                                Lcd_DrawPoint(x + j + 16, y + i, fc);
                            else
                            {
                                if (fc != bc)
                                    Lcd_DrawPoint(x + j + 16, y + i, bc);
                            }
                        }
                    }
                }
            }
            s += 2;
            x += 24;
        }
    }
}

void Gui_DrawFont_Num32(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint16_t num)
{
    unsigned char i, j, k, c;
    // lcd_text_any(x+94+i*42,y+34,32,32,0x7E8,0x0,sz32,knum[i]);
    //	w=w/8;

    for (i = 0; i < 32; i++)
    {
        for (j = 0; j < 4; j++)
        {
            c = *(sz32 + num * 32 * 4 + i * 4 + j);
            for (k = 0; k < 8; k++)
            {

                if (c & (0x80 >> k))
                    Lcd_DrawPoint(x + j * 8 + k, y + i, fc);
                else
                {
                    if (fc != bc)
                        Lcd_DrawPoint(x + j * 8 + k, y + i, bc);
                }
            }
        }
    }
}

int _Pow(int x, int y)
{
    unsigned char i;
    int result = 1;
    for (i = 0; i < y; i++)
    {
        result *= x;
    }
    return result;
}

void Gui_DrawNums_ASC8(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint16_t num, uint8_t length)
{

    uint8_t i;
    uint8_t flag = 0;
    uint8_t display = 0;
    unsigned char _num;

    for (i = length; i > 0; i--)
    {
        _num = '0' + num / _Pow(10, i - 1) % 10;

        // Check the number is 0
        if (_num == '0')
        {
            flag = 0;
        }
        else
        {
            flag = 1;
        }

        display += flag; // display flag

        if (display)
        {
            Gui_DrawFont_ASC8(x, y, fc, bc, &_num);
            x += 9;
        }
    }
}
