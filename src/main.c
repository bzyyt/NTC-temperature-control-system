/**
 * *********************************************
 *
 *  NTC温度控制系统
 *
 *  版本 1.0
 *
 *  20230706
 *
 * *********************************************
 */

#include "STC12C5A60S2.H"
#include "LCD1602.H"
#include "ADC.h"
#include <STDIO.H>
#include <MATH.H>

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long uint32_t;

typedef signed char int8_t;
typedef signed int int16_t;
typedef signed long int32_t;

#define FSCLK 11059200
// LED pin define
sbit LED = P1 ^ 0;

// 串口初始化
void UART_init(void)
{
    SCON = 0x50; // 设置为方式 1,8 位数据,可变波特率,接收允许
    TMOD = 0x20; // 定时器 1:模式 2,8 位自动重装模式,用于产生波特率
    TL1  = 0xFD; // 定时初值
    TH1  = 0xFD; // 定时器重装值
    TR1  = 1;
    ES   = 1;
    TI   = 1;
    EA   = 1;
}

// 电压转温度
float Temp_trans(float adx)
{
    float V0  = adx;
    float Vcc = 5.0;
    float R1  = 10000.0;
    float Rt  = V0 / (Vcc - V0) * R1;
    float T2  = 273.15 + 25;
    float Rp  = 10000.0;
    float B   = 3435.0;
    float T1  = 1 / (log(Rt / Rp) / B + 1 / T2);
    float T   = T1 - 273.15 + 0.5;
    return T;
}

// 延时1ms
void Delay1ms() //@11.0592MHz
{
    unsigned char data i, j;

    _nop_();
    _nop_();
    _nop_();
    i = 11;
    j = 190;
    do {
        while (--j)
            ;
    } while (--i);
}

// 毫秒延时
void delayms(int times)
{
    int i, j;
    for (i = 0; i < 1000; i++)
        for (j = 0; j < times; j++)
            ;
}

// 在屏幕上指定位置写数据
void Write_Data(uint8_t place, uint8_t shuju)
{
    Write_1602_Com(place);
    Write_1602_Data(shuju);
}

// 在屏幕上输出小数
void Write_float(uint8_t place, float num, uint8_t maxnum)
{
    /*
    place 是起始位置
    num 是具体的数值
    maxnum 是显示的最大位数
    */
    int integer   = (int)num;      // 整数部分
    float decimal = num - integer; // 小数部分
    unsigned str[16];              // 字符串
    uint8_t longi = 0;             // 整数部分的长度
    uint8_t add   = 0;             // 当前字符的位置
    int i;
    // 初始化str
    for (i = 0; i < 16; i++) {
        str[i] = 0x20;
    }
    if (maxnum > 15)
        maxnum = 15;
    maxnum--;
    // 负数
    if (num < 0) {
        str[add] = 0x2d; // 负号
        add++;
        num = -num;
    }
    // 整数部分
    if (log10(num) - (int)log10(num) > 0.0000001)
        longi = (int)log10(num) + 1;
    else
        longi = (int)log10(num);
    for (i = add + longi - 1; i >= add; i--) {
        str[i] = integer % 10 + 0x30;
        integer /= 10;
    }
    add += longi;
    // 小数部分
    if (decimal > 0) {
        str[add] = 0x2e;
        add++;
        for (i = add; i < maxnum; i++) {
            decimal *= 10;
            str[i] = (int)decimal % 10 + 0x30;
        }
    }
    str[maxnum] = 0x04;
    for (i = 0; i <= maxnum; i++) {
        Write_Data(place + i, str[i]);
    }
}

void main()
{

    int sum           = 0;
    float adcx        = 0.0;
    float temperature = 0.0;

    // 初始化区
    UART_init();         // 初始化串口
    LCD_1602_Init();     // 初始化LCD
    ADC_Init(ADC_PORT1); // 初始化AD 对应P1口
    LED = 1;             // 初始化LED

    // LCD显示的代码
    // Write_1602_String("Hello World!", 0X80); // 第一行第 1 列
    // Write_Num(0xffff, 0XC0);                 // 第二行第 1 列

    // 主函数代码
    while (1) {

        delayms(500); // 主函数每一步的延时

        LED = LED ^ 1;

        adcx        = GetADCResult(ADC_CH1); // 读取电压
        temperature = Temp_trans(adcx);      // 转换为温度
        Write_float(0X80, temperature, 6);
        printf("%f\r\n", temperature);
    }
}
