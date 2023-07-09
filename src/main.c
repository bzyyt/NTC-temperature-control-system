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
sbit LED          = P1 ^ 0;
float Target_temp = 40.5;
struct PID_Parameter {
    float deviation;       // 当前误差
    float deviation_Last;  // 上次的误差
    float deviations[200]; // 误差的数组
    uint8_t pla;
    float integeral;    // 误差积分
    float differential; // 误差微分
    float kp, ki, kd;   // 三者的比例函数
} parameter;

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
    float B   = 3435.0;
    float T1  = 1 / (log(Rt / R1) / B + 1 / T2);
    float T   = T1 - 273.15 + 0.5;
    return T;
}

// 延时10us
void Delay10us() //@11.0592MHz
{
    unsigned char data i;

    _nop_();
    _nop_();
    _nop_();
    i = 24;
    while (--i)
        ;
}

// 10ms
void Delay10ms() //@11.0592MHz
{
    unsigned char data i, j;

    _nop_();
    _nop_();
    i = 108;
    j = 144;
    do {
        while (--j)
            ;
    } while (--i);
}

// 2000ms
void Delay2000ms() //@11.0592MHz
{
    unsigned char data i, j, k;

    i = 85;
    j = 12;
    k = 155;
    do {
        do {
            while (--k)
                ;
        } while (--j);
    } while (--i);
}

// 在屏幕上指定位置写数据
void Write_Data(uint8_t place, uint8_t shuju)
{
    Write_1602_Com(place);
    Write_1602_Data(shuju);
}

// 在屏幕指定位置写“Temp= ”
void Write_Temp(uint8_t place)
{
    uint8_t i      = 0;
    uint8_t Temp[] = {
        0x54,
        0x65,
        0x6d,
        0x70,
        0x3d,
        0x20};
    for (i = 0; i <= 5; i++)
        Write_Data(place + i, Temp[i]);
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
        str[add] = 0x2e; // 小数点
        add++;
        for (i = add; i < maxnum; i++) {
            decimal *= 10;
            str[i] = (int)decimal % 10 + 0x30;
        }
    }
    str[maxnum++] = 0xDF;
    str[maxnum++] = 0x43;
    for (i = 0; i < maxnum; i++) {
        Write_Data(place + i, str[i]);
    }
}

// PWM加热系统 单位是1ms
void PWM_Hot(int percent)
{
    uint8_t i;
    LED = 0;
    for (i = 1; i <= 100; i++) {
        if (i > percent) {
            LED = 1; // 不加热
        }
        Delay10us();
    }
}

void light2()
{
    while (1) {
        LED = 1;
        Delay10ms();
        Delay10ms();
        Delay10ms();
        LED = 0;
        Delay10ms();
        Delay10ms();
        Delay10ms();
    }
}

void light()
{
    uint16_t t = 1;
    uint16_t i;
    while (1) {
        while (t <= 500) {
            LED = 1;
            _nop_();
            for (i = 1; i <= 500; i++) {
                if (i > t) {
                    LED = 0;
                }
                _nop_();
            }
            t++;
        }
        while (t > 0) {
            LED = 1;
            _nop_();
            for (i = 1; i <= 500; i++) {
                if (i > t) {
                    LED = 0;
                }
                _nop_();
            }
            t--;
        }
    }
}

// PID初始化
void PID_Init()
{
    uint8_t i = 0;
    for (i = 0; i < 200; i++) {
        parameter.deviations[i] = 0;
    }
    parameter.deviation      = 0;
    parameter.deviation_Last = 0;
    parameter.differential   = 0;
    parameter.integeral      = 0;

    parameter.kd = 20;  // 微分参数
    parameter.ki = 0.8; // 积分参数
    parameter.kp = 30;  // 比例参数
}

// PID系统
int PID(float now_temp)
{
    double result;
    parameter.deviation = Target_temp - now_temp;                            // 计算误差值
    parameter.integeral += parameter.deviation;                              // 积分误差
    parameter.differential = parameter.deviation - parameter.deviation_Last; // 微分误差

    result = parameter.ki * parameter.integeral + parameter.kp * parameter.deviation + parameter.kd * parameter.differential; // 计算PID

    parameter.integeral -= parameter.deviations[parameter.pla];
    parameter.deviations[parameter.pla] = parameter.deviation;
    parameter.pla++;
    parameter.pla %= 200;

    // printf("int = %f\n", parameter.integeral);
    // printf("dev = %f\n", parameter.deviation);
    // printf("dev_l = %f\n", parameter.deviation_Last);
    // printf("diff = %f\n", parameter.differential);

    parameter.deviation_Last = parameter.deviation;

    if (result > 100.0) result = 100;
    if (result < 0.0) result = 0;

    // result += 10;
    return (int)result;
}

void main()
{
    float adcx        = 0.0;
    float temperature = 0.0;
    uint8_t i, j, k;
    int percent = 100;

    // 初始化区
    UART_init();         // 初始化串口
    LCD_1602_Init();     // 初始化LCD
    ADC_Init(ADC_PORT1); // 初始化AD 对应P1口
    Write_Temp(0x80);    // 在指定位置写temp
    PID_Init();          // PID初始化

    // LCD显示的代码
    // Write_1602_String("Hello World!", 0X80); // 第一行第 1 列
    // Write_Num(0xffff, 0XC0);                 // 第二行第 1 列

    // 主函数代码

    // light2();
    // light();
    Delay2000ms();
    Delay2000ms();

    while (1) {
        // 开关电源算法
        // for (i = 1; i <= 10; i++) {
        //     for (j = 1; j <= 10; j++) {
        //         Delay10ms();
        //         adcx        = GetADCResult(ADC_CH1); // 读取电压
        //         temperature = Temp_trans(adcx);      // 转换为温度
        //         if (temperature > Target_temp) {
        //             // 温度>目标 为1 停止
        //             LED = 1;
        //         } else {
        //             LED = 0;
        //         }
        //     }
        //     printf("%f\r\n", temperature);
        // }
        // // LED = LED ^ 1;
        // // 0加热，1不加热
        // Write_float(0X86, temperature, 6);

        // PID
        for (i = 1; i <= 4; i++) {
            for (j = 1; j <= 100; j++) {
                adcx = 0;
                for (k = 1; k <= 2; k++) {
                    adcx += GetADCResult(ADC_CH1); // 读取电压
                }
                // adcx        = GetADCResult(ADC_CH1); // 读取电压
                temperature = Temp_trans(adcx / 2); // 转换为温度
                percent     = PID(temperature);     // 进行PID运算
                PWM_Hot(percent);                   // 进行加热
                // printf("percent = %d\n*\n", percent);
                // 1ms进行一次PID运算
            }
            // 100ms进行一次串口通信
            // printf("percent = %d\n", percent);
            printf("%f\n\n", temperature);
        }
        // 1秒输出一次温度
        Write_float(0X86, temperature, 6);
    }
}