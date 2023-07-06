/**
 * *********************************************
 *
 *  NTC�¶ȿ���ϵͳ
 *
 *  �汾 1.0
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

// ���ڳ�ʼ��
void UART_init(void)
{
    SCON = 0x50; // ����Ϊ��ʽ 1,8 λ����,�ɱ䲨����,��������
    TMOD = 0x20; // ��ʱ�� 1:ģʽ 2,8 λ�Զ���װģʽ,���ڲ���������
    TL1  = 0xFD; // ��ʱ��ֵ
    TH1  = 0xFD; // ��ʱ����װֵ
    TR1  = 1;
    ES   = 1;
    TI   = 1;
    EA   = 1;
}

// ��ѹת�¶�
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

// ��ʱ1ms
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

// ������ʱ
void delayms(int times)
{
    int i, j;
    for (i = 0; i < 1000; i++)
        for (j = 0; j < times; j++)
            ;
}

// ����Ļ��ָ��λ��д����
void Write_Data(uint8_t place, uint8_t shuju)
{
    Write_1602_Com(place);
    Write_1602_Data(shuju);
}

// ����Ļ�����С��
void Write_float(uint8_t place, float num, uint8_t maxnum)
{
    /*
    place ����ʼλ��
    num �Ǿ������ֵ
    maxnum ����ʾ�����λ��
    */
    int integer   = (int)num;      // ��������
    float decimal = num - integer; // С������
    unsigned str[16];              // �ַ���
    uint8_t longi = 0;             // �������ֵĳ���
    uint8_t add   = 0;             // ��ǰ�ַ���λ��
    int i;
    // ��ʼ��str
    for (i = 0; i < 16; i++) {
        str[i] = 0x20;
    }
    if (maxnum > 15)
        maxnum = 15;
    maxnum--;
    // ����
    if (num < 0) {
        str[add] = 0x2d; // ����
        add++;
        num = -num;
    }
    // ��������
    if (log10(num) - (int)log10(num) > 0.0000001)
        longi = (int)log10(num) + 1;
    else
        longi = (int)log10(num);
    for (i = add + longi - 1; i >= add; i--) {
        str[i] = integer % 10 + 0x30;
        integer /= 10;
    }
    add += longi;
    // С������
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

    // ��ʼ����
    UART_init();         // ��ʼ������
    LCD_1602_Init();     // ��ʼ��LCD
    ADC_Init(ADC_PORT1); // ��ʼ��AD ��ӦP1��
    LED = 1;             // ��ʼ��LED

    // LCD��ʾ�Ĵ���
    // Write_1602_String("Hello World!", 0X80); // ��һ�е� 1 ��
    // Write_Num(0xffff, 0XC0);                 // �ڶ��е� 1 ��

    // ����������
    while (1) {

        delayms(500); // ������ÿһ������ʱ

        LED = LED ^ 1;

        adcx        = GetADCResult(ADC_CH1); // ��ȡ��ѹ
        temperature = Temp_trans(adcx);      // ת��Ϊ�¶�
        Write_float(0X80, temperature, 6);
        printf("%f\r\n", temperature);
    }
}
