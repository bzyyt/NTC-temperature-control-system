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
    float deviation;       // ��ǰ���
    float deviation_Last;  // �ϴε����
    float deviations[200]; // ��������
    uint8_t pla;
    float integeral;    // ������
    float differential; // ���΢��
    float kp, ki, kd;   // ���ߵı�������
} parameter;

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
    float B   = 3435.0;
    float T1  = 1 / (log(Rt / R1) / B + 1 / T2);
    float T   = T1 - 273.15 + 0.5;
    return T;
}

// ��ʱ10us
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

// ����Ļ��ָ��λ��д����
void Write_Data(uint8_t place, uint8_t shuju)
{
    Write_1602_Com(place);
    Write_1602_Data(shuju);
}

// ����Ļ��ʼ����ʾ����
void Write_Init()
{
    uint8_t i      = 0;
    uint8_t Temp[] = {
        0x54, // T
        0x65, // e
        0x6d, // m
        0x70, // p
        0x3d, // =
        0x20, //
        0x20,
        0x20,
        0x2e,
        0x20,
        0x20,
        0xdf, // ��
        0x43  // C
    };
    uint8_t V[] = {
        0x56, // V
        0x3d, //=
        0x20, //
        0x20,
        0x20,
        0x2e,
        0x20,
        0x20,
        0x76 // v
    };
    for (i = 0; i < 13; i++) Write_Data(0x80 + i, Temp[i]);
    for (i = 0; i < 9; i++) Write_Data(0xc0 + i, V[i]);
}

// ����Ļ�����С��
void Write_float(uint8_t place, float num)
{
    int integer = (int)num;                     // ��������
    int decimal = (int)((num - integer) * 100); // С������
    Write_Num(integer, place);
    Write_Num(decimal, place + 3);
}

// PWM����ϵͳ ��λ��1ms
void PWM_Hot(int percent)
{
    uint8_t i;
    LED = 0;
    for (i = 1; i <= 100; i++) {
        if (i > percent) {
            LED = 1; // ������
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
    uint16_t s;
    while (1) {
        s   = (int)(500 * sin(t++ / 100.0));
        LED = 1;
        for (i = 1; i < s; i++) _nop_();
        LED = 0;
        for (i = s; i <= 500; i++) _nop_();
        t %= 314;
    }
}

// PID��ʼ��
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

    parameter.kd = 20;  // ΢�ֲ���
    parameter.ki = 0.8; // ���ֲ���
    parameter.kp = 30;  // ��������
}

// PIDϵͳ
int PID(float now_temp)
{
    double result;
    parameter.deviation = Target_temp - now_temp;                            // �������ֵ
    parameter.integeral += parameter.deviation;                              // �������
    parameter.differential = parameter.deviation - parameter.deviation_Last; // ΢�����

    result = parameter.ki * parameter.integeral + parameter.kp * parameter.deviation + parameter.kd * parameter.differential; // ����PID

    parameter.integeral -= parameter.deviations[parameter.pla];
    parameter.deviations[parameter.pla] = parameter.deviation;
    parameter.pla++;
    parameter.pla %= 200;

    parameter.deviation_Last = parameter.deviation;

    if (result > 100.0) result = 100;
    if (result < 0.0) result = 0;
    return (int)result;
}

void main()
{
    float adcx        = 0.0;
    float temperature = 0.0;
    uint8_t i, j, k;
    int percent = 100;

    // ��ʼ����
    UART_init();         // ��ʼ������
    LCD_1602_Init();     // ��ʼ��LCD
    ADC_Init(ADC_PORT1); // ��ʼ��AD ��ӦP1��
    Write_Init();        // ��ָ��λ��дtemp
    PID_Init();          // PID��ʼ��

    // LCD��ʾ�Ĵ���
    // Write_1602_String("Hello World!", 0X80); // ��һ�е� 1 ��
    // Write_Num(0xffff, 0XC0);                 // �ڶ��е� 1 ��

    // ����������

    // light2();
    // light();
    Delay2000ms();
    Delay2000ms();

    while (1) {
        // ���ص�Դ�㷨
        // for (i = 1; i <= 10; i++) {
        //     for (j = 1; j <= 10; j++) {
        //         Delay10ms();
        //         adcx        = GetADCResult(ADC_CH1); // ��ȡ��ѹ
        //         temperature = Temp_trans(adcx);      // ת��Ϊ�¶�
        //         if (temperature > Target_temp) {
        //             // �¶�>Ŀ�� Ϊ1 ֹͣ
        //             LED = 1;
        //         } else {
        //             LED = 0;
        //         }
        //     }
        //     printf("%f\r\n", temperature);
        // }
        // // LED = LED ^ 1;
        // // 0���ȣ�1������
        // Write_float(0X86, temperature, 6);

        // PID
        for (i = 1; i <= 4; i++) {
            for (j = 1; j <= 100; j++) {
                adcx = 0;
                for (k = 1; k <= 2; k++) {
                    adcx += GetADCResult(ADC_CH1); // ��ȡ��ѹ
                }
                // adcx        = GetADCResult(ADC_CH1); // ��ȡ��ѹ
                temperature = Temp_trans(adcx / 2); // ת��Ϊ�¶�
                percent     = PID(temperature);     // ����PID����
                PWM_Hot(percent);                   // ���м���
                // printf("percent = %d\n*\n", percent);
                // 1ms����һ��PID����
            }
            // 100ms����һ�δ���ͨ��
            // printf("percent = %d\n", percent);
            printf("%f\n\n", temperature);
        }
        // 1�����һ���¶�
        Write_float(0X86, temperature);
        Write_float(0xc3, adcx);
    }
}