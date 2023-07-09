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

// ����Ļָ��λ��д��Temp= ��
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
        str[add] = 0x2e; // С����
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

    // ��ʼ����
    UART_init();         // ��ʼ������
    LCD_1602_Init();     // ��ʼ��LCD
    ADC_Init(ADC_PORT1); // ��ʼ��AD ��ӦP1��
    Write_Temp(0x80);    // ��ָ��λ��дtemp
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
        Write_float(0X86, temperature, 6);
    }
}