#ifndef __RTC_H
#define __RTC_H

#include "main.h"


//DS1302��ַ����
#define ds1302_sec_add			  0x80		//�����ݵ�ַ
#define ds1302_min_add			  0x82		//�����ݵ�ַ
#define ds1302_hr_add			    0x84		//ʱ���ݵ�ַ
#define ds1302_date_add			  0x86		//�����ݵ�ַ
#define ds1302_month_add		  0x88		//�����ݵ�ַ
#define ds1302_day_add			  0x8a		//�������ݵ�ַ
#define ds1302_year_add			  0x8c		//�����ݵ�ַ
#define ds1302_control_add		0x8e		//�������ݵ�ַ
#define ds1302_charger_add		0x90 					 
#define ds1302_clkburst_add		0xbe

//��ʼʱ�䶨��
extern uint8_t time_buf[8];//��ʼʱ��
extern uint8_t readtime[14];//��ǰʱ��
extern uint8_t sec_buf;  //�뻺��
extern uint8_t sec_flag; //���־λ

void RST_Pull(uint8_t state);
void IO_Pull(uint8_t state);
uint8_t IO_Read(void);
void SCK_Pull(uint8_t state);

void ds1302_init(void);
void ds1302_write_byte(uint8_t addr, uint8_t d);
uint8_t ds1302_read_byte(uint8_t addr);
void ds1302_write_time(void);
void ds1302_read_time(void);











#endif

