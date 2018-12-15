#ifndef __RTC_H
#define __RTC_H

#include "main.h"


//DS1302地址定义
#define ds1302_sec_add			  0x80		//秒数据地址
#define ds1302_min_add			  0x82		//分数据地址
#define ds1302_hr_add			    0x84		//时数据地址
#define ds1302_date_add			  0x86		//日数据地址
#define ds1302_month_add		  0x88		//月数据地址
#define ds1302_day_add			  0x8a		//星期数据地址
#define ds1302_year_add			  0x8c		//年数据地址
#define ds1302_control_add		0x8e		//控制数据地址
#define ds1302_charger_add		0x90 					 
#define ds1302_clkburst_add		0xbe

//初始时间定义
extern uint8_t time_buf[8];//初始时间
extern uint8_t readtime[14];//当前时间
extern uint8_t sec_buf;  //秒缓存
extern uint8_t sec_flag; //秒标志位

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

