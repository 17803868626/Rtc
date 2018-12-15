#include "Rtc.h"

#include "main.h"
#include "gpio.h"

//初始时间定义
uint8_t time_buf[8] = {0x20,0x08,0x10,0x04,0x23,0x59,0x50,0x02};//初始时间
uint8_t readtime[14];//当前时间
uint8_t sec_buf=0;  //秒缓存
uint8_t sec_flag=0; //秒标志位


void RST_Pull(uint8_t state)
{
	if(state)HAL_GPIO_WritePin(RST_GPIO_Port,RST_Pin,GPIO_PIN_SET);
	if(!state)HAL_GPIO_WritePin(RST_GPIO_Port,RST_Pin,GPIO_PIN_RESET);
}

void IO_Pull(uint8_t state)
{
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = IO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(IO_GPIO_Port, &GPIO_InitStruct);

	if(state==1)HAL_GPIO_WritePin(IO_GPIO_Port,IO_Pin,GPIO_PIN_SET);
	if(state==0)HAL_GPIO_WritePin(IO_GPIO_Port,IO_Pin,GPIO_PIN_RESET);
}

uint8_t IO_Read()
{
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = IO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IO_GPIO_Port, &GPIO_InitStruct);
	
	return (uint8_t)HAL_GPIO_ReadPin(IO_GPIO_Port,IO_Pin);
}

void SCK_Pull(uint8_t state)
{
	if(state==1)HAL_GPIO_WritePin(SCK_GPIO_Port,SCK_Pin,GPIO_PIN_SET);
	if(state==0)HAL_GPIO_WritePin(SCK_GPIO_Port,SCK_Pin,GPIO_PIN_RESET);
}

void ds1302_init(void) 
{
	RST_Pull(0);			//RST脚置低
	SCK_Pull(0);			//SCK脚置低
}

//向DS1302写入一字节数据
void ds1302_write_byte(uint8_t addr, uint8_t d) 
{
	uint8_t i;
	RST_Pull(1);					//启动DS1302总线	
	//写入目标地址：addr
	addr = addr & 0xFE;   //最低位置零，寄存器0位为0时写，为1时读
	for (i = 0; i < 8; i ++) {
		if (addr & 0x01) {
			IO_Pull(1);
			}
		else {
			IO_Pull(0);
			}
		SCK_Pull(1);      //产生时钟
		SCK_Pull(0); 
		addr = addr >> 1;
		}	
	//写入数据：d
	for (i = 0; i < 8; i ++) {
		if (d & 0x01) {
			IO_Pull(1);
			}
		else {
			IO_Pull(0);
			}
		SCK_Pull(1);     //产生时钟
		SCK_Pull(0); 
		d = d >> 1;
		}
	RST_Pull(0);		//停止DS1302总线
}

//从DS1302读出一字节数据
uint8_t ds1302_read_byte(uint8_t addr) {

	uint8_t i,temp;	
	RST_Pull(1);					//启动DS1302总线
	//写入目标地址：addr
	addr = addr | 0x01;    //最低位置高，寄存器0位为0时写，为1时读
	for (i = 0; i < 8; i ++) {
		if (addr & 0x01) {
			IO_Pull(1);
			}
		else {
			IO_Pull(0);
			}
		SCK_Pull(1);
		SCK_Pull(0);
		addr = addr >> 1;
		}	
	//输出数据：temp
	for (i = 0; i < 8; i ++) {
		temp = temp >> 1;
		if (IO_Read()) {
			temp |= 0x80;
			}
		else {
			temp &= 0x7F;
			}
		SCK_Pull(1);
		SCK_Pull(0);
		}	
	RST_Pull(0);					//停止DS1302总线
	return temp;
}

//向DS302写入时钟数据
void ds1302_write_time(void) 
{
	ds1302_write_byte(ds1302_control_add,0x00);			//关闭写保护 
	ds1302_write_byte(ds1302_sec_add,0x80);				//暂停时钟 
	//ds1302_write_byte(ds1302_charger_add,0xa9);	    //涓流充电 
	ds1302_write_byte(ds1302_year_add,time_buf[1]);		//年 
	ds1302_write_byte(ds1302_month_add,time_buf[2]);	//月 
	ds1302_write_byte(ds1302_date_add,time_buf[3]);		//日 
	ds1302_write_byte(ds1302_hr_add,time_buf[4]);		//时 
	ds1302_write_byte(ds1302_min_add,time_buf[5]);		//分
	ds1302_write_byte(ds1302_sec_add,time_buf[6]);		//秒
	ds1302_write_byte(ds1302_day_add,time_buf[7]);		//周 
	ds1302_write_byte(ds1302_control_add,0x80);			//打开写保护     
}

//从DS302读出时钟数据
void ds1302_read_time(void)  
{
	time_buf[1]=ds1302_read_byte(ds1302_year_add);		//年 
	time_buf[2]=ds1302_read_byte(ds1302_month_add);		//月 
	time_buf[3]=ds1302_read_byte(ds1302_date_add);		//日 
	time_buf[4]=ds1302_read_byte(ds1302_hr_add);		//时 
	time_buf[5]=ds1302_read_byte(ds1302_min_add);		//分 
	time_buf[6]=(ds1302_read_byte(ds1302_sec_add))&0x7f;//秒，屏蔽秒的第7位，避免超出59
	time_buf[7]=ds1302_read_byte(ds1302_day_add);		//周 	
}















