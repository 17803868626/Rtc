#include "Rtc.h"

#include "main.h"
#include "gpio.h"

//��ʼʱ�䶨��
uint8_t time_buf[8] = {0x20,0x08,0x10,0x04,0x23,0x59,0x50,0x02};//��ʼʱ��
uint8_t readtime[14];//��ǰʱ��
uint8_t sec_buf=0;  //�뻺��
uint8_t sec_flag=0; //���־λ


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
	RST_Pull(0);			//RST���õ�
	SCK_Pull(0);			//SCK���õ�
}

//��DS1302д��һ�ֽ�����
void ds1302_write_byte(uint8_t addr, uint8_t d) 
{
	uint8_t i;
	RST_Pull(1);					//����DS1302����	
	//д��Ŀ���ַ��addr
	addr = addr & 0xFE;   //���λ���㣬�Ĵ���0λΪ0ʱд��Ϊ1ʱ��
	for (i = 0; i < 8; i ++) {
		if (addr & 0x01) {
			IO_Pull(1);
			}
		else {
			IO_Pull(0);
			}
		SCK_Pull(1);      //����ʱ��
		SCK_Pull(0); 
		addr = addr >> 1;
		}	
	//д�����ݣ�d
	for (i = 0; i < 8; i ++) {
		if (d & 0x01) {
			IO_Pull(1);
			}
		else {
			IO_Pull(0);
			}
		SCK_Pull(1);     //����ʱ��
		SCK_Pull(0); 
		d = d >> 1;
		}
	RST_Pull(0);		//ֹͣDS1302����
}

//��DS1302����һ�ֽ�����
uint8_t ds1302_read_byte(uint8_t addr) {

	uint8_t i,temp;	
	RST_Pull(1);					//����DS1302����
	//д��Ŀ���ַ��addr
	addr = addr | 0x01;    //���λ�øߣ��Ĵ���0λΪ0ʱд��Ϊ1ʱ��
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
	//������ݣ�temp
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
	RST_Pull(0);					//ֹͣDS1302����
	return temp;
}

//��DS302д��ʱ������
void ds1302_write_time(void) 
{
	ds1302_write_byte(ds1302_control_add,0x00);			//�ر�д���� 
	ds1302_write_byte(ds1302_sec_add,0x80);				//��ͣʱ�� 
	//ds1302_write_byte(ds1302_charger_add,0xa9);	    //������ 
	ds1302_write_byte(ds1302_year_add,time_buf[1]);		//�� 
	ds1302_write_byte(ds1302_month_add,time_buf[2]);	//�� 
	ds1302_write_byte(ds1302_date_add,time_buf[3]);		//�� 
	ds1302_write_byte(ds1302_hr_add,time_buf[4]);		//ʱ 
	ds1302_write_byte(ds1302_min_add,time_buf[5]);		//��
	ds1302_write_byte(ds1302_sec_add,time_buf[6]);		//��
	ds1302_write_byte(ds1302_day_add,time_buf[7]);		//�� 
	ds1302_write_byte(ds1302_control_add,0x80);			//��д����     
}

//��DS302����ʱ������
void ds1302_read_time(void)  
{
	time_buf[1]=ds1302_read_byte(ds1302_year_add);		//�� 
	time_buf[2]=ds1302_read_byte(ds1302_month_add);		//�� 
	time_buf[3]=ds1302_read_byte(ds1302_date_add);		//�� 
	time_buf[4]=ds1302_read_byte(ds1302_hr_add);		//ʱ 
	time_buf[5]=ds1302_read_byte(ds1302_min_add);		//�� 
	time_buf[6]=(ds1302_read_byte(ds1302_sec_add))&0x7f;//�룬������ĵ�7λ�����ⳬ��59
	time_buf[7]=ds1302_read_byte(ds1302_day_add);		//�� 	
}















