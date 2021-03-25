#include "includes.h"
#define F_SCK 40000UL  // SCK Ŭ�� �� = 40 Khz

#define W 0x6A
#define A 0x77
#define R 0x50
#define M 0x55
#define C 0x58
#define O 0x5c
#define L 0x38
#define D 0x5e

/* ������ ���� TCNT�� ���� */
#define DO 17
#define RE 43
#define MI 66
#define FA 77
#define FA_SHOP 86
#define SOL 97
#define LA 114
#define LA_SHOP 122
#define TI 129
#define UDO 137
#define UDO_SHOP 143
#define URE 149
#define UMI 161
#define UFA 166
#define USOL 176
#define URA 184
#define UTI 192
#define REST -2   // ��ǥ
#define EOS -1    // End Of Song
#define SONG_COUNT 3		// �뷡�� ����

#define ON 0
#define OFF 1

#define TASK_STK_SIZE  OS_TASK_DEF_STK_SIZE
#define N_TASKS 8
#define ATS75_ADDR 0x98 // 0b10011000, 7��Ʈ�� 1��Ʈ left shift
#define ATS75_CONFIG_REG 1
#define ATS75_TEMP_REG 0
#define UCHAR unsigned char // UCHAR ����

void InitI2C();										  // I2C �ʱ�ȭ �Լ�
void write_twi_1byte_nopreset(UCHAR reg, UCHAR data); // �ػ󵵸� ������ �� ���
void write_twi_0byte_nopreset(UCHAR reg);			  // Temparture Register�� �ִ� ����� ������ �� ���
int ReadTemperature(void);							  // �µ��� �о�鿩 ������.
void init_adc();									  // ADC�� �ʱ� ���� ����.
unsigned short read_adc();							  // ADC���� �о��.

OS_STK       TaskStk[N_TASKS][TASK_STK_SIZE];		  // Task ���� ����
OS_FLAG_GRP*	enable_temp;						  
OS_FLAG_GRP*	choice_temp_grp;
OS_FLAG_GRP*	choice_speed_grp;
OS_FLAG_GRP*	choice_task;
OS_EVENT*		mbox_tone;

volatile int state,tone;	// ���� ��� ������ ���� ���
volatile int pointer;		// �뷡�� ���� ������ �� ���
volatile int light;			// �о���� ADC���� ����.
volatile int temp;			// �о���� �µ��� ����.
volatile int choice_temp;   // showWarmOrColdTask�� showTempTask�� ������ �� ���.
volatile int song;			// �뷡�� ������ �� ���
volatile int task;			// �뷡 ���� task�� �µ� ���� task �� � ���� ������ �� ������ �� ���.
INT8U err;

/* Segment */
unsigned char digit[10] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7c,0x07,0x7f,0x67};
unsigned char fnd_sel[4] = {0x01,0x02,0x04,0x08};
	
/* Song and Tempo */
const int jinglebel[] = {DO, LA, SOL, FA, DO, REST, DO,LA,SOL, FA,RE, REST,
	RE,LA_SHOP,LA,SOL,MI,REST, UDO,UDO,LA_SHOP,SOL,LA,FA,REST,
	DO,LA,SOL,FA,DO,REST,DO,LA,SOL,FA,RE,REST,
	RE,LA_SHOP,LA,SOL,UDO,UDO,UDO,UDO,URE,UDO,LA_SHOP,SOL,FA,REST,
	LA,LA,LA,LA,LA,LA,LA,UDO,FA,SOL,LA,REST,
	LA_SHOP,LA_SHOP,LA_SHOP,LA_SHOP,LA_SHOP,LA,LA,LA,LA,SOL,SOL,FA,SOL,UDO,REST,
	LA,LA,LA,LA,LA,LA,LA,UDO,FA,SOL,LA,REST,
	LA_SHOP,LA_SHOP,LA_SHOP,LA_SHOP,LA_SHOP,LA,LA,LA,DO,DO,LA_SHOP,SOL,FA,REST,EOS};
const double tempo_jinglebel[] = {250,250,250,250,750,250,250,250,250,250,750,250,
	250,250,250,250,750,250,250,250,250,250,500,250,250,
	250,250,250,250,750,250,250,250,250,250,750,250,
	250,250,250,250,375,125,250,250,250,250,250,250,750,250,
	250,250,500,250,250,500,250,250,250,125,750,250,
	250,250,375,125,250,250,250,250,250,250,250,250,500,250,250,
	250,250,500,250,250,500,250,250,375,125,750,250,
250,250,375,125,250,250,250,250,250,250,250,250,750,250,0};
const int silent_night[] = {SOL,LA,SOL,MI,SOL,LA,SOL,MI,
							URE,URE,TI,UDO,UDO,SOL,
							LA,LA,UDO,TI,LA,SOL,LA,SOL,MI,
							LA,LA,UDO,TI,LA,SOL,LA,SOL,MI,
							URE,URE,UFA,URE,TI,UDO,UMI,
							UDO,SOL,MI,SOL,FA,RE,DO,EOS};				
const double tempo_silent_night[] ={750,250,500,1500,750,250,500,1500,
									1000,500,1500,1000,500,1500,
									1000,500,750,250,500,750,250,500,1500,
									1000,500,750,250,500,750,250,500,1500,
									1000,500,750,250,500,1500,1500,
									500,500,500,750,250,500,1500,0};  								
const int look_out_the_window[] = { LA,LA,LA,LA,FA_SHOP,LA,LA,LA,LA,FA_SHOP,LA,LA,FA_SHOP,LA,URE,UDO_SHOP,REST,
									UMI,UMI,UMI,UMI,URE,UDO_SHOP,UDO_SHOP,UDO_SHOP,UDO_SHOP,TI,LA,LA,LA,SOL,SOL,FA_SHOP,REST,
									LA,LA,LA,LA,FA_SHOP,LA,LA,LA,LA,FA_SHOP,LA,LA,FA_SHOP,LA,LA,URE,URE,UDO_SHOP,REST,
									UMI,UMI,UMI,UMI,URE,UDO_SHOP,UDO_SHOP,UDO_SHOP,UDO_SHOP,TI,LA,LA,LA,TI,UDO_SHOP,URE,REST,
									SOL,RE,SOL,TI,URE,URE,URE,URE,LA,FA_SHOP,RE,FA_SHOP,LA,REST,
									SOL,RE,SOL,TI,URE,URE,URE,URE,URE,UMI,URE,UDO_SHOP,TI,LA,TI,UDO_SHOP,EOS};
const double tempo_look_out[] = {250,500,250,500,500,250,500,250,500,500,250,500,250,500,500,1500,250,
								250,500,250,500,500,250,500,250,500,500,250,500,250,500,500,1500,250,
								250,500,250,500,500,250,500,250,500,500,250,500,250,250,250,250,250,1500,250,
								250,500,250,500,500,250,500,250,500,500,250,500,250,500,500,1500,250,
								500,500,500,500,500,250,250,1000,500,500,500,500,1500,250,
								500,500,500,500,500,250,250,750,250,500,500,500,500,500,500,1000,0};

const int *song_list[3] = {jinglebel,			// ¡�ۺ�
						   silent_night,		// ����� ��
						   look_out_the_window  // â���� ����
						   };
const double *tempo_list[3] = {tempo_jinglebel,tempo_silent_night,tempo_look_out}; // �� �뷡�� Tempo�� ����

/* Warm, Cold�� �ش�Ǵ� ���׸�Ʈ 16���� �� */
const unsigned char warm[] = {W,A,R,M};
const unsigned char cold[] = {C,O,L,D};

/* Task Prototype */
void TemperatureTask(void *data);
void showTempTask(void *data);
void showWarmOrColdTask(void *data);
void showTempLEDTask(void *data);

void readLightTask(void *data);
void PlaySongTask(void *data);
void showSpeedTask(void *data);
void showToneLEDTask(void *data);

/* INT5�� �ش�Ǵ� �ɿ��� �ܺ� ���ͷ�Ʈ�� �߻����� ���
 * �µ� ���� task�� �뷡 ���� task ���� task ��ȯ�� �Ͼ�� �ȴ�.
 */
ISR(INT5_vect)
{
	choice_task->OSFlagFlags = 0x00;
	
	// task == 0, �뷡 ���� task
	// task == 1, �µ� ���� task
	if(task)
	{
		task = 0;
		TIMSK |= (1 << TOIE2);							// timer/counter 2 �����÷ο� ���ͷ�Ʈ Ȱ��ȭ
		OSFlagPost(choice_task,0x01,OS_FLAG_SET,&err);	// choice_task�� flag�� 0x01�� ���� 	
	}
	else
	{
		task = 1;
		PORTA = 0x00;									// PORTA �� 0���� �ʱ�ȭ						
		TIMSK &= ~(1 << TOIE2);							// timer/counter 2 �����÷ο� ���ͷ�Ʈ ��Ȱ��ȭ
		OSFlagPost(choice_task,0x02,OS_FLAG_SET,&err);  // choice_task�� flag�� 0x02�� ����.
	}
	
	_delay_ms(70);
}

/* 1. �뷡 ���� task
 *	 - ������ ���
 * 2. �µ� ���� task
 *   - ���׸�Ʈ�� warm or cold�� ����� ������, ���� �µ��� ǥ���� ������ ����
 */
ISR(INT4_vect)
{
	// task == 0, �뷡 ���� task
	// task == 1, �µ� ���� task
	if(task)
	{
		choice_temp_grp->OSFlagFlags = 0x00;
		
		if(choice_temp) // warm or cold ��� => �µ� ������� ��ȯ
		{
			choice_temp = 0;
			OSFlagPost(choice_temp_grp,0x01,OS_FLAG_SET,&err); // choice_temp_grp�� flag�� 0x01�� ����
		}
		else // �µ� ��� => warm or cold ������� ��ȯ
		{
			choice_temp = 1;
			OSFlagPost(choice_temp_grp,0x02,OS_FLAG_SET,&err); // choice_temp_grp�� flag�� 0x02�� ����
		}
	}
	else
	{
		song = ++song % SONG_COUNT; // ����� �ش�Ǵ� index�� ����.
			
		pointer = 0; // ���� ���� index 0���� �ʱ�ȭ(ó������ ����ϱ� �����̴�.)
	}
		
	_delay_ms(70);
}

/* �ش�Ǵ� ���� ����ϱ� ���� �����÷ο츦 ����Ѵ�. */
ISR(TIMER2_OVF_vect)
{
	if (state == ON)
	{
		PORTB = 0x00;
		state = OFF;
	}
	else
	{
		PORTB = 0x10;
		state = ON;
	}

	TCNT2 = tone;
}

int main (void)
{
  OSInit();
  OS_ENTER_CRITICAL();
  task = 0;
  song = 0;
  pointer = 0;
  light = 0;
  choice_temp = 0;
  TCCR0 = 0x07;			// 1024����
  TIMSK = (1<<TOIE0)|(1 << TOIE2);   // �����÷ο� ���ͷ�Ʈ Ȱ��ȭ, TOIE0,TOIE2 ��Ʈ ��Ʈ
  TCNT0 = 256 - (CPU_CLOCK_HZ / OS_TICKS_PER_SEC / 1024);
  DDRA |= 0xff;	// ��� pin A�� ������� ������ش�.
  DDRE = 0xcf;  // PE4, PE5�� �Է����� �������.
  EICRB = 0x0A; // INT4, INT5 ���ͷ�Ʈ Ʈ���� ����(�ϰ� �������� ���ͷ�Ʈ �߻�)
  EIMSK = 0x30; // INT4, INT5 ���ͷ�Ʈ Ȱ��ȭ
  init_adc();   // ADC �ʱ�ȭ
  InitI2C();	// I2C �ʱ�ȭ
  
  sei();		// �������ͷ�Ʈ Ȱ��ȭ
 
  OS_EXIT_CRITICAL();
  
  /* Event Flag ���� */
  enable_temp = OSFlagCreate(0x00,&err);		
  choice_temp_grp = OSFlagCreate(0x01,&err);
  choice_speed_grp = OSFlagCreate(0x00,&err);
  choice_task = OSFlagCreate(0x01,&err);
  
  /* Mailbox ���� */
  mbox_tone = OSMboxCreate((void*)0);
  
  /* �뷡 ���� ���� Task ���� */
  OSTaskCreate(readLightTask,(void *)0,(void*)&TaskStk[0][TASK_STK_SIZE-1],0);
  OSTaskCreate(showSpeedTask, (void *)0, (void *)&TaskStk[1][TASK_STK_SIZE - 1], 1);
  OSTaskCreate(showToneLEDTask,(void *)0,(void*)&TaskStk[2][TASK_STK_SIZE-1],2);
  OSTaskCreate(PlaySongTask,(void *)0,(void*)&TaskStk[3][TASK_STK_SIZE-1],3);
  
  /* �µ� ���� ���� Task ���� */
  OSTaskCreate(TemperatureTask, (void *)0, (void *)&TaskStk[4][TASK_STK_SIZE - 1], 4);
  OSTaskCreate(showTempLEDTask, (void *)0, (void *)&TaskStk[5][TASK_STK_SIZE - 1], 5);
  OSTaskCreate(showTempTask, (void *)0, (void *)&TaskStk[6][TASK_STK_SIZE - 1], 6);
  OSTaskCreate(showWarmOrColdTask, (void *)0, (void *)&TaskStk[7][TASK_STK_SIZE - 1], 7);
		
  OSStart();
		
  return 0;
}

/* ADC �ʱ�ȭ */
void init_adc(){
	ADMUX=0x00;
	// 00000000
	// REFS(1:0) = "00" AREF(+5V) �������� ���
	// ADLAR = '0' ����Ʈ ������ ����
	// MUX(4:0) = "00000" ADC0 ���, �ܱ� �Է�
	ADCSRA = 0x87;
	// 10000111
	// ADEN = '1' ADC�� Enable
	// ADFR = '0' single conversion ���
	// ADPS(2:0) = "111" ���������Ϸ� 128����
}

/* ADC ���� �о�� ��ȯ */
unsigned short read_adc(){
	unsigned char adc_low,adc_high;
	unsigned short value;
	ADCSRA |= 0x40; // ADC start conversion, ADSC = '1'
	while((ADCSRA & (0x10)) != 0x10); // ADC ��ȯ �Ϸ� �˻�
	adc_low=ADCL;
	adc_high=ADCH;
	value = (adc_high <<8) | adc_low;
	
	return value;
}

/* I2C �ʱ�ȭ */
void InitI2C()
{
	PORTD = 3; 						// For Pull-up override value
	SFIOR &= ~(1 << PUD); 			// PUD
	TWSR = 0; 						// TWPS0 = 0, TWPS1 = 0
	TWBR = 32;						// for 100  K Hz bus clock
	TWCR = _BV(TWEA) | _BV(TWEN);	// TWEA = Ack pulse is generated
									// TWEN = TWI ������ �����ϰ� �Ѵ�
}

/* Cofiguration register ����, �ػ� ���� */
void write_twi_1byte_nopreset(UCHAR reg, UCHAR data)
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // START ����
	while(((TWCR & (1 << TWINT)) == 0x00) || ((TWSR & 0xf8) != 0x08 &&   (TWSR & 0xf8) != 0x10)); // ACK�� ��ٸ�
	TWDR = ATS75_ADDR | 0;  // SLA+W �غ�, W=0
	TWCR = (1 << TWINT) | (1 << TWEN);  // SLA+W ����
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x18);
	TWDR = reg;    // aTS75 Reg �� �غ�
	TWCR = (1 << TWINT) | (1 << TWEN);  // aTS75 Reg �� ����
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xF8) != 0x28);
	TWDR = data;    // DATA �غ�
	TWCR = (1 << TWINT) | (1 << TWEN);  // DATA ����
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xF8) != 0x28);
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // STOP ����
}

/* Temparture Register ���� */
void write_twi_0byte_nopreset(UCHAR reg)
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // START ����
	while(((TWCR & (1 << TWINT)) == 0x00) || ((TWSR & 0xf8) != 0x08 &&   (TWSR & 0xf8) != 0x10));  // ACK�� ��ٸ�
	TWDR = ATS75_ADDR | 0; // SLA+W �غ�, W=0
	TWCR = (1 << TWINT) | (1 << TWEN);  // SLA+W ����
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x18);
	TWDR = reg;    // aTS75 Reg �� �غ�
	TWCR = (1 << TWINT) | (1 << TWEN);  // aTS75 Reg �� ����
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xF8) != 0x28);
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // STOP ����
}

/* �µ��� �о�� ��ȯ�Ѵ�. */
int ReadTemperature(void)
{
	int value;
	
	TWCR = _BV(TWSTA) | _BV(TWINT) | _BV(TWEN); // START ����
	while(!(TWCR & _BV(TWINT)));

	TWDR = ATS75_ADDR | 1; //TEMP_I2C_ADDR + 1
	TWCR = _BV(TWINT) | _BV(TWEN);
	while(!(TWCR & _BV(TWINT)));

	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while(!(TWCR & _BV(TWINT)));

	// �µ������� 16bit �������� ���� �������Ƿ�
	// 8��Ʈ�� 2���� �޾ƾ� �Ѵ�.
	value = TWDR << 8; //ù��° data ����
	TWCR = _BV(TWINT) | _BV(TWEN);
	while(!(TWCR & _BV(TWINT)));

	value |= TWDR; // �ι�° data ����
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO); // STOP ����
	
	return value;
}

/* �µ��� �о�� �����Ѵ�. */
void TemperatureTask (void *data)
{
	write_twi_1byte_nopreset(ATS75_CONFIG_REG, 0x00); // 9��Ʈ, Normal
	write_twi_0byte_nopreset(ATS75_TEMP_REG);
	
	while (1)  {
		OSFlagPend(choice_task,0x02,OS_FLAG_WAIT_SET_ALL,0,&err); // �µ� ���� task�� �����ɶ����� block
		OSFlagPost(enable_temp,0x00,OS_FLAG_SET,&err);			  // enable_temp flag 0x00���� �ʱ�ȭ
		OS_ENTER_CRITICAL();
		temp = ReadTemperature(); // �µ��� �о��.
		OS_EXIT_CRITICAL();
	
		OSFlagPost(enable_temp,0x07,OS_FLAG_SET,&err);			  // �� task���� ������ �Ϸ�Ǿ����� �˸�.
		OSTimeDlyHMSM(0,0,0,100);
	}
}

/* ����µ��� ������ ���׸�Ʈ�� ��� */
void showTempTask(void *data)
{
	unsigned char value_int, value_deci, num[4];
	int i,value;
	DDRC = 0xff;
	DDRG = 0x0f;
	
	while(1)  
	{	
		OSFlagPend(choice_task,0x02,OS_FLAG_WAIT_SET_ALL,0,&err);		// �µ����� task�� ������ ������ block
		OSFlagPend(enable_temp,0x01,OS_FLAG_WAIT_SET_ALL,0,&err);		// �µ������� �Ϸ�Ǿ����� Ȯ�εɶ����� block
		OSFlagPend(choice_temp_grp,0x01,OS_FLAG_WAIT_SET_ALL,0,&err);   // ���� �µ� ǥ�� task�� Ȱ��ȭ�� ������ block
																																																																													 
		value = temp;
		if((value & 0x8000) != 0x8000)  // Sign ��Ʈ üũ
			num[3] = 19;
		else
		{
			num[3] = 17; // -�� ����
			value = (~value)-1;
		}
		
		value_int = (unsigned char)((value & 0x7f00) >> 8);
		value_deci = (unsigned char)(value & 0x00ff);
		
		num[2] = (value_int / 10) % 10;				// 10���ڸ� ����
		num[1] = value_int % 10;					// 1���ڸ� ����
		num[0] = ((value_deci & 0x80) == 0x80) * 5; // �Ҽ��Ʒ� ù°�ڸ� �� ����
		
		// ���׸�Ʈ�� ���� �µ� ���
		for(i=0; i<4; i++)
		{
			PORTC = digit[num[i]];
			PORTG = fnd_sel[i];
			
			if(i==1) 
				PORTC |= 0x80;
				
			_delay_ms(2);
		}
	}
}

/* warm or cold�� ���׸�Ʈ�� ǥ���Ѵ�. */
void showWarmOrColdTask(void *data)
{
	int now_state = 0;
	unsigned char value_int, num[4];
	int i,value;
	
	DDRC = 0xff;
	DDRG = 0x0f;

	while(1)
	{
		OSFlagPend(choice_task,0x02,OS_FLAG_WAIT_SET_ALL,0,&err); // �µ����� task�� ������ �� ���� block
		OSFlagPend(enable_temp,0x02,OS_FLAG_WAIT_SET_ALL,0,&err); // �µ������� �Ϸ�Ǿ����� Ȯ�ε� ������ block
		OSFlagPend(choice_temp_grp,0x02,OS_FLAG_WAIT_SET_ALL,0,&err); // warm or cold ��� task�� Ȱ��ȭ �ɶ����� block
		
		value = temp;
		if((value & 0x8000) != 0x8000)  // Sign ��Ʈ üũ
			num[3] = 19;
		else
		{
			num[3] = 17; // -�� ����
			value = (~value)-1;
		}

		value_int = (unsigned char)((value & 0x7f00) >> 8);
		
		if(value_int > 20)
			now_state = 1;
		else
			now_state = 0;

		for(i=0; i<4; i++)
		{
			if(now_state) // 20�� �ʰ��� ��� warm ��� 
				PORTC = warm[3-i];	
			else          // 20�� ������ ��� cold ���
				PORTC = cold[3-i];
			
			PORTG = fnd_sel[i];

			_delay_ms(2);
		}
	}
}

/* ���� �µ��� levelȭ �Ͽ� LED�� ����Ѵ�. */
void showTempLEDTask(void *data)
{
	unsigned char value_int;
	int temp_level = 0;

	while(1)
	{
		OSFlagPend(choice_task,0x02,OS_FLAG_WAIT_SET_ALL,0,&err); // �µ� ���� Task�� ������ ������ block
		OSFlagPend(enable_temp,0x04,OS_FLAG_WAIT_SET_ALL,0,&err); // �µ� ������ �Ϸ�Ǿ����� Ȯ�ε� ������ block
		
		value_int = (unsigned char)((temp & 0x7f00) >> 8);
		// 0�� ����, 40�� �ְ�� ����.
		temp_level =  value_int / 5;
		
		PORTA = (255 >> (8-temp_level));
		
		OSTimeDlyHMSM(0,0,0,100);
	}
}

/* ���� ������ ���� ADC���� �о�´�. */
void readLightTask(void *data)
{
	while(1)
	{	
		OSFlagPend(choice_task,0x01,OS_FLAG_WAIT_SET_ALL,0,&err); // �뷡 ���� task�� ���õ� ������ block																																															
		choice_speed_grp->OSFlagFlags = 0x00;
		OS_ENTER_CRITICAL();
		light = read_adc();
		OS_EXIT_CRITICAL();
		OSFlagPost(choice_speed_grp,0x03,OS_FLAG_SET,&err); // ADC�� �о���Ⱑ �Ϸ�Ǿ����� �˸�
		
		OSTimeDlyHMSM(0,0,0,500);
	}
}

/* �뷡�� �����Ѵ�. */
void PlaySongTask(void *data)
{	
	TCNT2 = DO;
	TCCR2 = 0x03;	 // 32����
	DDRB = 0x10;     // ���� ���� ��Ʈ(Port B�� 4�� ��Ʈ) ������� ����   
	double speed;
	
	while(1)
	{
		OSFlagPend(choice_task,0x01,OS_FLAG_WAIT_SET_ALL,0,&err);		// �뷡���� task�� ������ ������ block
		OSFlagPend(choice_speed_grp,0x01,OS_FLAG_WAIT_SET_ALL,0,&err);  // ���� ������ �Ϸ�� ������ block
		speed = light/100; 
		
		if(speed > 8)
			speed = 0.5; // speed �ܰ谡 8�ʰ��� ��� 1���
		else if(speed <=8 && speed >5)
			speed = 0.4; // speed �ܰ谡 5�ʰ� 8������ ��� 1.2���
		else 
			speed = 0.3; // speed �ܰ谡 5������ ��� 1.4���
		
		tone = song_list[song][pointer]; // ���� �뷡�� ������ ����
		OSMboxPost(mbox_tone,(void*)tone); // ���� ������ showToneLEDTask�� ����.
		 
		// ���� ������ �� ���� ���� ����� �� �ֵ��� ������ ��
		if(tone == EOS)
		{
			song = ++song % SONG_COUNT;
				
			pointer = 0;
			_delay_ms(300);
			continue;
		}
		
		// ��ǥ�� ��� timer/counter 2 �����÷ο� ���ͷ�Ʈ ��Ȱ��ȭ
		if(tone == REST)
		{
			TIMSK &= ~(1 << TOIE2);
			_delay_ms(tempo_list[song][pointer++]*speed);
		}
		else
		{
			TIMSK |= (1 << TOIE2);
			_delay_ms(tempo_list[song][pointer++]*speed);
			TIMSK &= ~(1 << TOIE2);
			_delay_ms(10);
		}
	}
}

/* ���� �������� ����Ѵ�. */
void showSpeedTask(void *data)
{
	int speed;
	int i,fnd[4];
	DDRC = 0xff;
	DDRG = 0x0f;
	
	while(1)
	{
		OSFlagPend(choice_task,0x01,OS_FLAG_WAIT_SET_ALL,0,&err);		// �뷡���� task�� ������ ������ block
		OSFlagPend(choice_speed_grp,0x02,OS_FLAG_WAIT_SET_ALL,0,&err);  // ���� ������ �Ϸ�� ������ block
		
		speed = light/100; 
		
		if(speed > 8)
			speed = 10;
		else if(speed <=8 && speed >5)
			speed = 12;
		else
			speed = 14;
		
		fnd[3] = digit[(speed/1000)%10];
		fnd[2] = digit[(speed/100)%10];
		fnd[1] = digit[(speed/10)%10];
		fnd[0] = digit[speed%10];
		
		for(i=0;i<4;i++) // �ڿ������� ���
		{
			PORTC = fnd[i];
		 	PORTG = fnd_sel[i];
			if(i == 1)
				PORTC |= 0x80;
			
			_delay_ms(2.5);
		}
		
		OSTimeDlyHMSM(0,0,0,10);
	}
}

/* ������ ������ levelȭ �Ͽ� ����Ѵ�. */
void showToneLEDTask(void *data)
{
	int level;
	while(1)
	{
		level = (int*) OSMboxPend(mbox_tone,0,&err); // PlaySongTask�κ��� ������ ������ ���� ������ block�ȴ�.
		level /= 21;
		PORTA = (0xff >> 8-level);
		
		OSTimeDlyHMSM(0,0,0,10);
	}
}