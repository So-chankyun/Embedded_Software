#include "includes.h"
#define F_SCK 40000UL  // SCK 클록 값 = 40 Khz

#define W 0x6A
#define A 0x77
#define R 0x50
#define M 0x55
#define C 0x58
#define O 0x5c
#define L 0x38
#define D 0x5e

/* 음정에 따른 TCNT값 정의 */
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
#define REST -2   // 쉼표
#define EOS -1    // End Of Song
#define SONG_COUNT 3		// 노래의 개수

#define ON 0
#define OFF 1

#define TASK_STK_SIZE  OS_TASK_DEF_STK_SIZE
#define N_TASKS 8
#define ATS75_ADDR 0x98 // 0b10011000, 7비트를 1비트 left shift
#define ATS75_CONFIG_REG 1
#define ATS75_TEMP_REG 0
#define UCHAR unsigned char // UCHAR 정의

void InitI2C();										  // I2C 초기화 함수
void write_twi_1byte_nopreset(UCHAR reg, UCHAR data); // 해상도를 설정할 때 사용
void write_twi_0byte_nopreset(UCHAR reg);			  // Temparture Register에 있는 결과를 가져올 때 사용
int ReadTemperature(void);							  // 온도를 읽어들여 저장함.
void init_adc();									  // ADC의 초기 상태 설정.
unsigned short read_adc();							  // ADC값을 읽어옴.

OS_STK       TaskStk[N_TASKS][TASK_STK_SIZE];		  // Task 스택 생성
OS_FLAG_GRP*	enable_temp;						  
OS_FLAG_GRP*	choice_temp_grp;
OS_FLAG_GRP*	choice_speed_grp;
OS_FLAG_GRP*	choice_task;
OS_EVENT*		mbox_tone;

volatile int state,tone;	// 음의 출력 설정을 위해 사용
volatile int pointer;		// 노래의 음을 지정할 때 사용
volatile int light;			// 읽어들인 ADC값을 저장.
volatile int temp;			// 읽어들인 온도를 저장.
volatile int choice_temp;   // showWarmOrColdTask와 showTempTask를 선택할 때 사용.
volatile int song;			// 노래를 선택할 때 사용
volatile int task;			// 노래 연주 task와 온도 측정 task 중 어떤 것을 실행할 지 결정할 때 사용.
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

const int *song_list[3] = {jinglebel,			// 징글벨
						   silent_night,		// 고요한 밤
						   look_out_the_window  // 창밖을 보라
						   };
const double *tempo_list[3] = {tempo_jinglebel,tempo_silent_night,tempo_look_out}; // 각 노래의 Tempo를 저장

/* Warm, Cold에 해당되는 세그먼트 16진수 값 */
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

/* INT5에 해당되는 핀에서 외부 인터럽트가 발생했을 경우
 * 온도 측정 task와 노래 연주 task 간에 task 전환이 일어나게 된다.
 */
ISR(INT5_vect)
{
	choice_task->OSFlagFlags = 0x00;
	
	// task == 0, 노래 연주 task
	// task == 1, 온도 측정 task
	if(task)
	{
		task = 0;
		TIMSK |= (1 << TOIE2);							// timer/counter 2 오버플로우 인터럽트 활성화
		OSFlagPost(choice_task,0x01,OS_FLAG_SET,&err);	// choice_task의 flag를 0x01로 설정 	
	}
	else
	{
		task = 1;
		PORTA = 0x00;									// PORTA 값 0으로 초기화						
		TIMSK &= ~(1 << TOIE2);							// timer/counter 2 오버플로우 인터럽트 비활성화
		OSFlagPost(choice_task,0x02,OS_FLAG_SET,&err);  // choice_task의 flag를 0x02로 설정.
	}
	
	_delay_ms(70);
}

/* 1. 노래 연주 task
 *	 - 다음곡 재생
 * 2. 온도 측정 task
 *   - 세그먼트에 warm or cold를 출력할 것인지, 현재 온도를 표시할 것인지 선택
 */
ISR(INT4_vect)
{
	// task == 0, 노래 연주 task
	// task == 1, 온도 측정 task
	if(task)
	{
		choice_temp_grp->OSFlagFlags = 0x00;
		
		if(choice_temp) // warm or cold 출력 => 온도 출력으로 전환
		{
			choice_temp = 0;
			OSFlagPost(choice_temp_grp,0x01,OS_FLAG_SET,&err); // choice_temp_grp의 flag를 0x01로 세팅
		}
		else // 온도 출력 => warm or cold 출력으로 전환
		{
			choice_temp = 1;
			OSFlagPost(choice_temp_grp,0x02,OS_FLAG_SET,&err); // choice_temp_grp의 flag를 0x02로 세팅
		}
	}
	else
	{
		song = ++song % SONG_COUNT; // 다음곡에 해당되는 index를 설정.
			
		pointer = 0; // 음정 선택 index 0으로 초기화(처음부터 재생하기 위함이다.)
	}
		
	_delay_ms(70);
}

/* 해당되는 음을 출력하기 위해 오버플로우를 사용한다. */
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
  TCCR0 = 0x07;			// 1024분주
  TIMSK = (1<<TOIE0)|(1 << TOIE2);   // 오버플로우 인터럽트 활성화, TOIE0,TOIE2 비트 세트
  TCNT0 = 256 - (CPU_CLOCK_HZ / OS_TICKS_PER_SEC / 1024);
  DDRA |= 0xff;	// 모든 pin A를 출력으로 만들어준다.
  DDRE = 0xcf;  // PE4, PE5를 입력으로 만들어줌.
  EICRB = 0x0A; // INT4, INT5 인터럽트 트리거 설정(하강 에지에서 인터럽트 발생)
  EIMSK = 0x30; // INT4, INT5 인터럽트 활성화
  init_adc();   // ADC 초기화
  InitI2C();	// I2C 초기화
  
  sei();		// 전역인터럽트 활성화
 
  OS_EXIT_CRITICAL();
  
  /* Event Flag 생성 */
  enable_temp = OSFlagCreate(0x00,&err);		
  choice_temp_grp = OSFlagCreate(0x01,&err);
  choice_speed_grp = OSFlagCreate(0x00,&err);
  choice_task = OSFlagCreate(0x01,&err);
  
  /* Mailbox 생성 */
  mbox_tone = OSMboxCreate((void*)0);
  
  /* 노래 연주 관련 Task 생성 */
  OSTaskCreate(readLightTask,(void *)0,(void*)&TaskStk[0][TASK_STK_SIZE-1],0);
  OSTaskCreate(showSpeedTask, (void *)0, (void *)&TaskStk[1][TASK_STK_SIZE - 1], 1);
  OSTaskCreate(showToneLEDTask,(void *)0,(void*)&TaskStk[2][TASK_STK_SIZE-1],2);
  OSTaskCreate(PlaySongTask,(void *)0,(void*)&TaskStk[3][TASK_STK_SIZE-1],3);
  
  /* 온도 측정 관련 Task 생성 */
  OSTaskCreate(TemperatureTask, (void *)0, (void *)&TaskStk[4][TASK_STK_SIZE - 1], 4);
  OSTaskCreate(showTempLEDTask, (void *)0, (void *)&TaskStk[5][TASK_STK_SIZE - 1], 5);
  OSTaskCreate(showTempTask, (void *)0, (void *)&TaskStk[6][TASK_STK_SIZE - 1], 6);
  OSTaskCreate(showWarmOrColdTask, (void *)0, (void *)&TaskStk[7][TASK_STK_SIZE - 1], 7);
		
  OSStart();
		
  return 0;
}

/* ADC 초기화 */
void init_adc(){
	ADMUX=0x00;
	// 00000000
	// REFS(1:0) = "00" AREF(+5V) 기준전압 사용
	// ADLAR = '0' 디폴트 오른쪽 정렬
	// MUX(4:0) = "00000" ADC0 사용, 단극 입력
	ADCSRA = 0x87;
	// 10000111
	// ADEN = '1' ADC를 Enable
	// ADFR = '0' single conversion 모드
	// ADPS(2:0) = "111" 프리스케일러 128분주
}

/* ADC 값을 읽어와 반환 */
unsigned short read_adc(){
	unsigned char adc_low,adc_high;
	unsigned short value;
	ADCSRA |= 0x40; // ADC start conversion, ADSC = '1'
	while((ADCSRA & (0x10)) != 0x10); // ADC 변환 완료 검사
	adc_low=ADCL;
	adc_high=ADCH;
	value = (adc_high <<8) | adc_low;
	
	return value;
}

/* I2C 초기화 */
void InitI2C()
{
	PORTD = 3; 						// For Pull-up override value
	SFIOR &= ~(1 << PUD); 			// PUD
	TWSR = 0; 						// TWPS0 = 0, TWPS1 = 0
	TWBR = 32;						// for 100  K Hz bus clock
	TWCR = _BV(TWEA) | _BV(TWEN);	// TWEA = Ack pulse is generated
									// TWEN = TWI 동작을 가능하게 한다
}

/* Cofiguration register 선택, 해상도 설정 */
void write_twi_1byte_nopreset(UCHAR reg, UCHAR data)
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // START 전송
	while(((TWCR & (1 << TWINT)) == 0x00) || ((TWSR & 0xf8) != 0x08 &&   (TWSR & 0xf8) != 0x10)); // ACK를 기다림
	TWDR = ATS75_ADDR | 0;  // SLA+W 준비, W=0
	TWCR = (1 << TWINT) | (1 << TWEN);  // SLA+W 전송
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x18);
	TWDR = reg;    // aTS75 Reg 값 준비
	TWCR = (1 << TWINT) | (1 << TWEN);  // aTS75 Reg 값 전송
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xF8) != 0x28);
	TWDR = data;    // DATA 준비
	TWCR = (1 << TWINT) | (1 << TWEN);  // DATA 전송
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xF8) != 0x28);
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // STOP 전송
}

/* Temparture Register 선택 */
void write_twi_0byte_nopreset(UCHAR reg)
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // START 전송
	while(((TWCR & (1 << TWINT)) == 0x00) || ((TWSR & 0xf8) != 0x08 &&   (TWSR & 0xf8) != 0x10));  // ACK를 기다림
	TWDR = ATS75_ADDR | 0; // SLA+W 준비, W=0
	TWCR = (1 << TWINT) | (1 << TWEN);  // SLA+W 전송
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x18);
	TWDR = reg;    // aTS75 Reg 값 준비
	TWCR = (1 << TWINT) | (1 << TWEN);  // aTS75 Reg 값 전송
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xF8) != 0x28);
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // STOP 전송
}

/* 온도를 읽어와 반환한다. */
int ReadTemperature(void)
{
	int value;
	
	TWCR = _BV(TWSTA) | _BV(TWINT) | _BV(TWEN); // START 전송
	while(!(TWCR & _BV(TWINT)));

	TWDR = ATS75_ADDR | 1; //TEMP_I2C_ADDR + 1
	TWCR = _BV(TWINT) | _BV(TWEN);
	while(!(TWCR & _BV(TWINT)));

	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while(!(TWCR & _BV(TWINT)));

	// 온도센서는 16bit 기준으로 값을 가져오므로
	// 8비트씩 2번을 받아야 한다.
	value = TWDR << 8; //첫번째 data 수신
	TWCR = _BV(TWINT) | _BV(TWEN);
	while(!(TWCR & _BV(TWINT)));

	value |= TWDR; // 두번째 data 수신
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO); // STOP 전송
	
	return value;
}

/* 온도를 읽어와 저장한다. */
void TemperatureTask (void *data)
{
	write_twi_1byte_nopreset(ATS75_CONFIG_REG, 0x00); // 9비트, Normal
	write_twi_0byte_nopreset(ATS75_TEMP_REG);
	
	while (1)  {
		OSFlagPend(choice_task,0x02,OS_FLAG_WAIT_SET_ALL,0,&err); // 온도 측정 task로 설정될때까지 block
		OSFlagPost(enable_temp,0x00,OS_FLAG_SET,&err);			  // enable_temp flag 0x00으로 초기화
		OS_ENTER_CRITICAL();
		temp = ReadTemperature(); // 온도를 읽어옴.
		OS_EXIT_CRITICAL();
	
		OSFlagPost(enable_temp,0x07,OS_FLAG_SET,&err);			  // 각 task에게 측정이 완료되었음을 알림.
		OSTimeDlyHMSM(0,0,0,100);
	}
}

/* 현재온도를 섭씨로 세그먼트에 출력 */
void showTempTask(void *data)
{
	unsigned char value_int, value_deci, num[4];
	int i,value;
	DDRC = 0xff;
	DDRG = 0x0f;
	
	while(1)  
	{	
		OSFlagPend(choice_task,0x02,OS_FLAG_WAIT_SET_ALL,0,&err);		// 온도측정 task로 설정될 때까지 block
		OSFlagPend(enable_temp,0x01,OS_FLAG_WAIT_SET_ALL,0,&err);		// 온도측정이 완료되었음이 확인될때까지 block
		OSFlagPend(choice_temp_grp,0x01,OS_FLAG_WAIT_SET_ALL,0,&err);   // 섭씨 온도 표시 task로 활성화될 때까지 block
																																																																													 
		value = temp;
		if((value & 0x8000) != 0x8000)  // Sign 비트 체크
			num[3] = 19;
		else
		{
			num[3] = 17; // -를 지정
			value = (~value)-1;
		}
		
		value_int = (unsigned char)((value & 0x7f00) >> 8);
		value_deci = (unsigned char)(value & 0x00ff);
		
		num[2] = (value_int / 10) % 10;				// 10의자리 저장
		num[1] = value_int % 10;					// 1의자리 저장
		num[0] = ((value_deci & 0x80) == 0x80) * 5; // 소수아래 첫째자리 값 저장
		
		// 세그먼트에 현재 온도 출력
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

/* warm or cold로 세그먼트에 표시한다. */
void showWarmOrColdTask(void *data)
{
	int now_state = 0;
	unsigned char value_int, num[4];
	int i,value;
	
	DDRC = 0xff;
	DDRG = 0x0f;

	while(1)
	{
		OSFlagPend(choice_task,0x02,OS_FLAG_WAIT_SET_ALL,0,&err); // 온도측정 task로 설정될 때 까지 block
		OSFlagPend(enable_temp,0x02,OS_FLAG_WAIT_SET_ALL,0,&err); // 온도측정이 완료되었음이 확인될 때까지 block
		OSFlagPend(choice_temp_grp,0x02,OS_FLAG_WAIT_SET_ALL,0,&err); // warm or cold 출력 task로 활성화 될때까지 block
		
		value = temp;
		if((value & 0x8000) != 0x8000)  // Sign 비트 체크
			num[3] = 19;
		else
		{
			num[3] = 17; // -를 지정
			value = (~value)-1;
		}

		value_int = (unsigned char)((value & 0x7f00) >> 8);
		
		if(value_int > 20)
			now_state = 1;
		else
			now_state = 0;

		for(i=0; i<4; i++)
		{
			if(now_state) // 20도 초과일 경우 warm 출력 
				PORTC = warm[3-i];	
			else          // 20도 이하일 경우 cold 출력
				PORTC = cold[3-i];
			
			PORTG = fnd_sel[i];

			_delay_ms(2);
		}
	}
}

/* 현재 온도를 level화 하여 LED로 출력한다. */
void showTempLEDTask(void *data)
{
	unsigned char value_int;
	int temp_level = 0;

	while(1)
	{
		OSFlagPend(choice_task,0x02,OS_FLAG_WAIT_SET_ALL,0,&err); // 온도 측정 Task로 설정될 때까지 block
		OSFlagPend(enable_temp,0x04,OS_FLAG_WAIT_SET_ALL,0,&err); // 온도 측정이 완료되었음이 확인될 때까지 block
		
		value_int = (unsigned char)((temp & 0x7f00) >> 8);
		// 0을 최저, 40을 최고로 본다.
		temp_level =  value_int / 5;
		
		PORTA = (255 >> (8-temp_level));
		
		OSTimeDlyHMSM(0,0,0,100);
	}
}

/* 현재 광량에 따른 ADC값을 읽어온다. */
void readLightTask(void *data)
{
	while(1)
	{	
		OSFlagPend(choice_task,0x01,OS_FLAG_WAIT_SET_ALL,0,&err); // 노래 연주 task가 선택될 때까지 block																																															
		choice_speed_grp->OSFlagFlags = 0x00;
		OS_ENTER_CRITICAL();
		light = read_adc();
		OS_EXIT_CRITICAL();
		OSFlagPost(choice_speed_grp,0x03,OS_FLAG_SET,&err); // ADC값 읽어오기가 완료되었음을 알림
		
		OSTimeDlyHMSM(0,0,0,500);
	}
}

/* 노래를 연주한다. */
void PlaySongTask(void *data)
{	
	TCNT2 = DO;
	TCCR2 = 0x03;	 // 32분주
	DDRB = 0x10;     // 버저 연결 포트(Port B의 4번 비트) 출력으로 설정   
	double speed;
	
	while(1)
	{
		OSFlagPend(choice_task,0x01,OS_FLAG_WAIT_SET_ALL,0,&err);		// 노래연주 task로 설정될 때까지 block
		OSFlagPend(choice_speed_grp,0x01,OS_FLAG_WAIT_SET_ALL,0,&err);  // 광량 측정이 완료될 때까지 block
		speed = light/100; 
		
		if(speed > 8)
			speed = 0.5; // speed 단계가 8초과일 경우 1배속
		else if(speed <=8 && speed >5)
			speed = 0.4; // speed 단계가 5초과 8이하일 경우 1.2배속
		else 
			speed = 0.3; // speed 단계가 5이하일 경우 1.4배속
		
		tone = song_list[song][pointer]; // 현재 노래의 음정을 지정
		OSMboxPost(mbox_tone,(void*)tone); // 현재 음정을 showToneLEDTask로 전송.
		 
		// 곡이 끝났을 때 다음 곡을 재생할 수 있도록 설정해 줌
		if(tone == EOS)
		{
			song = ++song % SONG_COUNT;
				
			pointer = 0;
			_delay_ms(300);
			continue;
		}
		
		// 쉼표일 경우 timer/counter 2 오버플로우 인터럽트 비활성화
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

/* 현재 몇배속인지 출력한다. */
void showSpeedTask(void *data)
{
	int speed;
	int i,fnd[4];
	DDRC = 0xff;
	DDRG = 0x0f;
	
	while(1)
	{
		OSFlagPend(choice_task,0x01,OS_FLAG_WAIT_SET_ALL,0,&err);		// 노래연주 task로 설정될 때까지 block
		OSFlagPend(choice_speed_grp,0x02,OS_FLAG_WAIT_SET_ALL,0,&err);  // 광량 측정이 완료될 때까지 block
		
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
		
		for(i=0;i<4;i++) // 뒤에서부터 출력
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

/* 현재의 음정을 level화 하여 출력한다. */
void showToneLEDTask(void *data)
{
	int level;
	while(1)
	{
		level = (int*) OSMboxPend(mbox_tone,0,&err); // PlaySongTask로부터 설정된 음정을 받을 때까지 block된다.
		level /= 21;
		PORTA = (0xff >> 8-level);
		
		OSTimeDlyHMSM(0,0,0,10);
	}
}