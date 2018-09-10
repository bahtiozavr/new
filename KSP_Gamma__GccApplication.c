/*
 * KSP_Gamma_GccApplication.c
 *
 * Created: 06.02.2018 9:43:05
 *  Author: GEFS_Weimer
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "twi.h"
#include "spi.h"
#include "UART.h"
#include "DAC.h"
#include "EEPROM.h"
#include "KSP_lib.h"

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)	((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)	 ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

#define GAMMA_02__	2
#define GAMMA_1__	10
#define GAMMA_30__	30

#define EEPROM_N						10

//16 Bit
#define EE_KSP_HIGHVOLT_ADDR			0x10
//16 Bit
#define EE_KSP_THRESHOLDMEAS_ADDR		0x30
//8 Bit
#define EE_KSP_NUMBERTRESH_ADDR			0x50
//8 Bit
#define EE_KSP_GAIN_ADDR				0x70
//16 Bit
#define EE_KSP_THRESHOLDMEASOFF_ADDR	0x90 

//16 Bit
#define EE_BASE_NUMBER_ADDR				0x100
//32 Bit
#define EE_BASE_HWORK_ADDR				0x120
//16 Bit
#define EE_BASE_TYPE_ADDR				0x140
//8 Bit
#define EE_BASE_DATA_WEEK_ADDR			0x160
//8 Bit
#define EE_BASE_DATA_YEAR_ADDR			0x180

uint32_t TIM0_Cnt_32t = 0;
uint32_t TIM0_Cnt_32t_old = 0; 
uint32_t Gamma_01 = 0, Gamma_02 = 0;
uint32_t Gamma_1 = 0, Gamma_30 = 0;
uint32_t Gammat_02 = 0;
uint32_t Gammat_1 = 0, Gammat_30 = 0;

uint16_t DAC_Value[4] = {0, 0, 0, 0};
uint16_t HighVoltage = 850;
uint16_t ThresholdMeasure = 350;
uint8_t NumberThreshold = 2;
uint8_t Gain = 0xC8;
uint16_t ThresholdMeasureOffset = 800;
//uint16_t sss[475];
//uint16_t sss_cnt = 0;

void DAC_SPI_Init(void)
{
	DDRB |= (1<<PORTB0);
	PORTB &= ~(1<<PORTB0);
}

void DAC_SPI_Send(uint8_t Addr, uint16_t data)
{
	//data = (data << 4) & 0xFFFC;
	//data = 0xffff;
	uint8_t buf[3] = {((DAC_WRREGN_UPDALL << 3) + Addr), ((data >> 4) & 0xFF), ((data<<4) & 0xF0)};
	SPI_SS_Off(SPI_PORT_SS_0, SPI_SS_0);
	//_delay_us(1);
	SPI_WriteArray(3, buf);
	/*SPI_WriteByte(0x00);
	SPI_WriteByte((data >> 8) & 0xFF);
	SPI_WriteByte(data & 0xFF);*/
	//_delay_us(1);
	SPI_SS_On(SPI_PORT_SS_0, SPI_SS_0);
	
	/*
	(command & B00111000) | (address & B00000111)
	*/
}

void DAC_SPI_Reset(uint8_t mode)
{
	//uint8_t buf[3] = {DAC_RST, 0, mode};
	SPI_SS_Off(SPI_PORT_SS_0, SPI_SS_0);
	//SPI_WriteArray(3, buf);
	SPI_WriteByte(0b101000);
	SPI_WriteByte(0x00);
	SPI_WriteByte(mode);
	SPI_SS_On(SPI_PORT_SS_0, SPI_SS_0);	
}

void MUX_SPI_Send(uint8_t value)
{
	SPI_SS_Off(SPI_PORT_SS_1, SPI_SS_1);
	SPI_WriteByte(value);
	SPI_SS_On(SPI_PORT_SS_1, SPI_SS_1);		
}

ISR(TIMER0_OVF_vect)
{
	TIM0_Cnt_32t += 0x100;
	//if(TIM0_Cnt_32t > 0x00FFFFFF) TIM0_Cnt_32t = 0;
}

uint8_t TIM2_Cnt = 0;
uint8_t Gamma_Cnt = 0;
ISR(TIMER2_COMPA_vect)
{
	TIM2_Cnt++;
	if(TIM2_Cnt >= 25){ 
		TIM2_Cnt = 0;
		uint32_t tmp = TIM0_Cnt_32t + TCNT0;
		if(tmp >= TIM0_Cnt_32t_old)
			Gamma_01 = tmp - TIM0_Cnt_32t_old;
		else 
			Gamma_01 = (0xFFFFFFFF - TIM0_Cnt_32t_old) + tmp + 1;
		TIM0_Cnt_32t_old = tmp; 
		//Gamma_01 = ((TIM0_Cnt_32t<<8) & 0xFFFFFF00) + TCNT0;
		Gammat_02 += Gamma_01;
		Gammat_1 += Gamma_01;
		Gammat_30 += Gamma_01;
		
		Gamma_Cnt++;
		if((Gamma_Cnt % GAMMA_02__) == 0) {
			Gamma_02 = Gammat_02;
			Gammat_02 = 0;
		}
		if((Gamma_Cnt % GAMMA_1__) == 0) {
			Gamma_1 = Gammat_1;
			Gammat_1 = 0;
		}
		if((Gamma_Cnt % GAMMA_30__) == 0) {
			Gamma_30 = Gammat_30;
			Gammat_30 = 0;
			Gamma_Cnt = 0;
		}
	}
	//PORTD ^= (1<<PORTD5);
}

ISR(WDT_vect){
	PORTD = 0x00;
}

int main(void)
{
	wdt_disable();
	wdt_enable(WDTO_15MS);
	WDTCSR |= (1<<WDIF) | (1<<WDIE);
	wdt_reset();
	
	DDRD = (1<<PORTD6) | (1<<PORTD5) | (1<<PORTD1) | (0<<PORTD0);
	PORTD = 0x00;
	DDRB |= (1<<PORTB1);
	PORTB = 0x00;
	
	// Timer/Counter 0 initialization
	// Clock source: T0 pin Falling Edge
	// Mode: Normal top=0xFF
	// OC0A output: Disconnected
	// OC0B output: Disconnected
	TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
	TCCR0B=(0<<WGM02) | (1<<CS02) | (1<<CS01) | (0<<CS00);
	TCNT0=0x00;
	OCR0A=0x00;
	OCR0B=0x00;
	
	// Timer/Counter 0 Interrupt(s) initialization
	TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (1<<TOIE0);
	
	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: 16000,000 kHz
	// Mode: CTC top=OCR1A
	// OC1A output: Disconnected
	// OC1B output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer Period: 0,01 ms
	// Timer1 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: On
	// Compare B Match Interrupt: Off
	TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
	TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);
	TCNT1H=0x00;
	TCNT1L=0x00;
	ICR1H=0x00;
	ICR1L=0x00;
	OCR1AH=0x00;
	OCR1AL=0x9F;
	OCR1BH=0x00;
	OCR1BL=0x00;
	
	// Timer/Counter 1 Interrupt(s) initialization
	TIMSK1=(0<<ICIE1) | (0<<OCIE1B) | (0<<OCIE1A) | (0<<TOIE1);
	
	// Timer/Counter 2 initialization
	// Clock source: System Clock
	// Clock value: 62,500 kHz
	// Mode: CTC top=OCR2A
	// OC2A output: Disconnected
	// OC2B output: Disconnected
	// Timer Period: 4 ms
	ASSR=(0<<EXCLK) | (0<<AS2);
	TCCR2A=(0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (1<<WGM21) | (0<<WGM20);
	TCCR2B=(0<<WGM22) | (1<<CS22) | (1<<CS21) | (0<<CS20);
	TCNT2=0x00;
	OCR2A=0xF9;
	OCR2B=0x00;
	
	// Timer/Counter 2 Interrupt(s) initialization
	TIMSK2=(0<<OCIE2B) | (1<<OCIE2A) | (0<<TOIE2);
	
	
	// USART initialization
	// Communication Parameters: 8 Data, 1 Stop, No Parity
	// USART Receiver: On
	// USART Transmitter: On
	// USART0 Mode: Asynchronous
	// USART Baud Rate: 19200
	UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
	UCSR0B=(1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
	UCSR0C=(0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
	UBRR0H=0x00;
	UBRR0L=0x33;
	
	UART_SendString(UART0, (uint8_t *)"UART0_AVR_Mega88PA\r\n");
	SPI_Init();
	DAC_SPI_Reset(DAC_RSTMODE_1);
	Gain = Read_Value_EEPROM_8(EE_KSP_GAIN_ADDR, EEPROM_N);
	HighVoltage = Read_Value_EEPROM_16(EE_KSP_HIGHVOLT_ADDR, EEPROM_N);
	if(HighVoltage > 1250) HighVoltage = 1250; else if(HighVoltage < 600) HighVoltage = 600;
	ThresholdMeasureOffset = Read_Value_EEPROM_16(EE_KSP_THRESHOLDMEASOFF_ADDR, EEPROM_N);
	if(ThresholdMeasureOffset > 4095) ThresholdMeasureOffset = 0;
	ThresholdMeasure = Read_Value_EEPROM_16(EE_KSP_THRESHOLDMEAS_ADDR, EEPROM_N);
	if(ThresholdMeasure > ((4095 - ThresholdMeasureOffset) / 10)) ThresholdMeasure = 50;
	NumberThreshold = Read_Value_EEPROM_8(EE_KSP_NUMBERTRESH_ADDR, EEPROM_N);
	if(NumberThreshold > 10) NumberThreshold = 10; else	if(NumberThreshold < 1) NumberThreshold = 1;
	wdt_reset();
	
	for(uint8_t i = 0; i < 100; i++){
		_delay_ms(10);
		wdt_reset();
	}
	
	PORTB |= (1<<PORTB1);
	
	DAC_Value[0] = HighVoltage;
	DAC_SPI_Send(DAC_ADDRA, DAC_Value[0]);
	DAC_Value[1] = ThresholdMeasure * NumberThreshold + ThresholdMeasureOffset;
	DAC_SPI_Send(DAC_ADDRB, DAC_Value[1]);
	MUX_SPI_Send(Gain);
	
	KSP_KSGEN_CurrentFreqSet(125);
	KSP_KSGEN_CurrentRun();
	wdt_reset();
	sei();
	uint16_t cnt = 100;
	
	///// BAHA'S CODE ADDITION
	float HV;
	/////
	
    while(1)
    {
	//TODO:: Please write your application code 
		_delay_us(10);
		if(cnt)
			wdt_reset();		
		if(UART_Read_Flag_RX()){
			uint8_t send_buf[32] = {UART_Buf[0], UART_Buf[1], '*', 0,'*',0,'*',0,0,0,'*',0};
			if(UART_Buf[0] == 'S'){
				
			}
			else if(UART_Buf[0] == 'A'){ //Установка
				switch (UART_Buf[1]){
					case 'A':{
						HighVoltage =  HexToU16(UART_Buf, 2);
						///// BAHA'S CODE ADDITION
						HV=HighVoltage*60.3307-35543,1688;
						HighVoltage=HV/10;
						if (HighVoltage>3987) HighVoltage=3987;
						/////
						DAC_Value[0] = HighVoltage;
						DAC_SPI_Send(DAC_ADDRA, DAC_Value[0]);
						break;
					}
					case 'B':{
						ThresholdMeasure =  HexToU16(UART_Buf, 2);
						DAC_Value[1] = ThresholdMeasure * NumberThreshold + ThresholdMeasureOffset;
						DAC_SPI_Send(DAC_ADDRB, DAC_Value[1]);
						break;
					}
					case 'C':{
						NumberThreshold = HexToU8(UART_Buf, 2);
						DAC_Value[1] = ThresholdMeasure * NumberThreshold + ThresholdMeasureOffset;
						DAC_SPI_Send(DAC_ADDRB, DAC_Value[1]);
						break;
					}
					case 'D':{
						Gain = HexToU8(UART_Buf, 2);
						MUX_SPI_Send(Gain);
						break;
					}
					case 'F':
						ThresholdMeasureOffset = HexToU16(UART_Buf, 2);
						DAC_Value[1] = ThresholdMeasure * NumberThreshold + ThresholdMeasureOffset;
						DAC_SPI_Send(DAC_ADDRB, DAC_Value[1]);
						break;
					default: 
						send_buf[2] = '?';
						break;				
				}
			}
			else if (UART_Buf[0] == 'B'){
				switch (UART_Buf[1]){
					case 'A':{
						//HighVoltage
						ascii_16_m(HighVoltage, send_buf, 2);
						break;
					}
					case 'B':{
						ascii_16_m(ThresholdMeasure, send_buf, 2);
						break;
					}
					case 'C':{
						ascii_8_m(NumberThreshold, send_buf, 2);
						break;
					}
					case 'D':{
						ascii_8_m(Gain, send_buf, 2);
						break;
					}
					case 'E':{
						ascii_32_m(Gamma_1, send_buf, 2);
						break;
					}
					default:
						send_buf[2] = '?';
						break;
				}
			}
			else if (UART_Buf[0] == 'G'){
				switch(UART_Buf[1]){
					case 'A':{
						KSP_KSGEN_CurrentFreqSet(HexToU8(UART_Buf,2));
						break;
					}
					case 'B':{
						KSP_KSGEN_CurrentStop();
						break;
					}
					case 'C':{
						KSP_KSGEN_CurrentRun();
						break;
					}
					case 'P':{
						KSP_KSGEN_CurrentPositive();
						break;
					}
					case 'N':{
						KSP_KSGEN_CurrentNegative();
						break;
					}
					default: 
						send_buf[2] = '?';
						break;
				}
			}
			else if (UART_Buf[0] == 'C'){
				if (UART_Buf[1] == 'A'){
					Write_Value_EEPROM_8	(Gain,						EE_KSP_GAIN_ADDR,				EEPROM_N,	true);
					Write_Value_EEPROM_8	(NumberThreshold,			EE_KSP_NUMBERTRESH_ADDR,		EEPROM_N,	true);
					Write_Value_EEPROM_16	(ThresholdMeasure,			EE_KSP_THRESHOLDMEAS_ADDR,		EEPROM_N,	true);
					Write_Value_EEPROM_16	(ThresholdMeasureOffset,	EE_KSP_THRESHOLDMEASOFF_ADDR,	EEPROM_N,	true);
					Write_Value_EEPROM_16	(HighVoltage,				EE_KSP_HIGHVOLT_ADDR,			EEPROM_N,	true);		
				}
				else{ 
					send_buf[2] = '?';
					break;
				}
			}
			else if(UART_Buf[0] == 'F'){
				uint8_t addr = 0;
				switch (UART_Buf[1]){
					case 'A': case 'B': case 'C': case 'D':
						addr = UART_Buf[1] - 'A' + DAC_ADDRA;
						break;
					case 'L': 
						addr = DAC_ADDRALL;
						break;
					default: 
						break;
				}
				cnt = HexToU16(UART_Buf, 2);
				DAC_SPI_Send(addr, cnt);
			}
			/*if(UART_Buf[0] == 'A') addr = DAC_ADDRA;
			else if(UART_Buf[0] == 'B') addr = DAC_ADDRB;
			else if(UART_Buf[0] == 'C') addr = DAC_ADDRC;
			else if(UART_Buf[0] == 'D') addr = DAC_ADDRD;
			else if(UART_Buf[0] == 'L') addr = DAC_ADDRALL;*/
			else if(UART_Buf[0] == 'M'){
				MUX_SPI_Send(HexToU8(UART_Buf, 1));
			}
			else if(UART_Buf[0] == 'E'){
				uint8_t buf[20] = {'E',0,0,0,0,'*','\r',0,0,'*', '\r',0};
				
				ascii_32_m( Gamma_01, buf, 1);
				UART_SendString(UART0, buf);
				
				ascii_32_m( Gamma_02, buf, 1);
				UART_SendString(UART0, buf);
				
				ascii_32_m( Gamma_1, buf, 1);
				UART_SendString(UART0, buf);
				
				ascii_32_m( Gamma_30, buf, 1);
				UART_SendString(UART0, buf);
				
				UART_SendString(UART0, (uint8_t *)"\r\n");
			}
			else if(UART_Buf[0] == 'W'){ 
				cnt = 0;
				break;
			}	
			else send_buf[2] = '?';
			UART_SendString(UART0, send_buf);
		}
    }
}