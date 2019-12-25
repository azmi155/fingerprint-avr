/*******************************************************
This program was created by the
CodeWizardAVR V3.12 Advanced
Automatic Program Generator
© Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 12/23/2019
Author  : 
Company : 
Comments: 


Chip type               : ATmega8535
Program type            : Application
AVR Core Clock frequency: 8.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 128
*******************************************************/

#include <mega8535.h>

// Alphanumeric LCD functions
#include <alcd.h>

// Declare your global variables here

//------------------------------------------------------------------------------------------
     #define F_CPU 8000000ul
#include <io.h>
#include <delay.h>
#include <avr/interrupt.h>

/***MACROS*/

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define uchar unsigned char
#define uint unsigned int

#define LCDPORTDIR DDRB
#define LCDPORT PORTB
#define rs 0
#define rw 1
#define en 2

#define RSLow (LCDPORT&=~(1<<rs))
#define RSHigh (LCDPORT|=(1<<rs))
#define RWLow (LCDPORT&=~(1<<rw))
#define ENLow (LCDPORT&=~(1<<en))
#define ENHigh (LCDPORT|=(1<<en))

#define KeyPORTdir DDRA
#define key PINA
#define KeyPORT PORTA
//After this, we have declared some variables and arrays for fingerprint command and response. We have also added some functions for fetching and setting data to RTC.


  void RTC_stp()
{ 
TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO);           //stop communication
}

void RTC_read()
{ 
TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
while((TWCR&0x80)==0x00);
TWDR=0xD0;                                         //RTC write (slave address)
TWCR=(1<<TWINT)|(1<<TWEN);
while(!(TWCR&(1<<TWINT)));
TWDR=0x00;                                         //RTC write (word address)
TWCR=(1<<TWINT)|(1<<TWEN);
while(!(TWCR&(1<<TWINT)));
TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);              //start RTC  communication again
while ((TWCR&0x80)==0x00);
TWDR=0xD1;                                        // RTC command to read
TWCR=(1<<TWINT)|(1<<TWEN);
while(!(TWCR&(1<<TWINT)));  
}

void serialbegin()
{
  UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
  UBRRH = (BAUD_PRESCALE >> 8);
  UBRRL = BAUD_PRESCALE;
  UCSRB=(1<<RXEN)|(1<<TXEN)|(1<<RXCIE);
  sei();
}

ISR(USART_RXC_vect)
{
char ch=UDR;
buf[ind++]=ch;
if(ind>0)
flag=1;
serial1Write(ch);
}

void serialwrite(char ch)
{
while ((UCSRA & (1 << UDRE)) == 0);
UDR = ch;
}

void serialprint(char *str)
{
    while(*str)
    {
        serialwrite(*str++);
    }
}


void SerialSoftWrite(char ch)
{
PORTD&=~(1<<7);
_delay_us(104);
for(int i=0;i<8;i++)
{
if(ch & 1)
PORTD|=(1<<7);
else
PORTD&=~(1<<7);
_delay_us(104);
ch>>=1;
}
PORTD|=(1<<7);
_delay_us(104);
}

void SerialSoftPrint(char *str)
{
while(*str)
{
SerialSoftWrite(*str);
str++;
}
}

int eeprom_write(unsigned int add,unsigned char data)
{
while(EECR&(1<<EEWE));
EEAR=add;
EEDR=data;
EECR|=(1<<EEMWE);
EECR|=(1<<EEWE);
return 0;
}
char eeprom_read(unsigned int add)
{
while(EECR & (1<<EEWE));
EEAR=add;
EECR|=(1<<EERE);
return EEDR;
}

void matchFinger()
{
    //  lcdwrite(1,CMD);
    //  lcdprint("Place Finger"); 
    //  lcdwrite(192,CMD);
    //  _delay_ms(2000);
     if(!sendcmd2fp((char *)&f_detect[0],sizeof(f_detect)))
     {
         if(!sendcmd2fp((char *)&f_imz2ch1[0],sizeof(f_imz2ch1)))
         {
            if(!sendcmd2fp((char *)&f_search[0],sizeof(f_search)))
            {
LEDHigh;
buzzer(200);
                uint id= data[0];
                     id<<=8;
                     id+=data[1];
                uint score=data[2];
                        score<<=8;
                        score+=data[3];
                (void)sprintf((char *)buf1,"Id: %d",(int)id);
                lcdwrite(1,CMD);
                lcdprint((char *)buf1);
saveData(id);

_delay_ms(1000);
lcdwrite(1,CMD);
lcdprint("Attendance");
lcdwrite(192,CMD);
lcdprint("Registered");
_delay_ms(2000);
LEDLow;


void deleteFinger()
{
    id=getId();
   f_delete[10]=id>>8 & 0xff;
   f_delete[11]=id & 0xff;
   f_delete[14]=(21+id)>>8 & 0xff;
   f_delete[15]=(21+id) & 0xff;
   if(!sendcmd2fp(&f_delete[0],sizeof(f_delete)))
  {
     lcdwrite(1,CMD);
     sprintf((char *)buf1,"Finger ID %d ",id);
     lcdprint((char *)buf1);
     lcdwrite(192, CMD);
     lcdprint("Deleted Success");
     
  }
   else
   {
       lcdwrite(1,CMD);
       lcdprint("Error");
   }
   _delay_ms(2000);
}

/*function to show attendence data on serial moinitor using softserial pin PD7*/
void ShowAttendance()
{
char buf[128];
lcdwrite(1,CMD);
lcdprint("Downloding....");
SerialSoftPrintln("Attendance Record");
SerialSoftPrintln(" ");
SerialSoftPrintln("S.No        ID1            ID2            Id3            ID4            ID5    ");
//serialprintln("Attendance Record");
//serialprintln(" ");
//serialprintln("S.No            ID1                     ID2                     Id3                     ID4                     ID5");
for(int cIndex=1;cIndex<=8;cIndex++)
{
sprintf((char *)buf,"%d    "
"%d:%d:%d  %d/%d/20%d    "  
"%d:%d:%d  %d/%d/20%d    "  
"%d:%d:%d  %d/%d/20%d    "  
"%d:%d:%d  %d/%d/20%d    "  
"%d:%d:%d  %d/%d/20%d    ",
cIndex,
eeprom_read((cIndex*6)),eeprom_read((cIndex*6)+1),eeprom_read((cIndex*6)+2),eeprom_read((cIndex*6)+3),eeprom_read((cIndex*6)+4),eeprom_read((cIndex*6)+5),
eeprom_read((cIndex*6)+48),eeprom_read((cIndex*6)+1+48),eeprom_read((cIndex*6)+2+48),eeprom_read((cIndex*6)+3+48),eeprom_read((cIndex*6)+4+48),eeprom_read((cIndex*6)+5+48),
eeprom_read((cIndex*6)+96),eeprom_read((cIndex*6)+1+96),eeprom_read((cIndex*6)+2+96),eeprom_read((cIndex*6)+3+96),eeprom_read((cIndex*6)+4+96),eeprom_read((cIndex*6)+5+96),
eeprom_read((cIndex*6)+144),eeprom_read((cIndex*6)+1+144),eeprom_read((cIndex*6)+2+144),eeprom_read((cIndex*6)+3+144),eeprom_read((cIndex*6)+4+144),eeprom_read((cIndex*6)+5+144),
eeprom_read((cIndex*6)+192),eeprom_read((cIndex*6)+1+192),eeprom_read((cIndex*6)+2+192),eeprom_read((cIndex*6)+3+192),eeprom_read((cIndex*6)+4+192),eeprom_read((cIndex*6)+5+192));


SerialSoftPrintln(buf);
//serialprintln(buf);
}
lcdwrite(192,CMD);
lcdprint("Done");
_delay_ms(2000);
}

void DeleteRecord()
{
lcdwrite(1,CMD);
lcdprint("Please Wait...");
for(int i=0;i<255;i++)
eeprom_write(i,10);
_delay_ms(2000);
lcdwrite(1,CMD);
lcdprint("Record Deleted");
lcdwrite(192,CMD);
lcdprint("Successfully");
_delay_ms(2000); 
} 


//-------------------------------------------------------------------------------------------

void main(void)
{
// Declare your local variables here

// Input/Output Ports initialization
// Port A initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRA=(0<<DDA7) | (0<<DDA6) | (0<<DDA5) | (0<<DDA4) | (0<<DDA3) | (0<<DDA2) | (0<<DDA1) | (0<<DDA0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTA=(0<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

// Port B initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRC=(1<<DDC7) | (1<<DDC6) | (1<<DDC5) | (1<<DDC4) | (1<<DDC3) | (1<<DDC2) | (1<<DDC1) | (1<<DDC0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRD=(0<<DDD7) | (0<<DDD6) | (0<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: Timer 0 Stopped
// Mode: Normal top=0xFF
// OC0 output: Disconnected
TCCR0=(0<<WGM00) | (0<<COM01) | (0<<COM00) | (0<<WGM01) | (0<<CS02) | (0<<CS01) | (0<<CS00);
TCNT0=0x00;
OCR0=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: Timer1 Stopped
// Mode: Normal top=0xFFFF
// OC1A output: Disconnected
// OC1B output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: Timer2 Stopped
// Mode: Normal top=0xFF
// OC2 output: Disconnected
ASSR=0<<AS2;
TCCR2=(0<<WGM20) | (0<<COM21) | (0<<COM20) | (0<<WGM21) | (0<<CS22) | (0<<CS21) | (0<<CS20);
TCNT2=0x00;
OCR2=0x00;

// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (0<<OCIE0) | (0<<TOIE0);

// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
// INT2: Off
MCUCR=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
MCUCSR=(0<<ISC2);

// USART initialization
// USART disabled
UCSRB=(0<<RXCIE) | (0<<TXCIE) | (0<<UDRIE) | (0<<RXEN) | (0<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);

// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
SFIOR=(0<<ACME);

// ADC initialization
// ADC disabled
ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);

// SPI initialization
// SPI disabled
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

// TWI initialization
// TWI disabled
TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

// Alphanumeric LCD initialization
// Connections are specified in the
// Project|Configure|C Compiler|Libraries|Alphanumeric LCD menu:
// RS - PORTC Bit 0
// RD - PORTC Bit 1
// EN - PORTC Bit 2
// D4 - PORTC Bit 4
// D5 - PORTC Bit 5
// D6 - PORTC Bit 6
// D7 - PORTC Bit 7
// Characters/line: 8
lcd_init(8);




while(1)
  { 
    RTC();

  //  if(match == LOW)
   // { 
matchFinger();
   // }

    if(enrol == LOW)
    {
buzzer(200);
        enrolFinger(); 
        _delay_ms(2000);
       // lcdinst();
    }

    else if(delet == LOW)
    {
buzzer(200);
        getId();
        deleteFinger();
        _delay_ms(1000);
    }
  } 
  return 0;
}
}
