/*
 * TRON3.c
 *
 * Created: 8/15/2021 8:39:40 PM
 * Author : keith
 */ 

/* Effects
power up
touch
sequence
motorcycle
*/

/*Sections
Each side

1, hand
2, forearm
3, upper arm
4, shoulder

5, Thigh
6, calf
7, Foot

8, Abs
9, Chest
10, eyes
11, ears
12, hair / beard


*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

//#include "common.h"
#include "circularbuffer.h"

#define SFRPTR(port) (volatile uint8_t*)& port

#define SET_OFF(PORT, PIN) (*PORT &= ~(1 << PIN))
#define SET_ON(PORT, PIN) (*PORT |= 1 << PIN)

#define BEAT_TIME 480

typedef struct
{
    volatile uint8_t *port;
    uint8_t bit;
}EL_Out_t;

volatile  EL_Out_t ELPins[] = {
   
        {SFRPTR(PORTB), PORTB4},        //EL 0
        {SFRPTR(PORTB), PORTB3},        //EL 1
        {SFRPTR(PORTB), PORTB2},        //EL 2
        {SFRPTR(PORTB), PORTB1},        //EL 3
        {SFRPTR(PORTB), PORTB0},        //EL 4
            
        {SFRPTR(PORTA), PORTA0},        //EL 5 8vbbS9TQ9u
        {SFRPTR(PORTA), PORTA1},        //EL 6
        {SFRPTR(PORTA), PORTA2},        //EL 7
        {SFRPTR(PORTA), PORTA3},        //EL 8
        {SFRPTR(PORTA), PORTA4},        //EL 9
        {SFRPTR(PORTA), PORTA5},        //EL 10
        {SFRPTR(PORTA), PORTA6},        //EL 11
        {SFRPTR(PORTA), PORTA7},        //EL 12
            
        {SFRPTR(PORTC), PORTC1},        //EL 13
        {SFRPTR(PORTC), PORTC0},        //EL 14
            
        {SFRPTR(PORTD), PORTD7},        //EL 15
        {SFRPTR(PORTD), PORTD6},        //EL 16
        {SFRPTR(PORTD), PORTD5},        //EL 17
        {SFRPTR(PORTD), PORTD4},        //EL 18
        {SFRPTR(PORTD), PORTD3},        //EL 19
        {SFRPTR(PORTD), PORTD2},        //EL 20
            
        {SFRPTR(PORTB), PORTB7},        //EL 21
        {SFRPTR(PORTB), PORTB6},        //EL 22
        {SFRPTR(PORTB), PORTB5},        //EL 23
    
};

volatile char mcusr __attribute__ ((section (".noinit")));
volatile uint32_t	TimerTimerTic =0; //1.024 ms per tic

/*Tron is 125 bpm or 2.09 beats per second
* 125 bpm * 2.10 minutes == 262.5 beats
*
* 1 beat is 1/2.09 beats per second 478.468899 ms per beat
* @ 1.024 ms per tic there are 467.25 timer tics per beat
*
*262 beats * 467 tics per beat =215,871 tics, well within a uint32
*
*tic error is 262 beats * 467.25 = 122.419 vs 122.354 == 65 tics or 1/2 beat
*

126bpm /60 seconds =2.1 bps
1/2.1 = 476.19 ms per beat
476 / 1.024 ms per tic is 465 tics per beat
*/

//**** Timer *********************************************************
#define PERIOD 584.5357

static inline uint32_t GetTime(void)
{
    cli(); 
    uint32_t Time = TimerTimerTic;
    sei();
    
    return Time;
}

//returns TRUE when time expires
bool IsTimedOut(uint32_t period, uint32_t startTime)
{
    bool to;
    //detects counter rollover due to the magic of fixed point math
    cli();
    to =  (TimerTimerTic  - startTime) > period ? true : false;
    sei();
    
    return to;
}

//return after ms milliseconds
void Delay(uint32_t ms)
{
    uint32_t Time = GetTime();
    
    while(!IsTimedOut(ms, Time));
}

// clock interrupt - clear flag immediately to resume count
ISR(TIMER0_COMPA_vect)
{
    TimerTimerTic++;     // 1.0 ms, Clock tic
}

static void TimerInit()
{
    // Timer/Counter 0 initialization
    // Clock source: System Clock
    // Clock value: 250.000 kHz
    // Mode: CTC top=OCR0A
    // OC0A output: Disconnected
    // OC0B output: Disconnected
    // Timer Period: 1 ms
    TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (0<<WGM00);
    TCCR0B=(0<<WGM02) | (0<<CS02) | (1<<CS01) | (1<<CS00);
    TCNT0=0x00;
    OCR0A=0xF9;
    OCR0B=0x00;
    
    //enable interrupt on compare
    TIMSK0 = _BV(OCIE0A);
    
    TimerTimerTic = 0;
}

//IO *****************************************************************
//setup IO lines
//Read using  PINx
//Write using PORTx
void IOSetup()
{
    //port directions
    DDRA = 0xff;
    DDRB = 0xff;
    DDRC = 0x03;
    DDRD = 0xFC;
    
    //pullups
    //PORTA = 0;
    //PORTB = 0;
    PORTC = 0x80;
    //PORTD = 0xF8;
}    
//* Serial ***********************************************************
ISR(USART0_UDRE_vect)
{
    uint8_t c;

    if(true == TxBuf_Get(&c))
    {
        UDR0 = c;
    }
    else
    {
        //disable tx data empty interrupt
        UCSR0B &= ~_BV(TXCIE0);
    }
}

void SerialInit()
{
    //Set baudrate and enabling interfaces
    //115200 8n1, atmega164 @ 16mhz specific
    //IsOpen = false;
    
    UBRR0H = 0;
    UBRR0L = 16;  //115200 baud
    //UBRR0L = 207; //9600 baud for Bluetooth
    UCSR0A |= _BV(U2X0);

    UCSR0C = (3 << UCSZ00); //8n1
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   // Enable RX and TX
    
    //IsOpen = true;
    sei();
}

void SerialPutBuf(uint8_t *data, uint16_t length)
{
    for( int i=0; i<(int)length;i++)
    {
        TxBuf_Put(data[i]);
        //put16(data[i]);
    }
}

//Put text on UART
void SerialPut(char* text)
{
    int i=0;
    while(text[i])
    {
        //drop data if buffer overflows
        TxBuf_Put(text[i]);
        i++;
    }
    
    //make sure the interrupt is enabled
    UCSR0B |= _BV(UDRIE0);
}

void SerialPut32(uint32_t number)
{
    char buf[32] = {0};
    register int i = 30;
    int j=0;
                
    if(number != 0)
    {

        for(; number && i ; --i, number /= 10)
        {
            j = number % 10;
            buf[i] = "0123456789abcdef"[j]; //number % 16];
        }
        
         SerialPut(&buf[i+1]);
    }
    else
    {
        TxBuf_Put('0');
    }
}

static void AllOn()
{
    PORTA =0xff;
    PORTB =0xff;
    PORTC =0xff;
    PORTD =0xff;
}

static void AllOff()
{
    PORTA =0;
    PORTB =0;
    PORTC =0;
    PORTD =0;
}

#define MC_SEGMENTS    23   /* how many segments to sequence through */
#define MC_WIDTH       3    /* how many segments to have on at one time*/
#define MC_CYCLES      3    /* how many times to cycle through the all the segments*/

static void motorcycle(void)
{
    uint32_t delay = 50;
    //int step = 0;
    //static bool on = true;
    bool done = false;
    int i=0;
    int j =0;
    
    SerialPut( "Motorcycle\n");
      
    while(!done)
    {
        //10 elements to cycle
        //cycle for ?? measures
            
        Delay(delay);

        //SerialPut( "M# "); SerialPut32(j); SerialPut( ",");SerialPut32(i); SerialPut( ",");SerialPut32(ELPins[i].bit); SerialPut( "\n");
            
        SET_ON(ELPins[i].port, ELPins[i].bit );
            
        if(i>= MC_WIDTH)
        {
            SET_OFF(ELPins[i- MC_WIDTH].port, ELPins[i- MC_WIDTH].bit );
        }
        else
        {
            SET_OFF(ELPins[(MC_SEGMENTS - MC_WIDTH)+i].port, ELPins[(MC_SEGMENTS- MC_WIDTH) +i].bit );
        }
            
        if(i++ > MC_SEGMENTS)
        {
            i=0;
            if(j++ >= MC_CYCLES)
            {
                done = true;
            }
        }
    }        
                        
}

//All on
//down then up
void metropolisGrace(void)
{
    int i=0;
    int j=0;
    uint32_t delay = BEAT_TIME;// / count;
    
    SerialPut( "metropolis Grace");

    Delay(delay);
    SET_OFF(ELPins[i].port, ELPins[i].bit );
    SET_OFF(ELPins[j].port, ELPins[j].bit );
    i++;j++;
    
    Delay(delay);
    SET_OFF(ELPins[i].port, ELPins[i].bit );
    SET_OFF(ELPins[j].port, ELPins[j].bit );
    i++;j++;
    
    Delay(delay);
    SET_OFF(ELPins[i].port, ELPins[i].bit );
    SET_OFF(ELPins[j].port, ELPins[j].bit );
    i++;j++;
    
    Delay(delay);
    SET_OFF(ELPins[i].port, ELPins[i].bit );
    SET_OFF(ELPins[j].port, ELPins[j].bit );
    i++;j++;
    
    Delay(delay);
    SET_OFF(ELPins[i].port, ELPins[i].bit );
    SET_OFF(ELPins[j].port, ELPins[j].bit );
    i++;j++;
    
    Delay(delay);
    SET_OFF(ELPins[i].port, ELPins[i].bit );
    SET_OFF(ELPins[j].port, ELPins[j].bit );
    i++;j++;
    
    Delay(delay);
    SET_ON(ELPins[i].port, ELPins[i].bit );
    SET_ON(ELPins[j].port, ELPins[j].bit );
    i--;j--;
    
    Delay(delay);
    SET_ON(ELPins[i].port, ELPins[i].bit );
    SET_ON(ELPins[j].port, ELPins[j].bit );
    i--;j--;
    
    Delay(delay);
    SET_ON(ELPins[i].port, ELPins[i].bit );
    SET_ON(ELPins[j].port, ELPins[j].bit );
    i--;j--;
    
    Delay(delay);
    SET_ON(ELPins[i].port, ELPins[i].bit );
    SET_ON(ELPins[j].port, ELPins[j].bit );
    i--;j--;
    
    Delay(delay);
    SET_ON(ELPins[i].port, ELPins[i].bit );
    SET_ON(ELPins[j].port, ELPins[j].bit );
    i--;j--;
    
    Delay(delay);
    SET_ON(ELPins[i].port, ELPins[i].bit );
    SET_ON(ELPins[j].port, ELPins[j].bit );
    i--;j--;

}


#define KEITH_SEGMENTS    23   /* how many segments to sequence through */
#define KEITH_WIDTH       3    /* how many segments to have on at one time*/
#define KEITH_CYCLES      3    /* how many times to cycle through the all the segments*/

//all off but arms
//rotate arms
void MetropolisKeith(void)
{
    uint32_t delay = 50;
    //int step = 0;
    //static bool on = true;
    bool done = false;
    int i=0;
    int j =0;
    
    SerialPut( "Metropolis Keith\n");
      
    while(!done)
    {
        //10 elements to cycle
        //cycle for ?? measures
            
        Delay(delay);

        //SerialPut( "M# "); SerialPut32(j); SerialPut( ",");SerialPut32(i); SerialPut( ",");SerialPut32(ELPins[i].bit); SerialPut( "\n");
            
        SET_ON(ELPins[i].port, ELPins[i].bit );
            
        if(i>= KEITH_WIDTH)
        {
            SET_OFF(ELPins[i- KEITH_WIDTH].port, ELPins[i- KEITH_WIDTH].bit );
        }
        else
        {
            SET_OFF(ELPins[(KEITH_SEGMENTS - KEITH_WIDTH)+i].port, ELPins[(KEITH_SEGMENTS- KEITH_WIDTH) +i].bit );
        }
            
        if(i++ > KEITH_SEGMENTS)
        {
            i=0;
            if(j++ >= KEITH_CYCLES)
            {
                done = true;
            }
        }
    }     
    
}

//scan from fingertip to fingertip
static void Scan(void)
{
    uint32_t delay = 50;
    int step = 0;
    static bool on = true;
    bool done = false;
    
    while(!done)
    {
        Delay(delay);
       
        //SerialPut( "EL B# "); SerialPut32(step); SerialPut( ",");SerialPut32(ELPins[step].bit); SerialPut( "\n");
        if(on)
        {
            if(step < 24)
            {
                SET_ON(ELPins[step].port, ELPins[step].bit );
                step++;
            }
            else
            {
                SerialPut( "set\n");
                on = !on;
            }
        } 
    
        if(!on)
        {
            if(step > 0)
            {
                SET_OFF(ELPins[step].port, ELPins[step].bit );
                step--;
            }
            else
            {
                SerialPut( "Reset\n");
                on = !on;
                done = true;
            }        
        }   
       
    }    
}


static void PowerUp(char count)
{
    //uint32_t Time ;
    int i=0;
    int j=12;
    uint32_t delay = BEAT_TIME / count;
    //bool done = false;
    
    //Time = GetTime();
    
    //Feet on
    Delay(delay);
    SET_ON(ELPins[i].port, ELPins[i].bit );
    SET_ON(ELPins[j].port, ELPins[j].bit );
    i++, j++;
        
    //calves on
    Delay(delay);
    SET_ON(ELPins[i].port, ELPins[i].bit );
    SET_ON(ELPins[j].port, ELPins[j].bit );
    i++, j++;
        
    //thighs on
    Delay(delay);
    SET_ON(ELPins[i].port, ELPins[i].bit );
    SET_ON(ELPins[j].port, ELPins[j].bit );
    i++, j++;
        
    if(count == 3) 
    {
        AllOff();
        return;
    }        
    
    //hips on
    Delay(delay);
    SET_ON(ELPins[i].port, ELPins[i].bit );
    SET_ON(ELPins[j].port, ELPins[j].bit );
    i++, j++;
    
    //abs on
    Delay(delay);
    SET_ON(ELPins[i].port, ELPins[i].bit );
    SET_ON(ELPins[j].port, ELPins[j].bit );
    i++, j++;
    
    //Chest on
    Delay(delay);
    SET_ON(ELPins[i].port, ELPins[i].bit );
    SET_ON(ELPins[j].port, ELPins[j].bit );
    i++, j++;
    
    //Shoulders on
    Delay(delay);
    SET_ON(ELPins[i].port, ELPins[i].bit );
    SET_ON(ELPins[j].port, ELPins[j].bit );
    i++, j++;
    
    //arms on
    Delay(delay);
    SET_ON(ELPins[i].port, ELPins[i].bit );
    SET_ON(ELPins[j].port, ELPins[j].bit );
    i++, j++;
    
    if(count == 8) 
    {
        AllOff();
        return;
    }        
    
    //Shoulders on
    Delay(delay);
    SET_ON(ELPins[i].port, ELPins[i].bit );
    SET_ON(ELPins[j].port, ELPins[j].bit );
    i++, j++;
    
    //arms on
    Delay(delay);
    SET_ON(ELPins[i].port, ELPins[i].bit );
    SET_ON(ELPins[j].port, ELPins[j].bit );
    i++, j++;
    
    //Shoulders on
    Delay(delay);
    SET_ON(ELPins[i].port, ELPins[i].bit );
    SET_ON(ELPins[j].port, ELPins[j].bit );
    i++, j++;
    
    //arms on
    Delay(delay);
    SET_ON(ELPins[i].port, ELPins[i].bit );
    SET_ON(ELPins[j].port, ELPins[j].bit );
    i++, j++;
    
    AllOff();
        
}

bool ButtonPressed(void)
{
    bool pressed = false;

    if( (PINC & 0x80) == 0)
    {
        pressed = true;
        SerialPut("pressed\n");
    }
    
    return pressed;
}

int main(void)
{
    static uint32_t Time;
    int beat = 1;
    int measure = 0;  

    IOSetup();
    SerialInit();
    SerialPut( "\n\nTRON II __DATE__ \nCopyright 2021 Keith Vasilakes\n\n");
    SerialPut32(12345678); SerialPut( "\n");
    SerialPut32(0xABCDEF12); SerialPut( "\n");

    
    TimerInit();
    
    Time = GetTime();
    
    while(ButtonPressed() == false);
    
    SerialPut( "** Power up\n");
    PowerUp(3);
    PowerUp(8);
    PowerUp(11);                 

    /* Replace with your application code */
    while (1) 
    {
        if(IsTimedOut(BEAT_TIME, Time))
        {
            Time = GetTime();

            if(beat %8 == 0) measure++;
            
            beat++;
            if(beat >8) beat =1;
            
            switch(measure)
            {
                case 0: //power up
                SerialPut("** case 0\n");
                    
                measure++;              
                break;
                
                case 1://awareness
                    SerialPut("** case 1\n");
                    AllOff();
                    measure++;
                    break;
                case 2:
                    SerialPut("** case 2\n");
                    AllOn();
                    measure++;
                    break;
                case 3:
                    SerialPut("** case 3\n");
                    metropolisGrace();
                    measure++;
                    break;
                case 4:
                    SerialPut("** case 4\n");
                    motorcycle();
                    measure++;
                    break;
                case 5:
                    SerialPut("** case 5\n");
                    AllOff();
                    measure++;
                    break;
                case 6:
                     SerialPut("** case 6\n");
                    Scan();
                    measure++;
                    break;
                case 7:
                    SerialPut("** case 5\n");
                    AllOn();
                    measure++;
                    break;
                case 8:
                    SerialPut("** case 5\n");
                    AllOff();
                    measure++;
                    break;
                case 9:
                    SerialPut("** case 5\n");
                    MetropolisKeith();
                    measure++;
                    break;
            }
            
        }    
    }//while
}


