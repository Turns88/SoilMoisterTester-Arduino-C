
//graphics headers
#include <lcd.h>
#include <macros.h>
#include <ascii_font.h>
#include <graphics.h>
#include <string.h>
//standard c headers
#include <stdio.h>
#include <stdint.h>

#include <stdlib.h>
//avr headers
#include <avr/io.h>
#include <util/delay.h>

#include <avr/io.h>
#include <avr/interrupt.h>


// Definitions reference Lecture 7 intro to microcontrontrollers
#define SET_BIT(reg, pin)           (reg) |= (1 << (pin))
#define CLEAR_BIT(reg, pin)       (reg) &= ~(1 << (pin))
#define WRITE_BIT(reg, pin, value)   (reg) = (((reg) & ~(1 << (pin))) | ((value) << (pin)))
#define BIT_VALUE(reg, pin)       (((reg) >> (pin)) & 1)
#define BIT_IS_SET(reg, pin)         (BIT_VALUE((reg),(pin))==1)
 #define DEBOUNCE_MS (100)

//uart definitions
#define BAUD (9600)
#define MYUBRR (F_CPU/16/BAUD-1)
 
// These buffers may be any size from 2 to 256 bytes.
#define  RX_BUFFER_SIZE  64
#define  TX_BUFFER_SIZE  64
 
 
//uart definitions from Topic 8 Lecture Notes
unsigned char rx_buf;
 
static volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
static volatile uint8_t tx_buffer_head;
static volatile uint8_t tx_buffer_tail;
static volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile uint8_t rx_buffer_head;
static volatile uint8_t rx_buffer_tail;
 
 
//Functions declaration
void setup(void);
void process(void);
void uart_init(unsigned int ubrr);
//uart functions
void uart_putchar(uint8_t c);
uint8_t uart_getchar(void);
uint8_t uart_available(void);
void uart_putstring(unsigned char* s);
void uart_getLine(unsigned char* buf, uint8_t n);
char emptyString[50];

//plant bitmap unused
const unsigned char plant[] = {
0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xf1, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xff, 
0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 
0xff, 0x80, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 
0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 
0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 
0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 
0xff, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xc3, 0x03, 0xff, 
0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xc1, 0x87, 0x3f, 0xff, 0xff, 0xff, 0xf0, 0xff, 
0xff, 0xff, 0xff, 0xc0, 0xbc, 0x0f, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x7c, 
0x1f, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x74, 0x3f, 0xff, 0xff, 0xff, 0xf0, 
0xff, 0xff, 0xff, 0xff, 0xf8, 0x6f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 
0x5f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xbf, 0xff, 0xff, 0xff, 0xff, 
0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 
0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 
0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 
0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 
0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 
0xff, 0xff, 0xf9, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x09, 0xff, 
0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xf0, 0xff, 
0xff, 0xff, 0xff, 0xff, 0x80, 0x3f, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 
0x3f, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xf0, 
0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x80, 
0x00, 0x7f, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x7f, 0xff, 0xff, 0xff, 
0xf0, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 
0x80, 0x00, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0xff, 0xff, 0xff, 
0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x01, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 
0xff, 0xc0, 0x01, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x01, 0xff, 0xff, 
0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x01, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x03, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0xff, 
0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xf0, 0xff, 
0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 
0x11, 0x7f, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xf0
};
/**********************************************/
void alarm(void){
	//SET_BIT(DDRB, 2);
	//SET_BIT(PORTB, 2);
	DDRB |= (1<<PB1);
   
    // PD6 is now an output

    OCR1A = 128;
    // set PWM for 50% duty cycle

    TCCR1A |= (1 << COM0A1);
    // set none-inverting mode

   
    TCCR1B = 2;
    
    TCCR1A |= (1 << WGM02) | (1 << WGM01) | (1 << WGM00);
    // set fast PWM Mode


    // while (1)
    // {
    //     TCCR1B = 2;
    //     _delay_ms(2000);
    //     TCCR1B = 3;
    //     _delay_ms(2000);
    //     TCCR1B = 4;
    //     _delay_ms(2000);
    //             TCCR1B = 5;
    //                     _delay_ms(2000);
    //             TCCR1B = 6;
    //                     _delay_ms(2000);
    //             TCCR1B = 7;
    //     _delay_ms(2000);
    //     TCCR1B = 3;
    //     _delay_ms(2000);
    //     TCCR1B = 2;
    //     _delay_ms(2000);
    //  }
}
void alarmOff(void) {
            CLEAR_BIT(PINB,4);
            CLEAR_BIT(DDRB,4);
        DDRB |= (1>>PB1);
   
    // PD6 is now an output

    OCR1A = 128;
    // set PWM for 50% duty cycle

    TCCR1A |= (1 >> COM0A1);
    // set none-inverting mode
   
    TCCR1B = 0;

    TCCR1A |= (1 >> WGM02) | (1 >> WGM01) | (1 >> WGM00);
}
/**********************************************/
void action(void) {

if (BIT_IS_SET(PINB, 4)){
            _delay_ms(DEBOUNCE_MS);
            //SET_BIT(PORTB, 0);
            //SET_BIT(PORTB, 2);
            process();
            //alarm();
	    }
else if (BIT_VALUE(PINB, 4) == 0){
            CLEAR_BIT(PINB,4);
            CLEAR_BIT(DDRB,4);
            CLEAR_BIT(PORTB, 0);
            CLEAR_BIT(PORTB, 2);
            alarmOff();
        } 
}
/**********************************************/




void setup(void) {
    SET_BIT(DDRB, 0);
    SET_BIT(DDRB, 2);
    CLEAR_BIT(DDRB, 4);

      char welcome1[15];
    char welcome2[20];
    char welcome3[20];
    strcpy(welcome1,"Paul's Plant");
    strcpy(welcome2,"Moisture Sensor!");
    strcpy(welcome3,"Press to start");  
  //initialise screen with low contrast. see lcd.h for options
  lcd_init(LCD_LOW_CONTRAST);
  //zero screen buffer (memory)
  clear_screen();
  //write buffer to LCD
  strcpy(emptyString, "********************************");
    draw_string(0,0,emptyString, BG_COLOUR);
    draw_string(0, 8, welcome1, FG_COLOUR);
    draw_string(0, 16, welcome2, FG_COLOUR);
    draw_string(0, 24, welcome3, FG_COLOUR);
    draw_string(0,32,emptyString, BG_COLOUR);


  show_screen();

//initialises ADC and UART port

 
    //init uart
    uart_init(MYUBRR);
 
    // Enable  LED on B5
    SET_BIT(DDRB, 5);
    //SET_BIT(DDRB, 2);
  	 //SET_BIT(PINB, 2);
 
    // initialise adc
      // ADC Enable and pre-scaler of 128: ref table 24-5 in datasheet
    // ADEN  = 1
    // ADPS2 = 1, ADPS1 = 1, ADPS0 = 1
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
 
   // select channel and ref input voltage
   // channel 0, PC0 (A0 on the uno)
   // MUX0=0, MUX1=0, MUX2=0, MUX3=0
   // REFS0=1
   // REFS1=0
    ADMUX = (1 << REFS0);
 
}
 
 
void process(void) {
 
   char temp_buf[64];
 
 
   // Start single conversion by setting ADSC bit in ADCSRA
    ADCSRA |= (1 << ADSC);
 
    // Wait for ADSC bit to clear, signalling conversion complete.
     while ( ADCSRA & (1 << ADSC) ) {}

    //Convert to degrees
  //Voltage at pin in milliVolts = (reading from ADC) * (5000/1024) 
//ADC = ADC  *(5000/1024);
  //Centigrade temperature = [(analog voltage in mV) - 500] / 10
//ADC = (ADC  -500) / 10;
 
    // Result now available in ADC
     uint16_t pot = ADC;
 
 
    // convert uint16_t to string
    itoa(pot, (char *) temp_buf,10);  
    
    char message[50];

     //when converted value is above a threshold, perform an action
     if (pot < 400){
       alarm();
       strcpy(message, "Water me please!");
          // Timer 0 in normal mode, with pre-scaler 1024 ==> ~60Hz overflow.
 
       SET_BIT(PORTB, 0);
     }

     else {
            alarmOff();
            strcpy(message, "Moist! Noice Job!");
            SET_BIT(PORTB, 2);
     }

 
 
   //send serial data
   uart_putstring((unsigned char *) temp_buf);
   uart_putchar('\n');
 

  clear_screen();
  //draw string using foreground or background options
  //Reference Topic 11 LCD notes
  char emptyString[50];
  strcpy(emptyString, "********************************");
  draw_string(0,8,emptyString, BG_COLOUR);
  draw_string(0, 24, temp_buf, FG_COLOUR);
  draw_string(0, 16, message, FG_COLOUR);
  draw_string(0,32,emptyString, BG_COLOUR);

  show_screen();

}

int main (void) {
  setup();

  for ( ;; ) {
   
    
   //_delay_ms(5000);
    action();
  }
}

 
// Initialize the UART Reference Topic 8 Serial Communications lecture notes
void uart_init(unsigned int ubrr) {
 
    cli();
 
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)(ubrr);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    tx_buffer_head = tx_buffer_tail = 0;
    rx_buffer_head = rx_buffer_tail = 0;
 
    sei();
 
}
 
 
 
// Transmit a byte
void uart_putchar(uint8_t c) {
    uint8_t i;
 
    i = tx_buffer_head + 1;
    if ( i >= TX_BUFFER_SIZE ) i = 0;
    while ( tx_buffer_tail == i ); // wait until space in buffer
    //cli();
    tx_buffer[i] = c;
    tx_buffer_head = i;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << UDRIE0);
    //sei();
}
 
// Receive a byte
uint8_t uart_getchar(void) {
    uint8_t c, i;
 
    while ( rx_buffer_head == rx_buffer_tail ); // wait for character
    i = rx_buffer_tail + 1;
    if ( i >= RX_BUFFER_SIZE ) i = 0;
    c = rx_buffer[i];
    rx_buffer_tail = i;
    return c;
}
 
 
// Transmit a string
void uart_putstring(unsigned char* s)
{
    // transmit character until NULL is reached
    while(*s > 0) uart_putchar(*s++);
}
 
 
// Receive a string
void uart_getLine(unsigned char* buf, uint8_t n)
{
    uint8_t bufIdx = 0;
    unsigned char c;

    do
    {
        // receive character
        c = uart_getchar();
 
        // store character in buffer
        buf[bufIdx++] = c;
    }
    while((bufIdx < n) && (c != '\n'));
 
    // ensure buffer is null terminated
    buf[bufIdx] = 0;
}
 
 
 
uint8_t uart_available(void) {
    uint8_t head, tail;
 
    head = rx_buffer_head;
    tail = rx_buffer_tail;
    if ( head >= tail ) return head - tail;
    return RX_BUFFER_SIZE + head - tail;
}
 
 
// Transmit Interrupt
ISR(USART_UDRE_vect) {
    uint8_t i;
 
    if ( tx_buffer_head == tx_buffer_tail ) {
        // buffer is empty, disable transmit interrupt
        UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    }
    else {
        i = tx_buffer_tail + 1;
        if ( i >= TX_BUFFER_SIZE ) i = 0;
        UDR0 = tx_buffer[i];
        tx_buffer_tail = i;
    }
}
 
// Receive Interrupt
ISR(USART_RX_vect) {
    uint8_t c, i;
 
    c = UDR0;
    i = rx_buffer_head + 1;
    if ( i >= RX_BUFFER_SIZE ) i = 0;
    if ( i != rx_buffer_tail ) {
        rx_buffer[i] = c;
        rx_buffer_head = i;
    }
}

