
#include <avr/io.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include <stdlib.h>


#define DEFAULT_NUM_TICKS_UNTIL_DISABLED (1 /*minutes*/ * 60 /*seconds*/ * 2 /*seconds per tick*/)
volatile int g_numTicksUntilDisabled = 0;

#define A PB0  //PB0
#define B PB1  //PB1
#define C PB3  //PB3
#define D PB4  //PB4

#define PIN_CONFIG 0
#define PIN_STATE 1

#define HIGH 1
#define LOW 0

#define OUTPUT 1
#define INPUT 0

#define LED_COUNT 12

int matrix[LED_COUNT][2][4] = {
  //           PIN_CONFIG                  PIN_STATE
  //    A       B       C      D         A     B    C    D
  { { OUTPUT, OUTPUT, INPUT, INPUT }, { HIGH, LOW, LOW, LOW } }, // AB 0
  { { OUTPUT, OUTPUT, INPUT, INPUT }, { LOW, HIGH, LOW, LOW } }, // BA 1
  { { INPUT, OUTPUT, OUTPUT, INPUT }, { LOW, HIGH, LOW, LOW } }, // BC 2
  { { INPUT, OUTPUT, OUTPUT, INPUT }, { LOW, LOW, HIGH, LOW } }, // CB 3
  { { OUTPUT, INPUT, OUTPUT, INPUT }, { HIGH, LOW, LOW, LOW } }, // AC 4
  { { OUTPUT, INPUT, OUTPUT, INPUT }, { LOW, LOW, HIGH, LOW } }, // CA 5
  { { OUTPUT, INPUT, INPUT, OUTPUT }, { HIGH, LOW, LOW, LOW } }, // AD 6
  { { OUTPUT, INPUT, INPUT, OUTPUT }, { LOW, LOW, LOW, HIGH } }, // DA 7
  { { INPUT, OUTPUT, INPUT, OUTPUT }, { LOW, HIGH, LOW, LOW } }, // BD 8
  { { INPUT, OUTPUT, INPUT, OUTPUT }, { LOW, LOW, LOW, HIGH } }, // DB 9
  { { INPUT, INPUT, OUTPUT, OUTPUT }, { LOW, LOW, HIGH, LOW } }, // CD 10
  { { INPUT, INPUT, OUTPUT, OUTPUT }, { LOW, LOW, LOW, HIGH } }  // DC 11
};

void pinMode( int pin, int direction ) {
    if (direction == OUTPUT) {
	    DDRB |= _BV(pin);
    } else {
	    DDRB &= ~_BV(pin);
    }
}

void digitalWrite( int pin, int state ) {
    if (state == HIGH) {
	    PORTB |= _BV(pin);
    } else {
	    PORTB &= ~_BV(pin);
    }
}

void turnOn( int led ) {
  pinMode( A, matrix[led][PIN_CONFIG][0] );
  pinMode( B, matrix[led][PIN_CONFIG][1] );
  pinMode( C, matrix[led][PIN_CONFIG][2] );
  pinMode( D, matrix[led][PIN_CONFIG][3] );
  digitalWrite( A, matrix[led][PIN_STATE][0] );
  digitalWrite( B, matrix[led][PIN_STATE][1] );
  digitalWrite( C, matrix[led][PIN_STATE][2] );
  digitalWrite( D, matrix[led][PIN_STATE][3] );
}

void sleep_until_interrupt()
{
    pinMode(A, INPUT);
    pinMode(B, INPUT);
    pinMode(C, INPUT);
    pinMode(D, INPUT);

	MCUCR |= _BV(SE);   //sleep enable bit
	GIMSK |= _BV(PCIE); // Enable pin-change interrupts to wake us up
	sleep_mode();
	GIMSK &= ~_BV(PCIE); // Disable pin-change interrupts
	MCUCR &= ~_BV(SE);   // clear sleep enable bit
}

void blink_led(uint8_t numTimes, bool fastDelay = false)
{
	PORTB &= ~_BV(0);
	for (uint8_t i = 0; i < numTimes; ++i)
	{
		if (fastDelay)
			_delay_ms(50);
		else
			_delay_ms(200);
		PORTB |= _BV(0); //on
		if (fastDelay)
			_delay_ms(50);
		else
			_delay_ms(200);
		PORTB &= ~_BV(0); //off
	}
}

int main(void)
{
	// Disable ADC; we don't need it, and it uses more power
	ACSR |= _BV(ACD); // analog comparator disable

	// Set up timer1 to trigger every 1/2 second
	TCCR1 |= _BV(CTC1);										// enable timer compare match CTC1
	TCCR1 |= _BV(CS13) | _BV(CS12) | _BV(CS11) | _BV(CS10); // set prescaler ck/16384 (1/(8Mhz/16384) = 2.048 ms)
	OCR1C = 244;											// count to 1/2 seconds (2.048ms * 244 = .5s)
	TIMSK |= _BV(OCIE1A);									// enable compare match irq

	// Set interrupt on PB2
	DDRB &= ~_BV(DDB2);					// PB2 set as input
	GIMSK |= _BV(INT0);					// enable PCIE interrupt
	PCMSK |= _BV(PCINT2);				// interrupt on PB2
	MCUCR |= (_BV(ISC01) | _BV(ISC00)); // enable rising edge (ISC01 and ISC00)
	MCUCR |= _BV(SM1);					// enable power-down for sleep (SM1)
	sei();								// enable interrupts

	sleep_until_interrupt();

	int lastLed = -1;
	int led = -1;
	while (1)
	{
		if (g_numTicksUntilDisabled <= 0)
        {
            sleep_until_interrupt();
        }

		led = rand() % LED_COUNT;
		do 
		{
			led = rand() % LED_COUNT;
		} while (led == lastLed);
        turnOn(led);
        _delay_ms(1000/LED_COUNT);

		lastLed = led;
    }
}

ISR(PCINT0_vect)
{
	// do nothing
}

ISR(INT0_vect)
{
	g_numTicksUntilDisabled = DEFAULT_NUM_TICKS_UNTIL_DISABLED;
}

ISR(TIMER1_COMPA_vect)
{
	g_numTicksUntilDisabled--;
}
