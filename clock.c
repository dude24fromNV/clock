#ifndef F_CPU
#define F_CPU   16000000UL
#endif

#include "segm.h"
#include "buttons.h"
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include </usr/lib/gcc/avr/5.4.0/include/stdbool.h>


/** Timer2 Interrupt (on overflow), see datasheet
 * For vectors, refer to <avr/iom328p.h>
 * For more on interrupts handling with AVR-GCC see
 * https://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html
 */
ISR(TIMER2_OVF_vect, ISR_BLOCK)
{
	TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20)); /* stop timer */
	/* It's often required to manually reset interrupt flag */
        /* to avoid infinite processing of it.                  */
        /* not on AVRs (unless OCB bit set)                     */
        /* 	TIFR2 &= ~TOV2;                                 */
}


void sleep_ms(uint16_t ms_val)
{
	/* Set Power-Save sleep mode */
	/* https://www.nongnu.org/avr-libc/user-manual/group__avr__sleep.html */
	set_sleep_mode(SLEEP_MODE_IDLE);
	cli();		/* Disable interrupts -- as memory barrier */
	sleep_enable();	/* Set SE (sleep enable bit) */
	sei();  	/* Enable interrupts. We want to wake up, don't we? */
	TIMSK2 |= (1 << TOIE2); /* Enable Timer2 Overflow interrupt by mask */
	while (ms_val--) {
		/* Count 1 ms from TCNT2 to 0xFF (up direction) */
		TCNT2 = (uint8_t)(0xFF - (F_CPU / 128) / 1000);

		/* Enable Timer2 */
		TCCR2B =  (1 << CS22) | (1 << CS20); /* f = Fclk_io / 128, start timer */

		sleep_cpu();	/* Put MCU to sleep */

		/* This is executed after wakeup */

	}
	sleep_disable();	/* Disable sleeps for safety */		
}


static struct segm_Port PB = {
	.DDR = &DDRB,
	.PIN = &PINB,
	.PORT = &PORTB,
};

static struct button_Port PD = {
        .DDR = &DDRD,
        .PORT = &PORTD,
        .PIN = &PIND,
};

static struct button_int button ={
        .set_h = {.port = &PD, .pin = 2},
        .set_m = {.port = &PD, .pin = 3},
};

static struct alarm_b alarm = {
        
        .alarm = {.port = &PD, .pin = 4},
        .alarm_on = {.port = &PD, .pin = 5},
};

static struct button_Pin bip = { .port= &PD, .pin = 6};

static struct segm_Display display = {
	.SHCP = {.port = &PB, .pin = 0},
	.STCP = {.port = &PB, .pin = 1},
	.DS   = {.port = &PB, .pin = 2},
	.delay_func = &_delay_loop_1,	/* 3 cycles / loop, busy wait */
	.sleep_ms_func = &sleep_ms,	/* 3 cycles / loop, busy wait */
	.is_comm_anode = false		/* We have common cathode display */
};

void init_timer1(){
        OCR1A = 0x3D08;
        TCCR1B |= (1 << WGM12);
        TIMSK1 |= (1 << OCIE1A);
        TCCR1B |= (1 << CS12) | (1 << CS10);
}

void init_bip(){
        TCCR0A |= (1 << WGM01);
        TCNT0 = 0x00;
        TIMSK0 |= (1 << OCIE0A);
        OCR0A = 0xFF;     
}
 

volatile uint8_t sec = 0;
volatile uint8_t min = 0;
volatile uint8_t hours = 0;
volatile uint8_t alarm_min = 0;
volatile uint8_t alarm_hours = 0;
volatile bool set_alarm = false;
volatile bool on_alarm = false;
volatile uint8_t i = 0;
int main(void)
{
	segm_init(&display);
        init_buttons(&button);
        init_alarm_button(&alarm);
        init_timer1();
        init_bip();

        uint8_t min_bcd[]={0,0};
        uint8_t hours_bcd[]={0,0};
        uint8_t alarm_min_bcd[]={0,0};
        uint8_t alarm_hours_bcd[]={0,0};
        
        uint8_t set[] = {0x6D,0x79,0x78,0x77}; //SetA
        uint8_t on_s[] = {0x3F, 0x54, 0x6D, 0}; //onS
        uint8_t off_a[] = {0x3F,0x71,0x71,0x77}; //offA
        uint8_t on_a[] = {0x3F, 0x54, 0x77, 0}; //onA

        uint8_t *pm = min_bcd;
        uint8_t *ph = hours_bcd; 
        uint8_t *pam = alarm_min_bcd;
        uint8_t *pah = alarm_hours_bcd;
        
        *(bip.port->DDR) |= 1 << bip.pin;
        *(bip.port->PORT) &= ~(1 << bip.pin);
        sei();
	while(1) {
                if (sec > 59) {
                        min++;
                        sec =0;
                }

                if (set_alarm == false){
                        if(min > 59){
                                hours++;
                                min = 0;
                                pm[1]=0;
                        }

                        if(hours >23){
                                hours = 0;
                                ph[1]=0;
                        }       
                                               
                        segm_bcd(min, pm);
                        segm_bcd(hours, ph);
                        uint8_t symbols[] = {segm_sym_table[ph[1]],segm_sym_table[ph[0]] + 0x80 , segm_sym_table[pm[1]], segm_sym_table[pm[0]]};
		        segm_indicate4(&display, symbols);
                } else {
                        
                        if(alarm_min > 59){
                                alarm_min = 0;
                                pam[1]=0;
                        }
                        if(alarm_hours >23){
                                alarm_hours = 0;
                                pah[1]=0;
                        }       
                                               
                        segm_bcd(alarm_min, pam);
                        segm_bcd(alarm_hours, pah);
                        uint8_t a_symbols[] = {segm_sym_table[pah[1]],segm_sym_table[pah[0]] + 0x80 , segm_sym_table[pam[1]], segm_sym_table[pam[0]]};
		        segm_indicate4(&display, a_symbols);
                }

                if(press_button(&alarm.alarm)){
                        if(set_alarm == true) {
                                set_alarm = false;
                                i=200;
                                do{
                                        segm_indicate4(&display, on_s);
                                        _delay_us(100);
                                        i--;
                                }while(i!=0);
                        } else {
                                set_alarm = true;
                                i=200;
                                do{
                                        segm_indicate4(&display, set);
                                        _delay_us(100);
                                        i--;
                                }while(i!=0);
                        }
                }

                if(press_button(&alarm.alarm_on)){
                        if (on_alarm == false){
                                on_alarm = true;
                                i=200;
                                do{
                                        segm_indicate4(&display, on_a);
                                        _delay_us(100);
                                        i--;
                                }while(i!=0);
                        } else {
                                on_alarm = false;
                                TCCR0B &= ~((1<<CS02)|(1<<CS00));
                                *(bip.port->PORT) &= ~(1<<bip.pin);
                                i=200;
                                do{
                                        segm_indicate4(&display, off_a);
                                        _delay_us(100);
                                        i--;
                                }while(i!=0);
                                
                        }
                }

                if ((alarm_hours == hours) && (alarm_min == min)){
                        if (on_alarm == true) TCCR0B |= (1<<CS02)|(1<<CS00);
                }

                        
                 
                         
	}
}

ISR (TIMER1_COMPA_vect)
{
        sec++;
}

ISR (INT0_vect)
{
        if (set_alarm == true){
                alarm_min++;
        } else {                
                min++;
        };
        _delay_us(500);     
}

ISR (INT1_vect)
{
        if (set_alarm == true){
                alarm_hours++;
        } else {
                hours++;
        }
        _delay_us(500);      
     
}


ISR (TIMER0_COMPA_vect)  
{
        i++;
        if (i >= 4) {
                *(bip.port->PORT) ^= 1<<bip.pin;
                i=0;
        }
}
