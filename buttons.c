#include "buttons.h"
#include <stdint.h>



void init_buttons(struct button_int *button)
{
        struct button_Pin *ptrb[] ={
                &button->set_h,
                &button->set_m,
        };
        
       for (int i = 0; i < (uint8_t)(sizeof ptrb / sizeof *ptrb); i++) {
                *(ptrb[i]->port->PORT)&= ~(1 << ptrb[i]->pin);
                *(ptrb[i]->port->DDR) |= 1 << ptrb[i]->pin;
                 
        } 
        EICRA |= (1 << ISC01)|(1 << ISC11)|(1 << ISC00)|(1 << ISC10);    // The rising edge of INT0 generates an interrupt request.
        EIMSK |= (1 << INT0)| (1 << INT1);     // Turns on INT*
               
}       

void init_alarm_button(struct alarm_b *alarm)
{
        *(alarm->alarm.port->PORT) &= ~(1 << alarm->alarm.pin);
        *(alarm->alarm.port->DDR) |= (1 << alarm->alarm.pin);
}

bool press_button(struct button_Pin *btn)
{
	if (!(*(btn->port->PIN) & (1 << btn->pin))) {
		_delay_us(500);
		if (!(*(btn->port->PIN) & (1 << btn->pin)))
			return true;
	}
	return false;
}