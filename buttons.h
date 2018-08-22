#include <stdint.h>
#include </usr/lib/gcc/avr/5.4.0/include/stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
struct button_Port {
	volatile uint8_t *DDR;	/* addr of GPIO DDRx direction register */
	volatile uint8_t *PIN;	/* addr of GPIO PINx input register */
	volatile uint8_t *PORT;	/* addr of GPIO PORTx data register */
};

struct button_Pin {
	struct button_Port *port;	/* GPIO port */
	uint8_t pin;		/* number of pin in GPIO port */
};

struct button_int {
        struct button_Pin set_h;
        struct button_Pin set_m;
};

struct alarm_b {
        struct button_Pin alarm;
        struct button_Pin alarm_on;
};

void init_buttons(struct button_int *button);

void init_alarm_button(struct alarm_b *alarm);

bool press_button(struct button_Pin *btn);