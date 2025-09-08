#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

int main(void) {
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOB);
	// Led setup
	gpio_set_mode(GPIOC, 
			GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL,
			GPIO13);
	
	// Button setup
	gpio_set_mode(GPIOB,
			GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_PULL_UPDOWN,
			GPIO12);

	while(1) {
		if (gpio_get(GPIOB, GPIO12)){
			for (int i = 0; i < 1000000; i++) {
				__asm__("nop");
			}

			gpio_toggle(GPIOC, GPIO13);
		}
	}
}
