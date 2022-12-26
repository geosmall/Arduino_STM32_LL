#ifndef UVOS_GPIO_H
#define UVOS_GPIO_H

/* Public Functions */
extern void UVOS_GPIO_On(uint32_t gpios_dev_id, uint8_t gpio_id);
extern void UVOS_GPIO_Off(uint32_t gpios_dev_id, uint8_t gpio_id);
extern void UVOS_GPIO_Toggle(uint32_t gpios_dev_id, uint8_t gpio_id);

#endif // UVOS_GPIO_H
