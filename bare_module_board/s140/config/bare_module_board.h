#ifndef BARE_MODULE_BOARD_H
#define BARE_MODULE_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

#define LEDS_NUMBER    1

#define LED1_B         NRF_GPIO_PIN_MAP(0,28)

#define LED_1          LED1_B

#define LEDS_ACTIVE_STATE 0

#define LEDS_LIST { LED_1}

#define LEDS_INV_MASK  LEDS_MASK

#define BSP_LED_0      LED_1

#define BUTTONS_NUMBER 1

#define BUTTON_1       NRF_GPIO_PIN_MAP(0,3)
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1

#define BSP_SELF_PINRESET_PIN NRF_GPIO_PIN_MAP(0, 19)

#define HWFC           true

#define PSS_RELAY_0_PIN     NRF_GPIO_PIN_MAP(0,30)
#define PSS_RELAY_1_PIN     NRF_GPIO_PIN_MAP(0,31)
#define PSS_RELAY_2_PIN     NRF_GPIO_PIN_MAP(0,29)
#define PSS_RELAY_3_PIN     NRF_GPIO_PIN_MAP(0,2)
#define PSS_RELAY_4_PIN     NRF_GPIO_PIN_MAP(1,13)

#ifdef __cplusplus
}
#endif

#endif // BARE_MODULE_BOARD_H
