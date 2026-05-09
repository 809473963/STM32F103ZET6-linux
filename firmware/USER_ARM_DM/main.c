#include "delay.h"
#include "led.h"
#include "protocol.h"
#include "sys.h"
#include "usart.h"

int main(void)
{
    uint16_t hb_ms = 0;

    delay_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    uart_init(115200);
    LED_Init();

    Protocol_Init();

    LED0 = 1;
    LED1 = 1;

    // printf removed: avoid polluting the protocol UART with startup text.

    while (1) {
        Protocol_TickMs(2);

        hb_ms += 2;
        if (hb_ms >= 500) {
            LED1 = !LED1;
            hb_ms = 0;
        }

        delay_ms(2);
    }
}
