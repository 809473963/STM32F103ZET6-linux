#include "delay.h"
#include "led.h"
#include "sys.h"
#include "usart.h"
#include "protocol.h"
#include "motor86.h"

int main(void)
{
    uint16_t hb = 0;

    delay_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    uart_init(115200);
    LED_Init();
    Motor86_Init();
    Protocol_Init();

    LED0 = 1;
    LED1 = 1;

    while (1)
    {
        hb++;
        if (hb >= 500) {
            LED1 = !LED1;
            hb = 0;
        }

        delay_ms(2);
    }
}
