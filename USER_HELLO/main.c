#include "delay.h"
#include "hello_ack.h"
#include "led.h"
#include "sys.h"
#include "usart.h"

int main(void)
{
    delay_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    uart_init(115200);
    LED_Init();
    HelloAck_Init();

    LED0 = 1;
    LED1 = 1;

    while (1) {
        LED0 = !LED0;
        delay_ms(200);
    }
}
