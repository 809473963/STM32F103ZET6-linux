#include "delay.h"
#include "led.h"
#include "sys.h"
#include "usart.h"
#include "protocol.h"
#include "encoder_motor.h"

int main(void)
{
    uint16_t hb_ms = 0;
    uint16_t log_ms = 0;
    int32_t cnt1 = 0;
    int32_t cnt2 = 0;
    int32_t cnt3 = 0;

    delay_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    uart_init(115200);
    LED_Init();
    Protocol_Init();

    LED0 = 1;
    LED1 = 1;

    printf("Encoder motor app ready (3 motors).\r\n");
    printf("Sync control: motor1 + motor2. Motor3: independent.\r\n");

    while (1)
    {
        Protocol_TickMs(2);

        hb_ms += 2;
        if (hb_ms >= 500) {
            LED1 = !LED1;
            hb_ms = 0;
        }

        log_ms += 2;
        if (log_ms >= 200) {
            log_ms = 0;
            cnt1 = EncoderMotor_GetEncoderCountById(ENCODER_MOTOR_ID_1);
            cnt2 = EncoderMotor_GetEncoderCountById(ENCODER_MOTOR_ID_2);
            cnt3 = EncoderMotor_GetEncoderCountById(ENCODER_MOTOR_ID_3);
            printf("enc cnt m1=%ld m2=%ld m3=%ld\r\n", (long)cnt1, (long)cnt2, (long)cnt3);
        }

        delay_ms(2);
    }
}
