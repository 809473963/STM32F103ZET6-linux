#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "motor.h"
#include "protocol.h"
 
/************************************************
 ALIENTEK战舰STM32开发板实验3
 按键输入实验  
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/


int main(void)
{
    delay_init();            // 延时函数初始化   
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 设置NVIC中断分组2
    uart_init(115200);       // 串口初始化为 115200
    LED_Init();              // LED端口初始化
    Motor_Init();            // 电机驱动初始化
    Protocol_Init();         // 通信协议初始化

    LED0 = 0; // 启动提示
    LED1 = 0;
    delay_ms(300);
    LED0 = 1;
    LED1 = 1;

    while(1)
    {
        Motor_Service();

        // 电机控制由串口协议帧驱动：
        // CMD_SET_MOTOR_SPEED / CMD_STOP_ALL
        // 这里仅做心跳，表示系统在线。
        static u16 hb = 0;
        hb++;
        if (hb >= 500) {
            LED1 = !LED1;
            hb = 0;
        }
    }
}
