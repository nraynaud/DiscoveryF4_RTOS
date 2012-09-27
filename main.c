#include "stm32f4xx_conf.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

xQueueHandle queue;

void send_uart(char c) {
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, c);
}

void send_string_uart(const char* s) {
    char c;
    int i = 0;
    while ((c = s[i++]))
        send_uart(c);
}
void send_byte_to_LIS(uint8_t byte,uint8_t address) {
    LIS302DL_Write(&byte, address, 1);
}

void init_accelerometer(void) {
    LIS302DL_InitTypeDef  LIS302DL_InitStruct;
    LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
    LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100;
    LIS302DL_InitStruct.Axes_Enable = LIS302DL_X_ENABLE | LIS302DL_Y_ENABLE | LIS302DL_Z_ENABLE;
    LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
    LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
    LIS302DL_Init(&LIS302DL_InitStruct);
    send_byte_to_LIS(0x4, LIS302DL_CTRL_REG3_ADDR);
}

void wire_exti_interrupt0_to_acceleromerter_int1() {
    GPIO_InitTypeDef   GPIO_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;
    EXTI_InitTypeDef   EXTI_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void init_usart(void){
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    USART_InitStructure.USART_BaudRate = 230400;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
    USART_Cmd(USART2, ENABLE);
}

void toggleLedCallback(xTimerHandle xTimer ) {
    STM_EVAL_LEDToggle(LED5);
}

void initx(void)
{
    STM_EVAL_LEDInit(LED3);
    STM_EVAL_LEDInit(LED4);
    STM_EVAL_LEDInit(LED5);
    STM_EVAL_LEDInit(LED6);
    STM_EVAL_LEDOff(LED3);
    STM_EVAL_LEDOff(LED4);
    STM_EVAL_LEDOff(LED5);
    STM_EVAL_LEDOff(LED6);
    init_usart();
    init_accelerometer();
    wire_exti_interrupt0_to_acceleromerter_int1();
}

void acceleration_task() {
    for(;;) {
        char display[130];
        signed char result;
        xQueueReceive(queue,&result,portMAX_DELAY);
        int display_value = (result + 128) / 2;
        int min = 0;
        int max = 0;
        if (display_value < 64) {
            min = display_value;
            max = 64;
        } else {
            min = 64;
            max = display_value;
        }
        for (int i = 0; i < sizeof(display); i++)
            if (i> min && i < max)
                display[i] = '-';
            else
                display[i] = ' ';
        display[sizeof(display) - 2] = '\n';
        display[sizeof(display) - 1] = 0;
        send_string_uart(display);
    }
}

int main(void)
{
    initx();
    xTimerHandle timer = xTimerCreate((const signed char *)"timer", 500 / portTICK_RATE_MS, pdTRUE, NULL, toggleLedCallback);
    xTimerStart(timer, 500);
    queue = xQueueCreate(20, 1);
    uint8_t Buffer[6];
    LIS302DL_Read(Buffer, LIS302DL_OUT_X_ADDR, 6);
    xTaskCreate(acceleration_task, (signed char*)"acceleration_task", 128, NULL, tskIDLE_PRIORITY+1, NULL);
    vTaskStartScheduler();
    return 0;
}

uint32_t LIS302DL_TIMEOUT_UserCallback(void) {
    send_string_uart("ACCELEROMETER TIMEOUT\n");
    STM_EVAL_LEDToggle(LED6);
    return 0;
}

void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
        unsigned long ulDummy;
        ulDummy = portSET_INTERRUPT_MASK_FROM_ISR();
        {
            uint8_t buffer[6];
            LIS302DL_Read(buffer, LIS302DL_OUT_X_ADDR, 6);
            xQueueSendFromISR(queue, &(buffer[4]), &xHigherPriorityTaskWoken);
            EXTI_ClearITPendingBit(EXTI_Line0);
        }
        portCLEAR_INTERRUPT_MASK_FROM_ISR( ulDummy );
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
}
