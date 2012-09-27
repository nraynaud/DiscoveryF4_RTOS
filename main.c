#include "stm32f4xx_conf.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

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

void init_usart(void){
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    /* Configure USART2 Tx (PA.02) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // Map USART2 to A.02
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    // Initialize USART
    USART_InitStructure.USART_BaudRate = 9660;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    /* Configure USART */
    USART_Init(USART2, &USART_InitStructure);
    /* Enable the USART */
    USART_Cmd(USART2, ENABLE);
}

static void TaskA(void *pvParameters)
{
    for(;;)
    {
        STM_EVAL_LEDToggle(LED3);
        vTaskDelay(500); 
    }
}

void toggleLedCallback(xTimerHandle xTimer ) {
    STM_EVAL_LEDToggle(LED5);
    send_string_uart("LOL\n");  
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
}

int main(void)
{
    initx();
    xTimerHandle timer = xTimerCreate("timer", 500 / portTICK_RATE_MS, pdTRUE, NULL, toggleLedCallback);
    xTimerStart(timer, 0);
    xTaskCreate(TaskA, (signed char*)"TaskA", 128, NULL, tskIDLE_PRIORITY+1, NULL);
    //xTaskCreate(TaskB, (signed char*)"TaskB", 128, NULL, tskIDLE_PRIORITY+1, NULL);
    //xTaskCreate(TaskC, (signed char*)"TaskC", 128, NULL, tskIDLE_PRIORITY+1, NULL);
    //xTaskCreate(TaskD, (signed char*)"TaskD", 128, NULL, tskIDLE_PRIORITY+1, NULL);
    vTaskStartScheduler();
    return 0;
}
