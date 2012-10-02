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
#include "CMSIS_DSP/arm_math.h"

#define DAC_DHR12R2_ADDRESS    0x40007414
#define DAC_DHR12R1_ADDRESS    0x40007408

#define SIN_TABLE_SIZE  128

uint16_t sin_table[SIN_TABLE_SIZE];

xQueueHandle queue;

void send_uart(char c) {
	putchar(c);
}

void send_string_uart(const char* s) {
	puts(s);
}
void send_byte_to_LIS(uint8_t byte, uint8_t address) {
	LIS302DL_Write(&byte, address, 1);
}

void display_bar(int min_val, int max_val, int value) {
	static char display[130];
	int range = (max_val - min_val);
	int average = (max_val + min_val) / 2;
	int display_value = (value - average) * 128 / range + 64;
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
		if (i > min && i < max)
			display[i] = '-';
		else
			display[i] = ' ';
	display[sizeof(display) - 2] = '\n';
	display[sizeof(display) - 1] = 0;
	send_string_uart(display);
}

void init_accelerometer(void) {
	LIS302DL_InitTypeDef LIS302DL_InitStruct;
	LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
	LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100;
	LIS302DL_InitStruct.Axes_Enable = LIS302DL_X_ENABLE | LIS302DL_Y_ENABLE
			| LIS302DL_Z_ENABLE;
	LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
	LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
	LIS302DL_Init(&LIS302DL_InitStruct);
	send_byte_to_LIS(0x4, LIS302DL_CTRL_REG3_ADDR);
}

void wire_exti_interrupt0_to_acceleromerter_int1() {
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0 );
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void init_usart(void) {
	uart_init(230400);
}

void TIM6_Config(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	/* TIM6 Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	/* Time base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	/* TIM6 TRGO selection */
	TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update );

	/* TIM6 enable counter */
	TIM_Cmd(TIM6, ENABLE);
}

void DAC_Ch2_SineWaveConfig(void) {
	float32_t xfactor = 2.0 * PI / SIN_TABLE_SIZE;
	for (int i = 0; i < SIN_TABLE_SIZE; i++) {
		uint16_t ival = arm_sin_f32(i * xfactor) * 2046.0 + 2046.0;
		sin_table[i] = ival;
		//display_bar(0, 4096, (int) sin_table[i]);
	}

	DMA_InitTypeDef DMA_InitStructure;
	DAC_InitTypeDef DAC_InitStructure;

	/* DAC channel1 Configuration */
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	/* DMA1_Stream5 channel1 configuration **************************************/
	DMA_DeInit(DMA1_Stream5 );
	DMA_InitStructure.DMA_Channel = DMA_Channel_7;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) DAC_DHR12R1_ADDRESS;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &sin_table;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = SIN_TABLE_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream5, ENABLE);
	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_DMACmd(DAC_Channel_1, ENABLE);
}

void init_DAC_DMA(void) {
	TIM6_Config();
	GPIO_InitTypeDef GPIO_InitStructure;

	/* DMA1 clock and GPIOA clock enable (to be used with DAC) */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 | RCC_AHB1Periph_GPIOA, ENABLE);
	/* DAC Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	/* DAC channel 1 (DAC_OUT1 = PA.4) configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void toggleLedCallback(xTimerHandle xTimer) {
	static char buffer[2048];
	STM_EVAL_LEDToggle(LED5);

	vTaskGetRunTimeStats(buffer);
	send_string_uart(buffer);
}

void acceleration_task() {
	vTaskDelay(60);
	uint8_t buffer[6];
	LIS302DL_Read(buffer, LIS302DL_OUT_X_ADDR, 6);
	for (;;) {
		signed char result;
		xQueueReceive(queue, &result, portMAX_DELAY);
		display_bar(-128, 128, result);
	}
}

void initx(void) {
	SCB ->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);
	STM_EVAL_LEDOff(LED3);
	STM_EVAL_LEDOff(LED4);
	STM_EVAL_LEDOff(LED5);
	STM_EVAL_LEDOff(LED6);
	init_usart();
	send_string_uart("INIT UART\n");
	init_accelerometer();
	send_string_uart("INIT ACCE\n");
	wire_exti_interrupt0_to_acceleromerter_int1();
	send_string_uart("INIT INT\n");
	init_DAC_DMA();
	send_string_uart("INIT DMA\n");
	DAC_Ch2_SineWaveConfig();
	send_string_uart("INIT SINE\n");

}



int main(void) {
	initx();
	xTimerHandle timer = xTimerCreate((const signed char *) "timer",
			1000 / portTICK_RATE_MS, pdTRUE, NULL, toggleLedCallback);
	send_string_uart("timer created\n");
	queue = xQueueCreate(20, 1);
	xTaskCreate(acceleration_task, (signed char*)"acceleration_task", 128, NULL,
			tskIDLE_PRIORITY+1, NULL);
	send_string_uart("queue created\n");
	xTimerStart(timer, 0);
	send_string_uart("timer started\n");
	send_string_uart("INIT DONE, Starting\n");
	vTaskStartScheduler();
	return 0;
}

uint32_t LIS302DL_TIMEOUT_UserCallback(void) {
	send_string_uart("ACCELEROMETER TIMEOUT\n");
	STM_EVAL_LEDToggle(LED6);
	return 0;
}

void EXTI0_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line0 ) != RESET) {
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		unsigned long ulDummy;
		ulDummy = portSET_INTERRUPT_MASK_FROM_ISR();
		{
			uint8_t buffer[6];
			LIS302DL_Read(buffer, LIS302DL_OUT_X_ADDR, 6);
			xQueueSendFromISR(queue, &(buffer[4]), &xHigherPriorityTaskWoken);
			EXTI_ClearITPendingBit(EXTI_Line0 );
		}
		portCLEAR_INTERRUPT_MASK_FROM_ISR( ulDummy);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
}



void HardFault_HandlerC(unsigned long *hardfault_args) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
	volatile unsigned long stacked_r0;
	volatile unsigned long stacked_r1;
	volatile unsigned long stacked_r2;
	volatile unsigned long stacked_r3;
	volatile unsigned long stacked_r12;
	volatile unsigned long stacked_lr;
	volatile unsigned long stacked_pc;
	volatile unsigned long stacked_psr;
	volatile unsigned long _CFSR;
	volatile unsigned char _MMFSR;
	volatile unsigned char _BFSR;
	volatile uint16_t _UFSR;
	volatile unsigned long _HFSR;
	volatile unsigned long _DFSR;
	volatile unsigned long _AFSR;
	volatile unsigned long _BFAR;
	volatile unsigned long _MMAR;

	stacked_r0 = ((unsigned long) hardfault_args[0]);
	stacked_r1 = ((unsigned long) hardfault_args[1]);
	stacked_r2 = ((unsigned long) hardfault_args[2]);
	stacked_r3 = ((unsigned long) hardfault_args[3]);
	stacked_r12 = ((unsigned long) hardfault_args[4]);
	stacked_lr = ((unsigned long) hardfault_args[5]);
	stacked_pc = ((unsigned long) hardfault_args[6]);
	stacked_psr = ((unsigned long) hardfault_args[7]);

	// Configurable Fault Status Register
	// Consists of MMSR, BFSR and UFSR
	_CFSR = (*((volatile unsigned long *) (0xE000ED28)));

	_MMFSR = (*((volatile unsigned char *) (0xE000ED28)));
	_BFSR = (*((volatile unsigned char *) (0xE000ED29)));
	_UFSR = (*((volatile uint16_t *) (0xE000ED2A)));
	// Hard Fault Status Register
	_HFSR = (*((volatile unsigned long *) (0xE000ED2C)));

	// Debug Fault Status Register
	_DFSR = (*((volatile unsigned long *) (0xE000ED30)));

	// Auxiliary Fault Status Register
	_AFSR = (*((volatile unsigned long *) (0xE000ED3C)));

	// Read the Fault Address Registers. These may not contain valid values.
	// Check BFARVALID/MMARVALID to see if they are valid values
	// MemManage Fault Address Register
	_MMAR = (*((volatile unsigned long *) (0xE000ED34)));
	// Bus Fault Address Register
	_BFAR = (*((volatile unsigned long *) (0xE000ED38)));

	__asm("BKPT #0\n");
	// Break into the debugger
#pragma GCC diagnostic pop

}

void BusFault_Handler(void) {
	send_string_uart("BusFault_Handler\n");
}

void MemManage_Handler(void) {
	send_string_uart("MemManage_Handler\n");
}

void HardFault_Handler(void) {
	/*
	 * Get the appropriate stack pointer, depending on our mode,
	 * and use it as the parameter to the C handler. This function
	 * will never return
	 */

	send_string_uart("HardFault_Handler\n");
	__asm( ".syntax unified\n"
			"MOVS   R0, #4  \n"
			"MOV    R1, LR  \n"
			"TST    R0, R1  \n"
			"BEQ    _MSP    \n"
			"MRS    R0, PSP \n"
			"B      HardFault_HandlerC      \n"
			"_MSP:  \n"
			"MRS    R0, MSP \n"
			"B      HardFault_HandlerC      \n"
			".syntax divided\n");
}

void NMI_Handler(void) {
	send_string_uart("NMI_Handler\n");
}

void UsageFault_Handler(void) {
	send_string_uart("UsageFault_Handler\n");
}

void DebugMon_Handler(void) {
	send_string_uart("DebugMon_Handler\n");
}

void WWDG_IRQHandler(void) {
	send_string_uart("WWDG_IRQHandler\n");
}

void PVD_IRQHandler(void) {
	send_string_uart("PVD_IRQHandler\n");
}
