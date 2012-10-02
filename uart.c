#include "stm32f4xx_conf.h"
#include "uart.h"
#include "stm32f4xx.h"
#include "ringbuf.h"
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#define RX_SIZE  128
#define TX_SIZE  128

#define MY_USART USART2


static struct ringbuf rx_buf = { .buf = (char[RX_SIZE]) {}, .bufsize = RX_SIZE };
static struct ringbuf tx_buf = { .buf = (char[TX_SIZE]) {}, .bufsize = TX_SIZE };

static volatile struct uart_stats {
    uint32_t    rx_overrun;
    uint32_t    rx_bytes;
    uint32_t    tx_bytes;
} uart_stats;


void USART2_IRQHandler(void)
{
    if (USART2->SR & USART_SR_RXNE) {
        if (!rb_putc(&rx_buf, MY_USART->DR))
            uart_stats.rx_overrun++;
        else
            uart_stats.rx_bytes++;
    }

    if (USART2->SR & USART_SR_TXE) {
        char c;
        if (rb_getc(&tx_buf, &c)) {
            // send a queued byte
            //
            USART2->DR = c;
        }
        else {
            // nothing to send, disable interrupt
            //
            USART2->CR1 &= ~USART_CR1_TXEIE;
        }
        uart_stats.tx_bytes++;
    }
}


int uart_chars_avail(void)
{
    return rx_buf.len;
}


ssize_t uart_write_r(struct _reent *r, int fd, const void *ptr, size_t len)
{
    const char *c = (const char*) ptr;

    for (int i = 0; i < len; i++) {
        while (!rb_putc(&tx_buf, *c));
        c++;

        // Enable TX empty interrupt
        MY_USART->CR1 |= USART_CR1_TXEIE;
    }

    return len;
}

ssize_t uart_read_r(struct _reent *r, int fd, void *ptr, size_t len)
{
    while (!rx_buf.len);

    if (len > rx_buf.len)
        len = rx_buf.len;

    char *c = (char*)ptr;
    for (int i = 0; i < len; i++)
        rb_getc(&rx_buf, c++);

    return len;
}


void uart_poll_send(const char *ch)
{
    while (*ch) {
        MY_USART->DR = *ch++ & 0xff;
        while (!(MY_USART->SR & USART_FLAG_TXE));
        uart_stats.tx_bytes++;
    }
}

/**
 * Initialize UART.
 *
 * \param  baudrate  Baudrate
 *
 *  PD5   USART2_TXD
 *  PD6   USART2_RXD
 *
 */
void uart_init(int baudrate)
{
    // Enable peripheral clocks
    //
    RCC->AHB1ENR |= RCC_AHB1Periph_GPIOD;
    RCC->APB1ENR |= RCC_APB1Periph_USART2;

    // Initialize Serial Port
    //
    GPIO_Init(GPIOD, &(GPIO_InitTypeDef) {
        .GPIO_Pin   = GPIO_Pin_5,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode  = GPIO_Mode_AF,
        .GPIO_OType = GPIO_OType_PP
    });

    GPIO_Init(GPIOD, &(GPIO_InitTypeDef) {
        .GPIO_Pin = GPIO_Pin_6,
        .GPIO_Mode = GPIO_Mode_AF,
        .GPIO_PuPd = GPIO_PuPd_UP
    });

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

    USART_Init(MY_USART, &(USART_InitTypeDef) {
        .USART_BaudRate = baudrate,
        .USART_WordLength = USART_WordLength_8b,
        .USART_StopBits = USART_StopBits_1,
        .USART_Parity = USART_Parity_No ,
        .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
        .USART_Mode = USART_Mode_Tx
    });

    NVIC_Init(&(NVIC_InitTypeDef) {
        .NVIC_IRQChannel = USART2_IRQn,
        .NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY,
        .NVIC_IRQChannelSubPriority = 0,
        .NVIC_IRQChannelCmd = ENABLE
    });

    USART_ITConfig(MY_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(MY_USART, ENABLE);
}

/*
// -------------------- Shell commands --------------------
//
static void cmd_baudrate(int argc, char *argv[])
{
    if (argc != 2) {
        printf("usage: %s <baudrate>\n", argv[0]);
        return;
    }

    uart_init(atoi(argv[1]));
}

static void cmd_uart_stats(void)
{
    printf("rx_bytes:   %8lu\n", uart_stats.rx_bytes);
    printf("rx_overrun: %8lu\n", uart_stats.rx_overrun);
    printf("tx_bytes:   %8lu\n", uart_stats.tx_bytes);
}


SHELL_CMD(baudrate,   (cmdfunc_t)cmd_baudrate,   "set baudrate")
SHELL_CMD(uart_stats, (cmdfunc_t)cmd_uart_stats, "show UART statistics")
*/
