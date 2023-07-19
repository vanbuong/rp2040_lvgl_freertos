/**
 * \file            lwesp_ll_rp2040.c
 * \brief           Generic RP2040 driver, included in various RP2040 driver variants
 */

/*
 * Copyright (c) 2023 Tilen MAJERLE
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of LwESP - Lightweight ESP-AT parser library.
 *
 * Author:          Tilen MAJERLE <tilen@majerle.eu>
 * Version:         v1.1.2-dev
 */

/*
 * How it works
 *
 * On first call to \ref lwesp_ll_init, new thread is created and processed in usart_ll_thread function.
 * USART is configured in RX DMA mode and any incoming bytes are processed inside thread function.
 * DMA and USART implement interrupt handlers to notify main thread about new data ready to send to upper layer.
 *
 * More about UART + RX DMA: https://github.com/MaJerle/stm32-usart-dma-rx-tx
 *
 * \ref LWESP_CFG_INPUT_USE_PROCESS must be enabled in `lwesp_config.h` to use this driver.
 */

#include "lwesp/lwesp.h"
#include "lwesp/lwesp_input.h"
#include "lwesp/lwesp_mem.h"
#include "system/lwesp_ll.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"

#if !__DOXYGEN__

#if !LWESP_CFG_INPUT_USE_PROCESS
#error "LWESP_CFG_INPUT_USE_PROCESS must be enabled in `lwesp_config.h` to use this driver."
#endif /* LWESP_CFG_INPUT_USE_PROCESS */

#if !defined(LWESP_USART_DMA_RX_BUFF_SIZE)
#define LWESP_USART_DMA_RX_BUFF_SIZE 0x1000
#define BUFF_MOD (LWESP_USART_DMA_RX_BUFF_SIZE - 1)
#endif /* !defined(LWESP_USART_DMA_RX_BUFF_SIZE) */

#if !defined(LWESP_MEM_SIZE)
#define LWESP_MEM_SIZE 0x4000
#endif /* !defined(LWESP_MEM_SIZE) */

#if !defined(LWESP_USART_RDR_NAME)
#define LWESP_USART_RDR_NAME RDR
#endif /* !defined(LWESP_USART_RDR_NAME) */

/* USART memory */
static uint8_t usart_mem[LWESP_USART_DMA_RX_BUFF_SIZE];
static uint8_t is_running, initialized;
static size_t read_pos, write_pos;

/* USART thread */
static void usart_ll_thread(void* arg);
static TaskHandle_t usart_ll_thread_id;

/* Message queue */
static QueueHandle_t usart_ll_mbox_id;

static void LWESP_USART_IRQHANDLER(void);

/**
 * \brief           USART data processing
 */
static void
usart_ll_thread(void* arg) {
    LWESP_UNUSED(arg);

    while (1) {
        void* d;
        /* Wait for the event message USART */
        xQueueReceive(usart_ll_mbox_id, &d, portMAX_DELAY);

        /* Read data */
        if (read_pos != write_pos && is_running) {
            if (write_pos > read_pos) {
                lwesp_input_process(&usart_mem[read_pos], write_pos - read_pos);
            } else {
                lwesp_input_process(&usart_mem[read_pos], sizeof(usart_mem) - read_pos);
                if (write_pos > 0) {
                    lwesp_input_process(&usart_mem[0], write_pos);
                }
            }
            read_pos = write_pos;
        }
    }
}

/**
 * \brief           Configure UART using DMA for receive in double buffer mode and IDLE line detection
 */
static void
prv_configure_uart(uint32_t baudrate) {

    if (!initialized) {
#if defined(LWESP_RESET_PIN)
        /* Configure RESET pin */
        gpio_init(21);
        gpio_set_dir(21, GPIO_OUT);
        gpio_put(21, 1);
#endif /* defined(LWESP_RESET_PIN) */

        /* Configure USART pins */
        /* TX PIN */
        gpio_set_function(8, GPIO_FUNC_UART);
        /* RX PIN */
        gpio_set_function(9, GPIO_FUNC_UART);

        /* Configure UART */
        uart_init(uart1, baudrate);

        // Set UART flow control CTS/RTS, we don't want these, so turn them off
        uart_set_hw_flow(uart1, false, false);

        // And set up and enable the interrupt handlers
        irq_set_exclusive_handler(UART1_IRQ, LWESP_USART_IRQHANDLER);

        // Now enable the UART to send interrupts - RX only
        uart_set_irq_enables(uart1, true, false);

        // Set FIFO interrupt trigger level
        // This step has to put after uart_set_irq_enables
        // Set rx threshold is 1/2 of FIFO size (16 of 32 bytes)
        const uint32_t UART_RX_THRESHOLD = 2;
        hw_write_masked(&uart_get_hw(uart1)->ifls, UART_RX_THRESHOLD << UART_UARTIFLS_RXIFLSEL_LSB,
                        UART_UARTIFLS_RXIFLSEL_BITS);

        /* Enable USART interrupts*/
        irq_set_enabled(UART1_IRQ, true);

        read_pos = 0;
        write_pos = 0;
        is_running = 1;

        /* Start USART */
    } else {
        vTaskDelay(pdMS_TO_TICKS(10));
        /* Disable and reinit uart again*/
        uart_get_hw(uart1)->cr &= ~UART_UARTCR_UARTEN_BITS;
        uart_set_baudrate(uart1, baudrate);
        uart_get_hw(uart1)->cr |= UART_UARTCR_UARTEN_BITS;
    }

    /* Create mbox and start thread */
    if (usart_ll_mbox_id == NULL) {
        usart_ll_mbox_id = xQueueCreate(10, sizeof(void*));
    }
    if (usart_ll_thread_id == NULL) {
        xTaskCreate(usart_ll_thread, "uart_ll_thread", 1536 / sizeof(portSTACK_TYPE), NULL, configMAX_PRIORITIES - 1, &usart_ll_thread_id);
    }
}

#if defined(LWESP_RESET_PIN)
/**
 * \brief           Hardware reset callback
 */
static uint8_t
prv_reset_device(uint8_t state) {
    gpio_put(LWESP_RESET_PIN, state);
    return 1;
}
#endif /* defined(LWESP_RESET_PIN) */

/**
 * \brief           Send data to ESP device
 * \param[in]       data: Pointer to data to send
 * \param[in]       len: Number of bytes to send
 * \return          Number of bytes sent
 */
static size_t
prv_send_data(const void* data, size_t len) {
    const uint8_t* d = data;

    uart_write_blocking(uart1, d, len);
    return len;
}

/**
 * \brief           Callback function called from initialization process
 */
lwespr_t
lwesp_ll_init(lwesp_ll_t* ll) {
#if !LWESP_CFG_MEM_CUSTOM
    static uint8_t memory[LWESP_MEM_SIZE];
    const lwesp_mem_region_t mem_regions[] = {{memory, sizeof(memory)}};

    if (!initialized) {
        lwesp_mem_assignmemory(mem_regions, LWESP_ARRAYSIZE(mem_regions)); /* Assign memory for allocations */
    }
#endif                                                                     /* !LWESP_CFG_MEM_CUSTOM */

    if (!initialized) {
        ll->send_fn = prv_send_data;     /* Set callback function to send data */
#if defined(LWESP_RESET_PIN)
        ll->reset_fn = prv_reset_device; /* Set callback for hardware reset */
#endif                                   /* defined(LWESP_RESET_PIN) */
    }

    prv_configure_uart(ll->uart.baudrate); /* Initialize UART for communication */
    initialized = 1;
    return lwespOK;
}

/**
 * \brief           Callback function to de-init low-level communication part
 */
lwespr_t
lwesp_ll_deinit(lwesp_ll_t* ll) {
    if (usart_ll_mbox_id != NULL) {
        QueueHandle_t tmp = usart_ll_mbox_id;
        usart_ll_mbox_id = NULL;
        vQueueDelete(tmp);
    }
    if (usart_ll_thread_id != NULL) {
        TaskHandle_t tmp = usart_ll_thread_id;
        usart_ll_thread_id = NULL;
        vTaskDelete(tmp);
    }
    initialized = 0;
    LWESP_UNUSED(ll);
    return lwespOK;
}

/**
 * \brief           UART global interrupt handler
 */
void
LWESP_USART_IRQHANDLER(void) {
    bool timeout_int = uart_get_hw(uart1)->mis & UART_UARTMIS_OFFSET;
	while (uart_is_readable(uart1)) {
		usart_mem[write_pos] = (uint8_t)uart_getc(uart1);
        write_pos = (write_pos + 1) % BUFF_MOD;
	}

    if (timeout_int && usart_ll_mbox_id != NULL) {
        void* d = (void*)1;
        BaseType_t yield = pdFALSE;
        xQueueSendToBackFromISR(usart_ll_mbox_id, &d, &yield);
    }
}

#endif /* !__DOXYGEN__ */
