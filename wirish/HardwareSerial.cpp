/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2011, 2012 LeafLabs, LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file wirish/HardwareSerial.cpp
 * @brief Wirish serial port implementation.
 */

#include <wirish/HardwareSerial.h>

#include <libmaple/libmaple.h>
#include <libmaple/gpio.h>
#include <libmaple/dma.h>
#include <libmaple/timer.h>
#include <libmaple/usart.h>
#include <wirish/wirish.h>

#define DEFINE_HWSERIAL(name, n)                                   \
    HardwareSerial name(USART##n,                                  \
                        BOARD_USART##n##_TX_PIN,                   \
                        BOARD_USART##n##_RX_PIN)

#if BOARD_HAVE_USART1
DEFINE_HWSERIAL(Serial1, 1);
#endif
#if BOARD_HAVE_USART2
DEFINE_HWSERIAL(Serial2, 2);
#endif
#if BOARD_HAVE_USART3
DEFINE_HWSERIAL(Serial3, 3);
#endif
#if BOARD_HAVE_UART4
DEFINE_HWSERIAL(Serial4, 4);
#endif
#if BOARD_HAVE_UART5
DEFINE_HWSERIAL(Serial5, 5);
#endif
#if BOARD_HAVE_USART6
DEFINE_HWSERIAL(Serial6, 6);
#endif

HardwareSerial::HardwareSerial(usart_dev *usart_device,
                               uint8 tx_pin,
                               uint8 rx_pin) {
    this->usart_device = usart_device;
    this->tx_pin = tx_pin;
    this->rx_pin = rx_pin;
    this->dmaTx = NULL;
    this->dmaRx = NULL;
}

/*
 * Set up/tear down
 */

#if STM32_MCU_SERIES == STM32_SERIES_F1
/* F1 MCUs have no GPIO_AFR[HL], so turn off PWM if there's a conflict
 * on this GPIO bit. */
static void disable_timer_if_necessary(timer_dev *dev, uint8 ch) {
    if (dev != NULL) {
        timer_set_mode(dev, ch, TIMER_DISABLED);
    }
}
#elif (STM32_MCU_SERIES == STM32_SERIES_F2) ||    \
      (STM32_MCU_SERIES == STM32_SERIES_F4)
#define disable_timer_if_necessary(dev, ch) ((void)0)
#else
#warning "Unsupported STM32 series; timer conflicts are possible"
#endif

void HardwareSerial::begin(uint32 baud) {
    ASSERT(baud <= this->usart_device->max_baud);

    if (baud > this->usart_device->max_baud) {
        return;
    }

    const stm32_pin_info *txi = &PIN_MAP[this->tx_pin];
    const stm32_pin_info *rxi = &PIN_MAP[this->rx_pin];

    disable_timer_if_necessary(txi->timer_device, txi->timer_channel);

    usart_config_gpios_async(this->usart_device,
                             rxi->gpio_device, rxi->gpio_bit,
                             txi->gpio_device, txi->gpio_bit,
                             0);
    usart_init(this->usart_device);
    usart_set_baud_rate(this->usart_device, USART_USE_PCLK, baud);
    usart_enable(this->usart_device);
}

void HardwareSerial::end(void) {
    usart_disable(this->usart_device);
}

/*
 * I/O
 */

uint32 HardwareSerial::available(void) {
    return usart_data_available(this->usart_device);
}

uint8 HardwareSerial::read(void) {
    // Block until a byte becomes available, to save user confusion.
    while (!this->available())
        ;
    return usart_getc(this->usart_device);
}

void HardwareSerial::write(unsigned char ch) {
    usart_putc(this->usart_device, ch);
}

int HardwareSerial::read(uint8 *buf, uint32 len) {
    dma_message msg;
    read(msg((volatile unsigned char* const)buf, len));
    dma_rx_wait();
    return len;
}

void HardwareSerial::write(const void *buf, uint32 len) {
    dma_message msg;
    write(msg((volatile unsigned char* const)buf, len));
    dma_tx_wait();
}

void HardwareSerial::write(dma_message& newMsg) {
    dma_tx_wait();
    newMsg.claim();
    dmaTx = &newMsg;
    dmaTxConf.tube_src = (volatile char*)(dmaTx->data);
    dmaTxConf.tube_nr_xfers = dmaTx->length;
    int status = dma_tube_cfg(this->usart_device->dma_device, this->usart_device->dma_tx_tube, &dmaTxConf);
    ASSERT(status == DMA_TUBE_CFG_SUCCESS);
    dma_enable(this->usart_device->dma_device, this->usart_device->dma_tx_tube);
}

int HardwareSerial::read(dma_message& newMsg) {
    if(dmaRx != NULL) return 0;
    if(receivedBytes > 0) {
        int r = receivedBytes;
        receivedBytes = 0;
        return r;
    }
    dmaRx = &newMsg;
    receivedBytes = 0;
    dmaRxConf.tube_dst = (volatile char*)(dmaRx->data);
    dmaRxConf.tube_nr_xfers = dmaRx->length;

    int status = dma_tube_cfg(this->usart_device->dma_device, this->usart_device->dma_rx_tube, &dmaRxConf);
    ASSERT(status == DMA_TUBE_CFG_SUCCESS);
    dma_enable(this->usart_device->dma_device, this->usart_device->dma_rx_tube);
    return 0;
}

void HardwareSerial::irq_dma_tx(void) {
    dma_irq_cause cause = dma_get_irq_cause(this->usart_device->dma_device, this->usart_device->dma_tx_tube);
    dma_disable(this->usart_device->dma_device, this->usart_device->dma_tx_tube);
    dmaTx->ret();
    switch(cause)
    {
        case DMA_TRANSFER_COMPLETE:
            // Transfer completed
            break;
        case DMA_TRANSFER_ERROR:
            // An error occurred during transfer
            write(*dmaTx);
            break;
        default:
            // Something went horribly wrong.
            // Should never happen.
            break;
    }
    if(dmaTx->callback) {
        (*(dmaTx->callback))(dmaTx, cause);
    }
}

#if BOARD_HAVE_USART1
void irq_usart_dma_tx_1(void) {Serial1.irq_dma_tx();}
#endif
#if BOARD_HAVE_USART2
void irq_usart_dma_tx_2(void) {Serial2.irq_dma_tx();}
#endif
#if BOARD_HAVE_USART3
void irq_usart_dma_tx_3(void) {Serial3.irq_dma_tx();}
#endif

void HardwareSerial::irq_dma_rx(void) {
    dma_irq_cause cause = dma_get_irq_cause(this->usart_device->dma_device, this->usart_device->dma_rx_tube);
    dma_disable(this->usart_device->dma_device, this->usart_device->dma_rx_tube);
    if(DMA_TRANSFER_COMPLETE == cause) {
        receivedBytes = dmaRxConf.tube_nr_xfers;
    }
    if(dmaRx->callback) {
        (*(dmaRx->callback))(dmaRx, cause);
    }
    dmaRx = NULL;
}

#if BOARD_HAVE_USART1
void irq_usart_dma_rx_1(void) {Serial1.irq_dma_rx();}
#endif
#if BOARD_HAVE_USART2
void irq_usart_dma_rx_2(void) {Serial2.irq_dma_rx();}
#endif
#if BOARD_HAVE_USART3
void irq_usart_dma_rx_3(void) {Serial3.irq_dma_rx();}
#endif

void HardwareSerial::setup_dma_tx(void) {
    void(*irq)(void);
    #if BOARD_HAVE_USART3
    if(this->tx_pin == Serial3.tx_pin) {
        irq = irq_usart_dma_tx_3;
        dmaTxConf.tube_req_src = DMA_REQ_SRC_USART3_TX;
    } else 
    #endif
    #if BOARD_HAVE_USART2
    if(this->tx_pin == Serial2.tx_pin) {
        irq = irq_usart_dma_tx_2;
        dmaTxConf.tube_req_src = DMA_REQ_SRC_USART2_TX;
    } else 
    #endif
    #if BOARD_HAVE_USART1
    if(this->tx_pin == Serial1.tx_pin) {
        irq = irq_usart_dma_tx_1;
        dmaTxConf.tube_req_src = DMA_REQ_SRC_USART1_TX;
    } else 
    #endif
    return;

    dma_attach_interrupt(this->usart_device->dma_device, this->usart_device->dma_tx_tube, irq);

    dmaTxConf.tube_dst = &(this->usart_device->regs->DR);
    dmaTxConf.tube_src_size = DMA_SIZE_8BITS;
    dmaTxConf.tube_dst_size = DMA_SIZE_8BITS;

    dmaTxConf.tube_flags =
                   (DMA_CFG_SRC_INC |
                    DMA_CFG_ERR_IE  |
                    DMA_CFG_CMPLT_IE);
    dmaTxConf.target_data = NULL;

    this->usart_device->regs->CR3 |= USART_CR3_DMAT;
    dma_init(DMA1);
}
void HardwareSerial::setup_dma_rx(void) {
    void(*irq)(void);
    #if BOARD_HAVE_USART3
    if(this->rx_pin == Serial3.rx_pin) {
        irq = irq_usart_dma_rx_3;
        dmaRxConf.tube_req_src = DMA_REQ_SRC_USART3_RX;
    } else 
    #endif
    #if BOARD_HAVE_USART2
    if(this->rx_pin == Serial2.rx_pin) {
        irq = irq_usart_dma_rx_2;
        dmaRxConf.tube_req_src = DMA_REQ_SRC_USART2_RX;
    } else
    #endif
    #if BOARD_HAVE_USART1
    if(this->rx_pin == Serial1.rx_pin) {
        irq = irq_usart_dma_rx_1;
        dmaRxConf.tube_req_src = DMA_REQ_SRC_USART1_RX;
    } else 
    #endif
    return;

    dma_attach_interrupt(this->usart_device->dma_device, this->usart_device->dma_rx_tube, irq);

    dmaRxConf.tube_src = &(this->usart_device->regs->DR);
    dmaRxConf.tube_src_size = DMA_SIZE_8BITS;
    dmaRxConf.tube_dst_size = DMA_SIZE_8BITS;

    dmaRxConf.tube_flags =
                   (DMA_CFG_DST_INC |
                    DMA_CFG_ERR_IE  |
                    DMA_CFG_CMPLT_IE);
    dmaRxConf.target_data = NULL;

    this->usart_device->regs->CR3 |= USART_CR3_DMAR;
    dma_init(DMA1);
}

void HardwareSerial::dma_tx_wait(void) {
    if(dmaTx) dmaTx->wait();
}
void HardwareSerial::dma_rx_wait(void) {
    if(dmaRx) dmaRx->wait();
}

void HardwareSerial::flush(void) {
    usart_reset_rx(this->usart_device);
}
