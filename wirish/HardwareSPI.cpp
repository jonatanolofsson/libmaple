/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
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
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief Wirish SPI implementation.
 */

#include <wirish/HardwareSPI.h>

#include <libmaple/timer.h>
#include <libmaple/util.h>
#include <libmaple/rcc.h>

#include <wirish/wirish.h>
#include <wirish/boards.h>

#if CYCLES_PER_MICROSECOND != 72
/* TODO [0.2.0?] something smarter than this */
#warning "Unexpected clock speed; SPI frequency calculation will be incorrect"
#endif

struct spi_pins {
    uint8 nss;
    uint8 sck;
    uint8 miso;
    uint8 mosi;
};

static const spi_pins* dev_to_spi_pins(spi_dev *dev);

static void enable_device(spi_dev *dev,
                          bool as_master,
                          SPIFrequency frequency,
                          spi_cfg_flag endianness,
                          spi_mode mode);

static const spi_pins board_spi_pins[] __FLASH__ = {
    {BOARD_SPI1_NSS_PIN,
     BOARD_SPI1_SCK_PIN,
     BOARD_SPI1_MISO_PIN,
     BOARD_SPI1_MOSI_PIN},
    {BOARD_SPI2_NSS_PIN,
     BOARD_SPI2_SCK_PIN,
     BOARD_SPI2_MISO_PIN,
     BOARD_SPI2_MOSI_PIN},
#ifdef STM32_HIGH_DENSITY
    {BOARD_SPI3_NSS_PIN,
     BOARD_SPI3_SCK_PIN,
     BOARD_SPI3_MISO_PIN,
     BOARD_SPI3_MOSI_PIN},
#endif
};


#if BOARD_HAVE_SPI1
HardwareSPI Spi1(1);
#endif
#if BOARD_HAVE_SPI2
HardwareSPI Spi2(2);
#endif
#if BOARD_HAVE_SPI3
HardwareSPI Spi3(3);
#endif

/*
 * Constructor
 */

HardwareSPI::HardwareSPI(uint32 spi_num) {
    switch (spi_num) {
    case 1:
        this->spi_d = SPI1;
        break;
    case 2:
        this->spi_d = SPI2;
        break;
#ifdef STM32_HIGH_DENSITY
    case 3:
        this->spi_d = SPI3;
        break;
#endif
    default:
        ASSERT(0);
    }
}

/*
 * Set up/tear down
 */

void HardwareSPI::begin(SPIFrequency frequency, uint32 bitOrder, uint32 mode) {
    if (mode >= 4) {
        ASSERT(0);
        return;
    }
    spi_cfg_flag end = bitOrder == MSBFIRST ? SPI_FRAME_MSB : SPI_FRAME_LSB;
    spi_mode m = (spi_mode)mode;
    enable_device(this->spi_d, true, frequency, end, m);
}

void HardwareSPI::begin(void) {
    this->begin(SPI_1_125MHZ, MSBFIRST, 0);
}

void HardwareSPI::beginSlave(uint32 bitOrder, uint32 mode) {
    if (mode >= 4) {
        ASSERT(0);
        return;
    }
    spi_cfg_flag end = bitOrder == MSBFIRST ? SPI_FRAME_MSB : SPI_FRAME_LSB;
    spi_mode m = (spi_mode)mode;
    enable_device(this->spi_d, false, (SPIFrequency)0, end, m);
}

void HardwareSPI::beginSlave(void) {
    this->beginSlave(MSBFIRST, 0);
}

void HardwareSPI::end(void) {
    if (!spi_is_enabled(this->spi_d)) {
        return;
    }

    // Follows RM0008's sequence for disabling a SPI in master/slave
    // full duplex mode.
    while (spi_is_rx_nonempty(this->spi_d)) {
        // FIXME [0.1.0] remove this once you have an interrupt based driver
        volatile uint16 rx __attribute__((unused)) = spi_rx_reg(this->spi_d);
    }
    while (!spi_is_tx_empty(this->spi_d))
        ;
    while (spi_is_busy(this->spi_d))
        ;
    spi_peripheral_disable(this->spi_d);
}

/*
 * I/O
 */

uint8 HardwareSPI::read(void) {
    uint8 buf[1];
    this->read(buf, 1);
    return buf[0];
}

void HardwareSPI::read(uint8 *buf, uint32 len) {
    uint32 rxed = 0;
    while (rxed < len) {
        while (!spi_is_rx_nonempty(this->spi_d))
            ;
        buf[rxed++] = (uint8)spi_rx_reg(this->spi_d);
    }
}

void HardwareSPI::write(uint8 byte) {
    this->write(&byte, 1);
}

void HardwareSPI::write(const uint8 *data, uint32 length) {
    uint32 txed = 0;
    while (txed < length) {
        txed += spi_tx(this->spi_d, data + txed, length - txed);
    }
}

uint8 HardwareSPI::transfer(uint8 byte) {
    this->write(byte);
    return this->read();
}

void HardwareSPI::write(dma_message& newMsg) {
    dma_tx_wait();
    newMsg.claim();
    dmaTx = &newMsg;
    dmaTxConf.tube_src = (volatile char*)(dmaTx->data);
    dmaTxConf.tube_nr_xfers = dmaTx->length;
    int status = dma_tube_cfg(this->spi_d->dma_device, this->spi_d->dma_tx_tube, &dmaTxConf);
    ASSERT(status == DMA_TUBE_CFG_SUCCESS);
    dma_enable(this->spi_d->dma_device, this->spi_d->dma_tx_tube);
}

int HardwareSPI::read(dma_message& newMsg) {
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

    int status = dma_tube_cfg(this->spi_d->dma_device, this->spi_d->dma_rx_tube, &dmaRxConf);
    ASSERT(status == DMA_TUBE_CFG_SUCCESS);
    dma_enable(this->spi_d->dma_device, this->spi_d->dma_rx_tube);
    return 0;
}

void HardwareSPI::irq_dma_tx(void) {
    dma_irq_cause cause = dma_get_irq_cause(this->spi_d->dma_device, this->spi_d->dma_tx_tube);
    dma_disable(this->spi_d->dma_device, this->spi_d->dma_tx_tube);
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

void irq_spi_dma_tx_1(void) {Spi1.irq_dma_tx();}
void irq_spi_dma_tx_2(void) {Spi2.irq_dma_tx();}
#if BOARD_HAVE_SPI3
void irq_spi_dma_tx_3(void) {Spi3.irq_dma_tx();}
#endif

void HardwareSPI::irq_dma_rx(void) {
    dma_irq_cause cause = dma_get_irq_cause(this->spi_d->dma_device, this->spi_d->dma_rx_tube);
    dma_disable(this->spi_d->dma_device, this->spi_d->dma_rx_tube);
    if(DMA_TRANSFER_COMPLETE == cause) {
        receivedBytes = dmaRxConf.tube_nr_xfers;
    }
    if(dmaRx->callback) {
        (*(dmaRx->callback))(dmaRx, cause);
    }
    dmaRx = NULL;
}

void irq_spi_dma_rx_1(void) {Spi1.irq_dma_rx();}
void irq_spi_dma_rx_2(void) {Spi2.irq_dma_rx();}
#if BOARD_HAVE_SPI3
void irq_spi_dma_rx_3(void) {Spi3.irq_dma_rx();}
#endif

void HardwareSPI::setup_dma_tx(void) {
    void(*irq)(void);
    #if BOARD_HAVE_SPI1
    if(this == &Spi1) {
        irq = irq_spi_dma_tx_1;
        dmaRxConf.tube_req_src = DMA_REQ_SRC_SPI1_TX;
    }
    #endif
    #if BOARD_HAVE_SPI2
    else if(this == &Spi2) {
        irq = irq_spi_dma_tx_2;
        dmaRxConf.tube_req_src = DMA_REQ_SRC_SPI2_TX;
    }
    #endif
    #if BOARD_HAVE_SPI3
    else if(this == &Spi3) {
        irq = irq_spi_dma_tx_3;
        dmaRxConf.tube_req_src = DMA_REQ_SRC_SPI3_TX;
    }
    #endif
    else return;

    dma_attach_interrupt(this->spi_d->dma_device, this->spi_d->dma_tx_tube, irq);

    dmaTxConf.tube_dst = &(this->spi_d->regs->DR);
    dmaTxConf.tube_src_size = DMA_SIZE_8BITS;
    dmaTxConf.tube_dst_size = DMA_SIZE_8BITS;

    dmaTxConf.tube_flags =
                   (DMA_CFG_SRC_INC |
                    DMA_CFG_ERR_IE  |
                    DMA_CFG_CMPLT_IE);
    dmaTxConf.target_data = NULL;

    //~ this->spi_d->regs->CR3 |= SPI_CR3_DMAT;
    dma_init(DMA1);
}

void HardwareSPI::setup_dma_rx(void) {
    void(*irq)(void);
    #if BOARD_HAVE_SPI1
    if(this == &Spi1) {
        irq = irq_spi_dma_rx_1;
        dmaRxConf.tube_req_src = DMA_REQ_SRC_SPI1_RX;
    }
    #endif
    #if BOARD_HAVE_SPI2
    else if(this == &Spi2) {
        irq = irq_spi_dma_rx_2;
        dmaRxConf.tube_req_src = DMA_REQ_SRC_SPI2_RX;
    }
    #endif
    #if BOARD_HAVE_SPI3
    else if(this == &Spi3) {
        irq = irq_spi_dma_rx_3;
        dmaRxConf.tube_req_src = DMA_REQ_SRC_SPI3_RX;
    }
    #endif
    else return;

    dma_attach_interrupt(this->spi_d->dma_device, this->spi_d->dma_rx_tube, irq);

    dmaRxConf.tube_src = &(this->spi_d->regs->DR);
    dmaRxConf.tube_src_size = DMA_SIZE_8BITS;
    dmaRxConf.tube_dst_size = DMA_SIZE_8BITS;

    dmaRxConf.tube_flags =
                   (DMA_CFG_DST_INC |
                    DMA_CFG_ERR_IE  |
                    DMA_CFG_CMPLT_IE);
    dmaRxConf.target_data = NULL;

    //~ this->spi_d->regs->CR3 |= SPI_CR3_DMAR;
    dma_init(DMA1);
}
void HardwareSPI::dma_tx_wait(void) {
    if(dmaTx) dmaTx->wait();
}
void HardwareSPI::dma_rx_wait(void) {
    if(dmaRx) dmaRx->wait();
}

/*
 * Pin accessors
 */

uint8 HardwareSPI::misoPin(void) {
    return dev_to_spi_pins(this->spi_d)->miso;
}

uint8 HardwareSPI::mosiPin(void) {
    return dev_to_spi_pins(this->spi_d)->mosi;
}

uint8 HardwareSPI::sckPin(void) {
    return dev_to_spi_pins(this->spi_d)->sck;
}

uint8 HardwareSPI::nssPin(void) {
    return dev_to_spi_pins(this->spi_d)->nss;
}

/*
 * Deprecated functions
 */

uint8 HardwareSPI::send(uint8 data) {
    uint8 buf[] = {data};
    return this->send(buf, 1);
}

uint8 HardwareSPI::send(uint8 *buf, uint32 len) {
    uint32 txed = 0;
    uint8 ret = 0;
    while (txed < len) {
        this->write(buf[txed++]);
        ret = this->read();
    }
    return ret;
}

uint8 HardwareSPI::recv(void) {
    return this->read();
}

/*
 * Auxiliary functions
 */

static void configure_gpios(spi_dev *dev, bool as_master);
static spi_baud_rate determine_baud_rate(spi_dev *dev, SPIFrequency freq);

static const spi_pins* dev_to_spi_pins(spi_dev *dev) {
    switch (dev->clk_id) {
    case RCC_SPI1: return board_spi_pins;
    case RCC_SPI2: return board_spi_pins + 1;
#ifdef STM32_HIGH_DENSITY
    case RCC_SPI3: return board_spi_pins + 2;
#endif
    default:       return NULL;
    }
}

/* Enables the device in master or slave full duplex mode.  If you
 * change this code, you must ensure that appropriate changes are made
 * to HardwareSPI::end(). */
static void enable_device(spi_dev *dev,
                          bool as_master,
                          SPIFrequency freq,
                          spi_cfg_flag endianness,
                          spi_mode mode) {
    spi_baud_rate baud = determine_baud_rate(dev, freq);
    uint32 cfg_flags = (endianness | SPI_DFF_8_BIT | SPI_SW_SLAVE |
                        (as_master ? SPI_SOFT_SS : 0));

    spi_init(dev);
    configure_gpios(dev, as_master);
    if (as_master) {
        spi_master_enable(dev, baud, mode, cfg_flags);
    } else {
        spi_slave_enable(dev, mode, cfg_flags);
    }
}

static void disable_pwm(const stm32_pin_info *i) {
    if (i->timer_device) {
        timer_set_mode(i->timer_device, i->timer_channel, TIMER_DISABLED);
    }
}

static void configure_gpios(spi_dev *dev, bool as_master) {
    const spi_pins *pins = dev_to_spi_pins(dev);

    if (!pins) {
        return;
    }

    const stm32_pin_info *nssi = &PIN_MAP[pins->nss];
    const stm32_pin_info *scki = &PIN_MAP[pins->sck];
    const stm32_pin_info *misoi = &PIN_MAP[pins->miso];
    const stm32_pin_info *mosii = &PIN_MAP[pins->mosi];

    disable_pwm(nssi);
    disable_pwm(scki);
    disable_pwm(misoi);
    disable_pwm(mosii);

    spi_config_gpios(dev, as_master, nssi->gpio_device, nssi->gpio_bit,
                     scki->gpio_device, scki->gpio_bit, misoi->gpio_bit,
                     mosii->gpio_bit);
}

static const spi_baud_rate baud_rates[MAX_SPI_FREQS] __FLASH__ = {
    SPI_BAUD_PCLK_DIV_2,
    SPI_BAUD_PCLK_DIV_4,
    SPI_BAUD_PCLK_DIV_8,
    SPI_BAUD_PCLK_DIV_16,
    SPI_BAUD_PCLK_DIV_32,
    SPI_BAUD_PCLK_DIV_64,
    SPI_BAUD_PCLK_DIV_128,
    SPI_BAUD_PCLK_DIV_256,
};

/*
 * Note: This assumes you're on a LeafLabs-style board
 * (CYCLES_PER_MICROSECOND == 72, APB2 at 72MHz, APB1 at 36MHz).
 */
static spi_baud_rate determine_baud_rate(spi_dev *dev, SPIFrequency freq) {
    if (rcc_dev_clk(dev->clk_id) == RCC_APB2 && freq == SPI_140_625KHZ) {
        /* APB2 peripherals are too fast for 140.625 KHz */
        ASSERT(0);
        return (spi_baud_rate)~0;
    }
    return (rcc_dev_clk(dev->clk_id) == RCC_APB2 ?
            baud_rates[freq + 1] :
            baud_rates[freq]);
}
