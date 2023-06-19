/*
 * Copyright (C) 2023 ML!PA Consulting GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_samd5x
 * @{
 *
 * @file
 * @brief       Implementation of the CAN controller driver
 *
 * @author      Firas Hamdi <firas.hamdi@ml-pa.com>
 * @}
 */

#include <assert.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <cpu_conf.h>

#include "candev_samd5x.h"
#include "periph/can.h"
#include "can/device.h"
#include "periph_cpu.h"

#define ENABLE_DEBUG 0
#include "debug.h"

typedef enum {
    MODE_INIT,
    MODE_TEST,
} can_mode_t;

typedef struct __attribute__((packed)) {
    uint8_t put:4;          /** Message Marker Put index */
    uint8_t get:4;          /** Message Marker Get index */
} can_mm_t;

static can_t *_can;

static int _init(candev_t *candev);
static int _send(candev_t *candev, const struct can_frame *frame);
static int _set_filter(candev_t *candev, const struct can_filter *filter);
static int _remove_filter(candev_t *candev, const struct can_filter *filter);
static void _isr(candev_t *candev);

static int _set_mode(Can *can, can_mode_t mode);

static const candev_driver_t candev_samd5x_driver = {
    .init = _init,
    .send = _send,
    .set_filter = _set_filter,
    .remove_filter = _remove_filter,
    .isr = _isr,
};

static const struct can_bittiming_const bittiming_const = {
    .tseg1_min = 1,
    .tseg1_max = 256,
    .tseg2_min = 1,
    .tseg2_max = 128,
    .sjw_max = 1,
    .brp_min = 1,
    .brp_max = 512,
    .brp_inc = 1,
};

static int _power_on(can_t *dev)
{
    if (dev->conf->can == CAN0) {
        DEBUG_PUTS("CAN0 controller is used");
        MCLK->AHBMASK.reg |= MCLK_AHBMASK_CAN0;
    }
    else if (dev->conf->can == CAN1) {
        DEBUG_PUTS("CAN1 controller is used");
        MCLK->AHBMASK.reg |= MCLK_AHBMASK_CAN1;
    }
    else {
        DEBUG_PUTS("Unsupported CAN channel");
        return -1;
    }

    return 0;
}

static void _enter_init_mode(Can *can)
{
    can->CCCR.reg |= CAN_CCCR_INIT;
    while(!(can->CCCR.reg & CAN_CCCR_INIT));
    DEBUG_PUTS("Device in init mode");
}

static void _exit_init_mode(Can *can)
{
    if (can->CCCR.reg & CAN_CCCR_INIT) {
        can->CCCR.reg &= ~CAN_CCCR_INIT;
    }

    while(can->CCCR.reg & CAN_CCCR_INIT);
    DEBUG_PUTS("Device out of init mode");
}

static int _set_mode(Can *can, can_mode_t can_mode)
{
    switch (can_mode) {
        case MODE_INIT:
            _enter_init_mode(can);
            can->CCCR.reg |= CAN_CCCR_CCE;
            break;
        case MODE_TEST:
            DEBUG_PUTS("test mode");
            _enter_init_mode(can);
            can->CCCR.reg |= CAN_CCCR_CCE;
            can->CCCR.reg |= CAN_CCCR_TEST;
            can->TEST.reg |= CAN_TEST_LBCK;
            _exit_init_mode(can);
            break;
        default:
            DEBUG_PUTS("Unsupported mode");
            return -1;
    }

    return 0;
}

static void _setup_clock(can_t *dev)
{
    if (dev->conf->can == CAN0) {
        GCLK->PCHCTRL[CAN0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN(SAM0_GCLK_PERIPH);
    }
    else if (dev->conf->can == CAN1) {
        GCLK->PCHCTRL[CAN1_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN(SAM0_GCLK_PERIPH);
    }
    else {
        DEBUG_PUTS("CAN channel not supported");
    }
}

static void _set_bit_timing(can_t *dev)
{
    assert(dev->candev.bittiming.sjw >= 1);
    assert(dev->candev.bittiming.phase_seg2 >= 1);
    assert(dev->candev.bittiming.phase_seg1 + dev->candev.bittiming.prop_seg >= 1);
    assert(dev->candev.bittiming.brp >= 1);

    DEBUG("bitrate=%" PRIu32 ", sample_point=%" PRIu32 ", brp=%" PRIu32 ", prop_seg=%" PRIu32
          ", phase_seg1=%" PRIu32 ", phase_seg2=%" PRIu32 ", sjw=%" PRIu32 "\n", dev->candev.bittiming.bitrate, dev->candev.bittiming.sample_point,
          dev->candev.bittiming.brp, dev->candev.bittiming.prop_seg, dev->candev.bittiming.phase_seg1, dev->candev.bittiming.phase_seg2, dev->candev.bittiming.sjw);

    /* Set bit timing */
    dev->conf->can->NBTP.reg = (uint32_t)((CAN_NBTP_NTSEG2(dev->candev.bittiming.phase_seg2 - 1))
                            | (CAN_NBTP_NTSEG1(dev->candev.bittiming.phase_seg1 + dev->candev.bittiming.prop_seg - 1))
                            | (CAN_NBTP_NBRP(dev->candev.bittiming.brp - 1))
                            | (CAN_NBTP_NSJW(dev->candev.bittiming.sjw - 1)));
}

static void _set_tx_fifo_data_size(can_t *dev, uint8_t size) {
    assert(size < 0x8);
    dev->conf->can->TXESC.reg |= CAN_TXESC_TBDS(size);
}

static void _set_rx_buffer_data_size(can_t *dev, uint8_t size) {
    assert(size < 0x8);
    dev->conf->can->RXESC.reg |= CAN_RXESC_RBDS(size);
}

static void _set_rx_fifo_0_data_size(can_t *dev, uint8_t size) {
    assert(size < 0x8);
    dev->conf->can->RXESC.reg |= CAN_RXESC_F0DS(size);
}

static void _set_rx_fifo_1_data_size(can_t *dev, uint8_t size) {
    assert(size < 0x8);
    dev->conf->can->RXESC.reg |= CAN_RXESC_F1DS(size);
}

void candev_samd5x_set_pins(can_t *dev)
{
    assert(dev->conf->tx_pin != GPIO_UNDEF);
    assert(dev->conf->rx_pin != GPIO_UNDEF);

    gpio_init(dev->conf->tx_pin, GPIO_OUT);
    gpio_init(dev->conf->rx_pin, GPIO_IN);

    gpio_init_mux(dev->conf->tx_pin, dev->conf->mux);
    gpio_init_mux(dev->conf->rx_pin, dev->conf->mux);
}

void candev_samd5x_enter_sleep_mode(candev_t *candev)
{
    can_t *dev = container_of(candev, can_t, candev);

    dev->conf->can->CCCR.reg |= CAN_CCCR_CSR;
    while(!(dev->conf->can->CCCR.reg & CAN_CCCR_CSA));
    DEBUG_PUTS("Device in sleep mode");
}

void candev_samd5x_exit_sleep_mode(candev_t *candev)
{
    can_t *dev = container_of(candev, can_t, candev);

    dev->conf->can->CCCR.reg &= ~CAN_CCCR_CSR;
    while(dev->conf->can->CCCR.reg & CAN_CCCR_CSA);
    DEBUG_PUTS("Device out of sleep mode");
}

void candev_samd5x_tdc_control(can_t *dev)
{
    if(dev->conf->tdc_ctrl) {
        DEBUG_PUTS("Enable Transceiver Delay Compensation");
        dev->conf->can->DBTP.reg |= CAN_DBTP_TDC;
    }
    else {
        DEBUG_PUTS("Disable Transceiver Delay Compensation");
        dev->conf->can->DBTP.reg &= ~(CAN_DBTP_TDC);
    }
}

void candev_samd5x_enter_test_mode(candev_t *candev)
{
    can_t *dev = container_of(candev, can_t, candev);

    _set_mode(dev->conf->can, MODE_TEST);
}

void can_init(can_t *dev, const can_conf_t *conf)
{
    dev->candev.driver = &candev_samd5x_driver;

    struct can_bittiming timing = { .bitrate = CANDEV_SAMD5X_DEFAULT_BITRATE,
                                    .sample_point = CANDEV_SAMD5X_DEFAULT_SPT };

    uint32_t clk_freq = sam0_gclk_freq(SAM0_GCLK_PERIPH);
    can_device_calc_bittiming(clk_freq, &bittiming_const, &timing);

    memcpy(&dev->candev.bittiming, &timing, sizeof(timing));
    dev->conf = conf;
}

static int _init(candev_t *candev)
{
    can_t *dev = container_of(candev, can_t, candev);
    int res = 0;

    sam0_gclk_enable(SAM0_GCLK_PERIPH);

    _setup_clock(dev);
    _power_on(dev);

    candev_samd5x_set_pins(dev);

    res = _set_mode(dev->conf->can, MODE_INIT);

    _set_bit_timing(dev);

    candev_samd5x_tdc_control(dev);

    /*Configure the start addresses of the RAM message sections */
    dev->conf->can->SIDFC.reg = CAN_SIDFC_FLSSA((uint32_t)(dev->msg_ram_conf.std_filter)) | CAN_SIDFC_LSS((uint32_t)(ARRAY_SIZE(dev->msg_ram_conf.std_filter)));
    dev->conf->can->XIDFC.reg = CAN_XIDFC_FLESA((uint32_t)(dev->msg_ram_conf.ext_filter)) | CAN_XIDFC_LSE((uint32_t)(ARRAY_SIZE(dev->msg_ram_conf.ext_filter)));
    dev->conf->can->RXF0C.reg = CAN_RXF0C_F0SA((uint32_t)(dev->msg_ram_conf.rx_fifo_0)) | CAN_RXF0C_F0S((uint32_t)(ARRAY_SIZE(dev->msg_ram_conf.rx_fifo_0)));
    dev->conf->can->RXF1C.reg = CAN_RXF1C_F1SA((uint32_t)(dev->msg_ram_conf.rx_fifo_1)) | CAN_RXF1C_F1S((uint32_t)(ARRAY_SIZE(dev->msg_ram_conf.rx_fifo_1)));
    dev->conf->can->RXBC.reg = CAN_RXBC_RBSA((uint32_t)(dev->msg_ram_conf.rx_buffer));
    dev->conf->can->TXEFC.reg = CAN_TXEFC_EFSA((uint32_t)(dev->msg_ram_conf.tx_event_fifo)) | CAN_TXEFC_EFS((uint32_t)(ARRAY_SIZE(dev->msg_ram_conf.tx_event_fifo)));
    dev->conf->can->TXBC.reg = CAN_TXBC_TBSA((uint32_t)(dev->msg_ram_conf.tx_fifo_queue)) | CAN_TXBC_TFQS((uint32_t)(ARRAY_SIZE(dev->msg_ram_conf.tx_fifo_queue)));

    /* In the vendor file, the data field size in CanMramTxbe is set to 64 bytes although it can be configurable. That's why 64 bytes is used here by default */
    _set_tx_fifo_data_size(dev, CAN_RXESC_F1DS_DATA64_Val);

    /* In the vendor file, the data field size in CanMramRxbe is set to 64 bytes although it can be configurable. That's why 64 bytes is used here by default */
    _set_rx_buffer_data_size(dev, CAN_RXESC_RBDS_DATA64_Val);
    /* In the vendor file, the data field size in CanMramRxf0e is set to 64 bytes although it can be configurable. That's why 64 bytes is used here by default */
    _set_rx_fifo_0_data_size(dev, CAN_RXESC_F0DS_DATA64_Val);
    /* In the vendor file, the data field size in CanMramRxf1e is set to 64 bytes although it can be configurable. That's why 64 bytes is used here by default */
    _set_rx_fifo_1_data_size(dev, CAN_RXESC_F1DS_DATA64_Val);

    dev->conf->can->CCCR.reg |= CAN_CCCR_DAR;

    /* Global configuration of filters */
    dev->conf->can->GFC.reg = CAN_GFC_ANFE((uint32_t)dev->conf->global_filter_cfg) | CAN_GFC_ANFS((uint32_t)dev->conf->global_filter_cfg);
    /* Reject all remote frames */
    dev->conf->can->GFC.reg |= CAN_GFC_RRFE | CAN_GFC_RRFS;

    /* Control FIFO/queue op */

    /* Enable the peripheral's interrupt */
    if (dev->conf->can == CAN0) {
        NVIC_EnableIRQ(CAN0_IRQn);
    }
    else {
        NVIC_EnableIRQ(CAN1_IRQn);
    }

    dev->conf->can->IE.reg |= CAN_IE_RF0NE | CAN_IE_RF1NE;
    dev->conf->can->IE.reg |= CAN_IE_TEFNE;
    /* Enable the interrupt lines */
    dev->conf->can->ILE.reg = CAN_ILE_EINT0 | CAN_ILE_EINT1;

    _can = dev;

    /* Exit initialization mode */
    _exit_init_mode(dev->conf->can);

    return res;
}

static uint8_t _form_message_marker(can_mm_t *can_mm)
{
    return can_mm->put | (can_mm->get << 4);
}

static int _send(candev_t *candev, const struct can_frame *frame)
{
    can_t *dev = container_of(candev, can_t, candev);

    if (frame->can_dlc > CAN_MAX_DLEN) {
        DEBUG_PUTS("CAN frame payload not supported");
        return -1;
    }

    /* Check if the Tx FIFO is full */
    if (dev->conf->can->TXFQS.reg & CAN_TXFQS_TFQF) {
        DEBUG_PUTS("Tx FIFO is full");
        return -1;
    }

    can_mm_t can_mm = {0};
    uint8_t fifo_queue_put_idx = (dev->conf->can->TXFQS.reg >> CAN_TXFQS_TFQPI_Pos) & 0x1F;
    DEBUG("Tx FIFO put index = %u\n", fifo_queue_put_idx);
    uint8_t fifo_queue_get_idx = (dev->conf->can->TXFQS.reg >> CAN_TXFQS_TFGI_Pos) & 0x1F;
    DEBUG("Tx FIFO get index = %u\n", fifo_queue_get_idx);
    can_mm.put = fifo_queue_put_idx;
    can_mm.get = fifo_queue_get_idx;

    if (frame->can_id & CAN_EFF_FLAG) {
        DEBUG_PUTS("Extended ID");
        dev->msg_ram_conf.tx_fifo_queue[can_mm.put].TXBE_0.bit.ID = frame->can_id & CAN_EFF_MASK;
        dev->msg_ram_conf.tx_fifo_queue[can_mm.put].TXBE_0.bit.XTD = 1;
    }
    else {
        DEBUG_PUTS("Standard identifier");
        dev->msg_ram_conf.tx_fifo_queue[can_mm.put].TXBE_0.bit.ID = (frame->can_id & CAN_SFF_MASK) << 18;
        dev->msg_ram_conf.tx_fifo_queue[can_mm.put].TXBE_0.bit.XTD = 0;
    }
    dev->msg_ram_conf.tx_fifo_queue[can_mm.put].TXBE_0.bit.RTR = (frame->can_id & CAN_RTR_FLAG) >> 30;
    dev->msg_ram_conf.tx_fifo_queue[can_mm.put].TXBE_0.bit.ESI = 1;
    dev->msg_ram_conf.tx_fifo_queue[can_mm.put].TXBE_1.bit.DLC = frame->can_dlc;
    dev->msg_ram_conf.tx_fifo_queue[can_mm.put].TXBE_1.bit.EFC = 1;
    dev->msg_ram_conf.tx_fifo_queue[can_mm.put].TXBE_1.bit.MM = _form_message_marker(&can_mm);
    memcpy((void *)dev->msg_ram_conf.tx_fifo_queue[can_mm.put].TXBE_DATA, frame->data, frame->can_dlc);

    /* Request transmission */
    dev->conf->can->TXBAR.reg |= (1 << can_mm.put);

    return 0;
}

static bool _find_filter(can_t *can, const struct can_filter *filter, bool is_std_filter, int16_t *idx)
{
    if (is_std_filter) {
        for (uint8_t i = 0; i < ARRAY_SIZE(can->msg_ram_conf.std_filter); i++) {
            if (((filter->can_id & CAN_SFF_MASK) == can->msg_ram_conf.std_filter[i].SIDFE_0.bit.SFID1)) {
                *idx = i;
                return true;
            }
        }
    }
    else {
        for (uint8_t i = 0; i < ARRAY_SIZE(can->msg_ram_conf.ext_filter); i++) {
            if (((filter->can_id & CAN_EFF_MASK) == can->msg_ram_conf.ext_filter[i].XIDFE_0.bit.EFID1)) {
                *idx = i;
                return true;
            }
        }
    }

    return false;
}

static int _set_filter(candev_t *candev, const struct can_filter *filter)
{
    can_t *dev = container_of(candev, can_t, candev);

    int16_t idx = 0;
    bool _filter_exists = false;
    if (filter->can_id & CAN_EFF_FLAG) {
        DEBUG_PUTS("Extended filter to add in the extended filter section of the message RAM");
        /* Check if the filter already exists */
        _filter_exists = _find_filter(dev, filter, false, &idx);
        if (_filter_exists) {
            DEBUG_PUTS("Extended filter already exists --> Update it");
        }
        else {
            /* Find a free slot where to save the filter */
            for (;(uint16_t)idx < ARRAY_SIZE(dev->msg_ram_conf.ext_filter); idx++) {
                if (dev->msg_ram_conf.ext_filter[idx].XIDFE_0.bit.EFEC == CAN_FILTER_DISABLE) {
                    DEBUG_PUTS("empty slot");
                    break;
                }
            }
        }

        if (idx == ARRAY_SIZE(dev->msg_ram_conf.ext_filter)) {
            DEBUG_PUTS("Reached maximum capacity of extended filters --> Could not add filter");
            return -1;
        }

        DEBUG("Filter to add at idx = %d\n", idx);
        dev->msg_ram_conf.ext_filter[idx].XIDFE_0.bit.EFID1 = filter->can_id;
        dev->msg_ram_conf.ext_filter[idx].XIDFE_0.bit.EFEC = filter->can_filter_conf;
        dev->msg_ram_conf.ext_filter[idx].XIDFE_1.bit.EFID2 = filter->can_mask & CAN_EFF_MASK;
        dev->msg_ram_conf.ext_filter[idx].XIDFE_1.bit.EFT = filter->can_filter_type & CAN_EFF_MASK;
    }
    else {
        DEBUG_PUTS("Standard filter to add in the standard filter section of the message RAM");
        /* Check if the filter already exists */
        _filter_exists = _find_filter(dev, filter, true, &idx);
        if (_filter_exists) {
            DEBUG_PUTS("Standard filter already exists --> Update it");
        }
        else {
            /* Find a free slot where to save the filter */
            for (; (uint16_t)idx < ARRAY_SIZE(dev->msg_ram_conf.std_filter); idx++) {
                /* Find a free slot where to save the filter */
                if (dev->msg_ram_conf.std_filter[idx].SIDFE_0.bit.SFEC == CAN_FILTER_DISABLE) {
                    DEBUG_PUTS("empty slot");
                    break;
                }
            }
        }

        if (idx == ARRAY_SIZE(dev->msg_ram_conf.std_filter)) {
            DEBUG_PUTS("Reached maximum capacity of standard filters --> Could not add filter");
            return -1;
        }

        DEBUG("Filter to add at idx = %d\n", idx);
        dev->msg_ram_conf.std_filter[idx].SIDFE_0.bit.SFEC = filter->can_filter_conf;
        dev->msg_ram_conf.std_filter[idx].SIDFE_0.bit.SFT = filter->can_filter_type;
        dev->msg_ram_conf.std_filter[idx].SIDFE_0.bit.SFID2 = filter->can_mask & CAN_SFF_MASK;
        dev->msg_ram_conf.std_filter[idx].SIDFE_0.bit.SFID1 = filter->can_id & CAN_SFF_MASK;
    }

    return idx;
}

static int _remove_filter(candev_t *candev, const struct can_filter *filter)
{
    can_t *dev = container_of(candev, can_t, candev);

    int16_t idx = 0;
    bool _filter_exists = false;
    if (filter->can_id & CAN_EFF_FLAG) {
        _filter_exists = _find_filter(dev, filter, false, &idx);
        if (_filter_exists) {
            DEBUG("Extended filter to disable at idx = %d\n", idx);
            dev->msg_ram_conf.ext_filter[idx].XIDFE_0.bit.EFEC = CAN_FILTER_DISABLE;

            for (uint8_t i = 0; i < ARRAY_SIZE(dev->msg_ram_conf.ext_filter); i++) {
                DEBUG("can->msg_ram_conf.std_filter[%u] = 0x%08lx, filter conf = %u\n", i,
                        (uint32_t)(dev->msg_ram_conf.ext_filter[i].XIDFE_0.bit.EFID1), dev->msg_ram_conf.ext_filter[i].XIDFE_0.bit.EFEC);
            }
        }
        else {
            DEBUG_PUTS("Filter not found");
            return -1;
        }
    }
    else {
        _filter_exists = _find_filter(dev, filter, true, &idx);
        if(_filter_exists) {
            DEBUG("Standard filter to disable at idx = %d\n", idx);
            dev->msg_ram_conf.std_filter[idx].SIDFE_0.bit.SFEC = CAN_FILTER_DISABLE;
        }
        else {
            DEBUG_PUTS("Filter not found");
            return -1;
        }
    }

    return idx;
}

static void _isr(candev_t *candev)
{
    can_t *dev = container_of(candev, can_t, candev);

    uint32_t irq_reg = dev->conf->can->IR.reg;
    if (irq_reg & CAN_IR_TSW) {
        DEBUG_PUTS("Timestamp wraparound interrupt");
        /* Clear the interrupt source flag */
        dev->conf->can->IR.reg |= CAN_IR_TSW;
    }

    if (irq_reg & CAN_IR_RF0N) {
        DEBUG_PUTS("New message in Rx FIFO 0");
        /* Clear the interrupt source flag */
        dev->conf->can->IR.reg |= CAN_IR_RF0N;

        uint16_t rx_get_idx = 0;
        uint16_t rx_put_idx = 0;
        rx_get_idx = (dev->conf->can->RXF0S.reg >> CAN_RXF0S_F0GI_Pos) & 0x3F;
        DEBUG("rx get index = %u\n", rx_get_idx);
        rx_put_idx = (dev->conf->can->RXF0S.reg >> CAN_RXF0S_F0PI_Pos) & 0x3F;
        DEBUG("rx put index = %u\n", rx_put_idx);

        struct can_frame frame_received = {0};
        if (!dev->msg_ram_conf.rx_fifo_0[rx_get_idx].RXF0E_0.bit.XTD) {
            DEBUG_PUTS("Received standard CAN frame");
            frame_received.can_id = dev->msg_ram_conf.rx_fifo_0[rx_get_idx].RXF0E_0.bit.ID >> 18;
        }
        else {
            DEBUG_PUTS("Received extended CAN frame");
            frame_received.can_id = dev->msg_ram_conf.rx_fifo_0[rx_get_idx].RXF0E_0.bit.ID;
        }
        frame_received.can_dlc = dev->msg_ram_conf.rx_fifo_0[rx_get_idx].RXF0E_1.bit.DLC;
        memcpy(frame_received.data, (uint32_t *)dev->msg_ram_conf.rx_fifo_0[rx_get_idx].RXF0E_DATA, frame_received.can_dlc);

        dev->conf->can->RXF0A.reg = CAN_RXF0A_F0AI(rx_get_idx);
        if (dev->candev.event_callback) {
            dev->candev.event_callback(&(dev->candev), CANDEV_EVENT_RX_INDICATION, &frame_received);
        }
    }

    if (irq_reg & CAN_IR_RF1N) {
        DEBUG_PUTS("New message in Rx FIFO 1");
        /* Clear the interrupt source flag */
        dev->conf->can->IR.reg |= CAN_IR_RF1N;

        uint16_t rx_get_idx = 0;
        uint16_t rx_put_idx = 0;
        rx_get_idx = (dev->conf->can->RXF1S.reg >> CAN_RXF1S_F1GI_Pos) & 0x3F;
        DEBUG("rx get index = %u\n", rx_get_idx);
        rx_put_idx = (dev->conf->can->RXF1S.reg >> CAN_RXF1S_F1PI_Pos) & 0x3F;
        DEBUG("rx put index = %u\n", rx_put_idx);

        struct can_frame frame_received = {0};
        if (!dev->msg_ram_conf.rx_fifo_1[rx_get_idx].RXF1E_0.bit.XTD) {
            DEBUG_PUTS("Received standard CAN frame");
            frame_received.can_id = dev->msg_ram_conf.rx_fifo_1[rx_get_idx].RXF1E_0.bit.ID >> 18;
        }
        else {
            DEBUG_PUTS("Received extended CAN frame");
            frame_received.can_id = dev->msg_ram_conf.rx_fifo_1[rx_get_idx].RXF1E_0.bit.ID;
        }
        frame_received.can_dlc = dev->msg_ram_conf.rx_fifo_1[rx_get_idx].RXF1E_1.bit.DLC;
        memcpy(frame_received.data, (uint32_t *)dev->msg_ram_conf.rx_fifo_1[rx_get_idx].RXF1E_DATA, frame_received.can_dlc);

        dev->conf->can->RXF1A.reg = CAN_RXF1A_F1AI(rx_get_idx);
        if (dev->candev.event_callback) {
            dev->candev.event_callback(&(dev->candev), CANDEV_EVENT_RX_INDICATION, &frame_received);
        }
    }
    if (irq_reg & CAN_IR_TEFN) {
        DEBUG_PUTS("New Tx event FIFO entry");
        dev->conf->can->IR.reg |= CAN_IR_TEFN;
        if (dev->candev.event_callback) {
            dev->candev.event_callback(&(dev->candev), CANDEV_EVENT_TX_CONFIRMATION, NULL);
        }
    }
}

void ISR_CAN1(void)
{
    DEBUG_PUTS("ISR CAN1");

    if (_can->candev.event_callback) {
        _can->candev.event_callback(&(_can->candev), CANDEV_EVENT_ISR, NULL);
    }

}

void ISR_CAN0(void)
{
    DEBUG_PUTS("ISR CAN0");

    if (_can->candev.event_callback) {
        _can->candev.event_callback(&(_can->candev), CANDEV_EVENT_ISR, NULL);
    }
}