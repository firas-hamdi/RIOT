#ifndef CANDEV_SAMD5X_H
#define CANDEV_SAMD5X_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CAN_INST_NUM)
#include "can/candev.h"

#ifndef CANDEV_SAMD5X_DEFAULT_BITRATE
/** Default bitrate */
#define CANDEV_SAMD5X_DEFAULT_BITRATE 500000U
#endif

#ifndef CANDEV_SAMD5X_DEFAULT_SPT
/** Default sampling-point */
#define CANDEV_SAMD5X_DEFAULT_SPT     875
#endif

#ifndef CANDEV_SAMD5X_DEFAULT_STD_FILTER_NUM
#define CANDEV_SAMD5X_DEFAULT_STD_FILTER_NUM	3
#endif

#ifndef CANDEV_SAMD5X_DEFAULT_EXT_FILTER_NUM
#define CANDEV_SAMD5X_DEFAULT_EXT_FILTER_NUM	3
#endif

#ifndef CANDEV_SAMD5X_DEFAULT_RX_FIFO_0_ELTS_NUM
#define CANDEV_SAMD5X_DEFAULT_RX_FIFO_0_ELTS_NUM	32
#endif

#ifndef CANDEV_SAMD5X_DEFAULT_RX_FIFO_1_ELTS_NUM
#define CANDEV_SAMD5X_DEFAULT_RX_FIFO_1_ELTS_NUM	32
#endif

#ifndef CANDEV_SAMD5X_DEFAULT_TX_EVT_FIFO_ELTS_NUM
#define CANDEV_SAMD5X_DEFAULT_TX_EVT_FIFO_ELTS_NUM	16
#endif

#ifndef CANDEV_SAMD5X_DEFAULT_TX_BUFFER_NUM
#define CANDEV_SAMD5X_DEFAULT_TX_BUFFER_NUM			16
#endif

#ifndef CANDEV_SAMD5X_DEFAULT_TX_BUFFER_FIFO_QUEUE_NUM
#define CANDEV_SAMD5X_DEFAULT_TX_BUFFER_FIFO_QUEUE_NUM		16
#endif

/* unit: elements */
#define CANDEV_SAMD5X_MAX_STD_FILTER		128
#define CANDEV_SAMD5X_MAX_EXT_FILTER		64
#define CANDEV_SAMD5X_MAX_RX_FIFO_0_ELTS	64
#define CANDEV_SAMD5X_MAX_RX_FIFO_1_ELTS	64
#define CANDEV_SAMD5X_MAX_RX_BUFFER			64
#define CANDEV_SAMD5X_MAX_TX_EVT_FIFO_ELTS	32
#define CANDEV_SAMD5X_MAX_TX_BUFFER			32
#define CANDEV_SAMD5X_MSG_RAM_MAX_SIZE		448

/**
 * @brief CAN device configuration descriptor
 */
typedef struct {
    Can *can;                                       /**< CAN device handler */
    gpio_t rx_pin;                                  /**< CAN Rx pin */
    gpio_t tx_pin;                                  /**< CAN Tx pin */
    uint8_t gclk_src;                               /**< GCLK source supplying the CAN controller */
} can_conf_t;
#define HAVE_CAN_CONF_T

/**
 * @brief CAN message RAM accessible to the CAN controller
 */
typedef struct {
    /** Standard filters space in the CAN message RAM */
    CanMramSidfe std_filter[CANDEV_SAMD5X_DEFAULT_STD_FILTER_NUM];
    /** Extended filters space in the CAN message RAM */
    CanMramXifde ext_filter[CANDEV_SAMD5X_DEFAULT_EXT_FILTER_NUM];
    /** Reception FIFO_0 space in the CAN message RAM */
    CanMramRxf0e rx_fifo_0[CANDEV_SAMD5X_DEFAULT_RX_FIFO_0_ELTS_NUM];
    /** Reception FIFO_1 space in the CAN message RAM */
    CanMramRxf1e rx_fifo_1[CANDEV_SAMD5X_DEFAULT_RX_FIFO_1_ELTS_NUM];
    /** Reception buffers space in the CAN message RAM */
    CanMramRxbe rx_buffer[CANDEV_SAMD5X_MAX_RX_BUFFER];
    /** Transmission events FIFO space in the CAN message RAM */
    CanMramTxefe tx_event_fifo[CANDEV_SAMD5X_DEFAULT_TX_EVT_FIFO_ELTS_NUM];
    /** Transmission FIFO space in the CAN message RAM */
    CanMramTxbe tx_fifo_queue[CANDEV_SAMD5X_DEFAULT_TX_BUFFER_FIFO_QUEUE_NUM];
} msg_ram_conf_t;

/**
 * @brief CAN device descriptor
 */
typedef struct {
    /** Structure to hold driver state */
    candev_t candev;
    /** CAN device configuration descriptor */
    const can_conf_t *conf;
    /** CAN message RAM accessible to the CAN controller */
    msg_ram_conf_t msg_ram_conf;
    /** Enable/Disable Transceiver Delay Compensation */
    bool tdc_ctrl;
    /** False to use Tx FIFO operation, True to use Tx Queue operation */
    bool tx_fifo_queue_ctrl;
} can_t;
#define HAVE_CAN_T

/**
 * @brief   Enable/Disable the transmitter delay compensation
 *
 * @param[in] dev   device descriptor
 *
 */
void candev_samd5x_tdc_control(can_t *dev);

/**
 * @brief   Set the device in test mode
 *
 * @param[in] candev   candev driver
 *
 */
void candev_samd5x_enter_test_mode(candev_t *candev);

/**
 * @brief   Enter the device into sleep mode
 *
 * @param[in] candev   candev driver
 *
 */
void candev_samd5x_enter_sleep_mode(candev_t *candev);

/**
 * @brief   Wake up the device from sleep mode
 *
 * @param[in] candev   candev driver
 *
 */
void candev_samd5x_exit_sleep_mode(candev_t *candev);

#endif /* CAN_INST_NUM */
#endif /* CANDEV_SAMD5X_H */
