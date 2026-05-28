/**
 * @file BNO08xSH2HAL.cpp
 * @author Myles Parfeniuk
 */

#include "BNO08xSH2HAL.hpp"
#include "BNO08x.hpp"
#include <string.h>

BNO08x* BNO08xSH2HAL::imu;
uint8_t BNO08xSH2HAL::pending_rx[SH2_HAL_DMA_SIZE];
int BNO08xSH2HAL::pending_rx_len = 0;

static int parse_valid_packet_len(const uint8_t* buffer, unsigned max_len)
{
    if ((buffer == nullptr) || (max_len < 4U))
        return 0;

    uint16_t packet_sz = PARSE_PACKET_LENGTH(buffer);
    packet_sz &= ~0x8000U;

    if ((packet_sz < 4U) || (packet_sz > max_len))
        return 0;

    return static_cast<int>(packet_sz);
}

/**
 * @brief Sets the BNO08x driver object to be used with sh2 HAL lib callbacks.
 *
 * @param hal_imu Pointer to BNO08x driver object to be used with sh2 HAL lib callbacks.
 *
 * @return void, nothing to return
 */
void BNO08xSH2HAL::set_hal_imu(BNO08x* hal_imu)
{
    imu = hal_imu;
    pending_rx_len = 0;
}

/**
 * @brief Opens SPI instance by waiting for interrupt.
 *
 * @return Always returns 0.
 */
int BNO08xSH2HAL::spi_open(sh2_Hal_t* self)
{
    spi_wait_for_int();

    return 0;
}

/**
 * @brief Closes SPI instance (nothing to do here, but required by sh2 HAL lib for cases where other
 * communication protocols are used.)
 *
 * @return void, nothing to return
 */
void BNO08xSH2HAL::spi_close(sh2_Hal_t* self)
{
    // do nothing
}

/**
 * @brief SPI rx callback for sh2 HAL lib.
 *
 * @param self sh2 HAL lib object being used with BNO08x driver instance.
 * @param pBuffer Buffer to store received packet.
 * @param len Length of bytes to read.
 * @param t_us Time in microseconds (unused, forced sh2 HAL lib required function type)
 *
 * @return Size of received packet in bytes, 0 on failure.
 */
int BNO08xSH2HAL::spi_read(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len, uint32_t* t_us)
{
    if ((pBuffer == nullptr) || (len < 4U))
        return 0;

    if (pending_rx_len > 0)
    {
        const int packet_len = pending_rx_len;
        pending_rx_len = 0;

        if (static_cast<unsigned>(packet_len) <= len)
        {
            memcpy(pBuffer, pending_rx, packet_len);
            return packet_len;
        }
    }

    const bool init_phase = (imu->product_IDs.numEntries == 0U) && !imu->init_status.data_proc_task;

    // During startup the BNO085 may need host clocks even when HINT is already high.
    // Runtime reads are still gated by HINT to avoid stimulating extra channel-0 packets.
    if (!init_phase && !spi_wait_for_int())
        return 0;

    static uint8_t tx_dummy[SH2_HAL_DMA_SIZE];
    const unsigned transfer_len = (len > sizeof(tx_dummy)) ? sizeof(tx_dummy) : len;
    memset(tx_dummy, 0x00, transfer_len);

    if (init_phase)
        tx_dummy[0] = 0x04U; // valid empty SHTP packet: len=4, channel=0, seq=0

    imu->spi_transaction.length = transfer_len * 8U;
    imu->spi_transaction.rxlength = transfer_len * 8U;
    imu->spi_transaction.tx_buffer = tx_dummy;
    imu->spi_transaction.rx_buffer = pBuffer;
    imu->spi_transaction.flags = 0;

    gpio_set_level(imu->imu_config.io_cs, 0);
    esp_err_t ret = spi_device_polling_transmit(imu->spi_hdl, &imu->spi_transaction);
    gpio_set_level(imu->imu_config.io_cs, 1);

    if (ret != ESP_OK)
        return 0;

    return parse_valid_packet_len(pBuffer, len);
}

/**
 * @brief SPI tx callback for sh2 HAL lib.
 *
 * @param self sh2 HAL lib object being used with BNO08x driver instance.
 * @param pBuffer Buffer containing data to write.
 * @param len Length in bytes to write.
 *
 * @return Size of sent data (len param), 0 on failure.
 */
int BNO08xSH2HAL::spi_write(sh2_Hal_t* self, uint8_t* pBuffer, unsigned len)
{
    if ((pBuffer == nullptr) || (len == 0U))
        return 0;

    const bool init_phase = (imu->product_IDs.numEntries == 0U) && !imu->init_status.data_proc_task;

    if (!init_phase && !spi_wait_for_int())
        return 0;

    static uint8_t tx_full[SH2_HAL_DMA_SIZE];
    static uint8_t rx_discard[SH2_HAL_DMA_SIZE];
    const unsigned transfer_len = init_phase ? sizeof(tx_full) : len;
    memset(tx_full, 0x00, transfer_len);
    memcpy(tx_full, pBuffer, len);

    imu->spi_transaction.length = transfer_len * 8U;
    imu->spi_transaction.rxlength = init_phase ? (transfer_len * 8U) : 0;
    imu->spi_transaction.tx_buffer = init_phase ? tx_full : pBuffer;
    imu->spi_transaction.rx_buffer = init_phase ? rx_discard : NULL;
    imu->spi_transaction.flags = 0;

    gpio_set_level(imu->imu_config.io_cs, 0);

    if (spi_device_polling_transmit(imu->spi_hdl, &imu->spi_transaction) != ESP_OK)
    {
        gpio_set_level(imu->imu_config.io_cs, 1);
        return 0;
    }

    gpio_set_level(imu->imu_config.io_cs, 1);

    if (init_phase)
    {
        const int packet_len = parse_valid_packet_len(rx_discard, transfer_len);
        if (packet_len > 0)
        {
            memcpy(pending_rx, rx_discard, packet_len);
            pending_rx_len = packet_len;
        }
    }

    return len;
}

/**
 * @brief Get time in microseconds callback for sh2 HAL lib.
 *
 * @param self sh2 HAL lib object being used with BNO08x driver instance.
 *
 * @return Time in microseconds.
 */
uint32_t BNO08xSH2HAL::get_time_us(sh2_Hal_t* self)
{
    uint64_t time_us = esp_timer_get_time();

    if (time_us > UINT32_MAX)
        time_us -= UINT32_MAX;

    return static_cast<uint32_t>(time_us & 0xFFFFFFFFU);
}

/**
 * @brief General event callback for sh2 HAL lib, used to notify tasks of reset.
 *
 * @param cookie User set input parameter as void pointer (unused), see sh2_Open().
 * @param pEvent Pointer to asynchronous event.
 *
 * @return void, nothing to return
 */
void BNO08xSH2HAL::hal_cb(void* cookie, sh2_AsyncEvent_t* pEvent)
{
    if (pEvent->eventId == SH2_RESET)
        xEventGroupSetBits(imu->sync_ctx.evt_grp_task, BNO08xPrivateTypes::EVT_GRP_BNO08x_TASK_RESET_OCCURRED);
}

/**
 * @brief Sensor event callback for sh2 HAL lib, sends received reports to data_proc_task().
 *
 * @param cookie User set input parameter as void pointer (unused), see sh2_Open().
 * @param event Pointer to sensor event.
 *
 * @return void, nothing to return
 */
void BNO08xSH2HAL::sensor_event_cb(void* cookie, sh2_SensorEvent_t* event)
{
    xQueueSend(imu->queue_rx_sensor_event, event, 0);
}

/**
 * @brief Hardware reset callback for sh2 HAL lib, toggle RST gpio.
 *
 * @return void, nothing to return
 */
void BNO08xSH2HAL::hardware_reset()
{
    imu->toggle_reset();
}

/**
 * @brief SPI wait for HINT sh2 HAL lib callback.
 *
 * @return True if interrupt was detected before timeout.
 */
bool BNO08xSH2HAL::spi_wait_for_int()
{
    if (imu->wait_for_hint() != ESP_OK)
    {
        return false;
    }

    return true;
}

/**
 * @brief SPI rx packet header (invoked from SPI rx callback.)
 *
 * @param pBuffer Buffer to store received header.
 *
 * @return Packet size (should always be 4), 0 on failure.
 */
uint16_t BNO08xSH2HAL::spi_read_sh2_packet_header(uint8_t* pBuffer)
{
    uint8_t dummy_header_tx[4] = {0};
    uint16_t packet_sz = 0;

    // setup transaction to receive first 4 bytes (packet header)
    imu->spi_transaction.rx_buffer = pBuffer;
    imu->spi_transaction.tx_buffer = dummy_header_tx;
    imu->spi_transaction.length = 4 * 8;
    imu->spi_transaction.rxlength = 4 * 8;
    imu->spi_transaction.flags = 0;

    if (spi_device_polling_transmit(imu->spi_hdl, &imu->spi_transaction) != ESP_OK)
        return 0;

    packet_sz = PARSE_PACKET_LENGTH(pBuffer);

    // clear continuation/batch bit
    packet_sz &= ~0x8000U;

    return packet_sz;
}

/**
 * @brief SPI rx packet body (invoked from SPI rx callback.)
 *
 * @param pBuffer Buffer to store received packet body.
 *
 * @return Packet size, 0 on failure.
 */
int BNO08xSH2HAL::spi_read_sh2_packet_body(uint8_t* pBuffer, uint16_t packet_sz)
{
    if (packet_sz < 4U)
        return 0;

    const uint16_t body_sz = packet_sz - 4U;
    imu->spi_transaction.rx_buffer = pBuffer + 4;
    imu->spi_transaction.tx_buffer = NULL;
    imu->spi_transaction.length = body_sz * 8U;
    imu->spi_transaction.rxlength = body_sz * 8U;
    imu->spi_transaction.flags = 0;

    if (spi_device_polling_transmit(imu->spi_hdl, &imu->spi_transaction) != ESP_OK)
        return 0;
    else
        return packet_sz;
}
