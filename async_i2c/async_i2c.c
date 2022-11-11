#include "hardware/gpio.h"
#include "hardware/clocks.h"

#include "async_i2c.h"

static void handle_tx_empty_irq(AsyncI2CHandle *handle);

void async_i2c_init(AsyncI2CHandle *handle)
{
    handle->tx_data = NULL;
    handle->tx_cplt_callback = NULL;

    // disable i2c
    handle->instance->enable &= ~(1 << I2C_IC_ENABLE_ENABLE_LSB);

    // 7 bit address
    handle->instance->con &= ~(1 << I2C_IC_CON_IC_10BITADDR_MASTER_LSB);

    // master mode
    handle->instance->con |= ((1 << I2C_IC_CON_MASTER_MODE_LSB) | (1 << I2C_IC_CON_IC_SLAVE_DISABLE_LSB));

    // hold bus when rx fifo full
    handle->instance->con |= (1 << I2C_IC_CON_RX_FIFO_FULL_HLD_CTRL_LSB);

    // restart can be generated
    handle->instance->con |= (1 << I2C_IC_CON_IC_RESTART_EN_LSB);

    // fast mode (works in standard mode too)
    handle->instance->con |= (I2C_IC_CON_SPEED_VALUE_FAST << I2C_IC_CON_SPEED_LSB);

    // see "i2c.c" from official SDK for calculations
    uint32_t sys_clk_hz = clock_get_hz(clk_sys);
    uint32_t peroid = (sys_clk_hz + handle->baudrate / 2) / handle->baudrate;
    uint32_t low_cnt = peroid * 3 / 5;
    uint32_t high_cnt = peroid - low_cnt;

    uint32_t sda_tx_hold_cnt;
    if (handle->baudrate < 1000000U)
        sda_tx_hold_cnt = ((sys_clk_hz * 3) / 10000000U) + 1;
    else
        sda_tx_hold_cnt = ((sys_clk_hz * 3) / 25000000U) + 1;

    handle->instance->fs_scl_hcnt = high_cnt;
    handle->instance->fs_scl_lcnt = low_cnt;
    handle->instance->fs_spklen = low_cnt < 16 ? 1 : low_cnt / 16;
    handle->instance->sda_hold |= (sda_tx_hold_cnt << I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_LSB);

    // set pins
    gpio_set_function(handle->scl_gpio, GPIO_FUNC_I2C);
    gpio_set_function(handle->sda_gpio, GPIO_FUNC_I2C);
    gpio_pull_up(handle->scl_gpio);
    gpio_pull_up(handle->sda_gpio);

    // make sure interrupts are disabled
    handle->instance->intr_mask = 0;

    // enable i2c
    handle->instance->enable |= (1 << I2C_IC_ENABLE_ENABLE_LSB);

    handle->tx_state = ASYNC_I2C_STATE_TX_READY;
}

AsyncI2CStatus async_i2c_transmit(AsyncI2CHandle *handle, uint8_t addr, void *data, uint16_t len)
{
    if ((handle->tx_state != ASYNC_I2C_STATE_TX_READY) || (data == NULL) || (len == 0))
        return ASYNC_I2C_ERR;

    handle->tx_state = ASYNC_I2C_STATE_TX_BUSY;

    handle->instance->enable &= ~(1 << I2C_IC_ENABLE_ENABLE_LSB);
    handle->instance->tar = addr;
    handle->instance->enable |= (1 << I2C_IC_ENABLE_ENABLE_LSB);

    uint32_t i2c_cmd = 0;

    // one byte is loaded in this function call
    len--;

    handle->tx_len = len;
    handle->tx_data = data;

    i2c_cmd |= (0xff & (*((uint8_t *)data)));

    // generate stop after this byte
    if (len == 0)
    {
        i2c_cmd |= (1 << I2C_IC_DATA_CMD_STOP_LSB);

        // i2c will be ready to use after this function call
        handle->tx_state = ASYNC_I2C_STATE_TX_READY;

        // push first command
        handle->instance->data_cmd = i2c_cmd;

        if (handle->tx_cplt_callback != NULL)
            handle->tx_cplt_callback(handle->instance);
    }
    else
    {
        // push first command
        // it must be pushed before tx interrupts are enabled
        handle->instance->data_cmd = i2c_cmd;

        // tx empty interrupt
        handle->instance->intr_mask |= (1 << I2C_IC_INTR_MASK_M_TX_EMPTY_LSB);
    }

    return ASYNC_I2C_OK;
}

void async_i2c_register_tx_cplt_callback(AsyncI2CHandle *handle, void (*callback)(i2c_hw_t *))
{
    handle->tx_cplt_callback = callback;
}

void async_i2c_irq_handler(AsyncI2CHandle *handle)
{
    // tx fifo empty interrupt
    if (handle->instance->intr_stat & I2C_IC_INTR_STAT_R_TX_EMPTY_BITS)
    {
        handle_tx_empty_irq(handle);
    }
    else
    {
        uint32_t dummy_read = handle->instance->clr_intr;
    }
}

static void handle_tx_empty_irq(AsyncI2CHandle *handle)
{
    uint32_t i2c_cmd = 0;

    handle->tx_data++;
    handle->tx_len--;

    i2c_cmd |= (0xff & (*((uint8_t *)handle->tx_data)));

    if (handle->tx_len == 0)
    {
        i2c_cmd |= (1 << I2C_IC_DATA_CMD_STOP_LSB);
        handle->instance->intr_mask &= ~(1 << I2C_IC_INTR_MASK_M_TX_EMPTY_LSB);

        // last byte must be pushed before calling user callback
        handle->instance->data_cmd = i2c_cmd;

        handle->tx_state = ASYNC_I2C_STATE_TX_READY;

        // call user callback
        if (handle->tx_cplt_callback != NULL)
            handle->tx_cplt_callback(handle->instance);

        return;
    }

    // push command and data to fifo
    handle->instance->data_cmd = i2c_cmd;

    // cleared automatically by hardware
}