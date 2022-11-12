#ifndef __ASYNC_I2C_H
#define __ASYNC_I2C_H

#include <stdint.h>
#include "hardware/structs/i2c.h"

typedef enum
{
    ASYNC_I2C_ERR = 0,
    ASYNC_I2C_OK = 1
} AsyncI2CStatus;

typedef enum
{
    ASYNC_I2C_STATE_READY,
    ASYNC_I2C_STATE_BUSY
} AsyncI2CState;

typedef struct
{
    // set before first call to async_i2c_init
    i2c_hw_t *instance;

    // set before first call to async_i2c_init
    uint32_t baudrate;

    // set before first call to async_i2c_init
    uint32_t sda_gpio;
    uint32_t scl_gpio;

    // ----------------------------------
    // internal use below - do not modify
    // ----------------------------------
    void *tx_data;
    uint16_t tx_len;
    AsyncI2CState tx_state;
    void (*tx_cplt_callback)(i2c_hw_t *);

    void *rx_data;
    uint16_t rx_len;
    AsyncI2CState rx_state;
    void (*rx_cplt_callback)(i2c_hw_t *);

    void (*error_callback)(i2c_hw_t *);

} AsyncI2CHandle;

/// @brief Initializes interface handle and underlying hardware
/// @param handle Handle for I2C interface. User must fill necessary fields before calling this function (see async_i2c.h)
/// @return ASYNC_I2C_OK on success, ASYNC_I2C_ERR otherwise
AsyncI2CStatus async_i2c_init(AsyncI2CHandle *handle);

/// @brief Asynchronously sends data via I2C interface
/// @param handle Handle for I2C interface
/// @param addr Peripheral 7 bit address
/// @param data Pointer to user data
/// @param len Length of user data
/// @return ASYNC_I2C_OK on success, ASYNC_I2C_ERR otherwise
AsyncI2CStatus async_i2c_transmit(AsyncI2CHandle *handle, uint8_t addr, void *data, uint16_t len);

/// @brief Asynchronously recieves data via I2C interface
/// @param handle Handle for I2C interface
/// @param addr Peripheral 7 bit address
/// @param data Pointer to recieve buffer
/// @param len Length of the data to be recieved
/// @return ASYNC_I2C_OK on success, ASYNC_I2C_ERR otherwise
AsyncI2CStatus async_i2c_recieve(AsyncI2CHandle *handle, uint8_t addr, void *data, uint16_t len);

/// @brief Registers callback that will be called when all user data is transmitted
/// @param handle Handle for I2C interface
/// @param callback User function to be called
/// @return ASYNC_I2C_OK on success, ASYNC_I2C_ERR otherwise
AsyncI2CStatus async_i2c_register_tx_cplt_callback(AsyncI2CHandle *handle, void (*callback)(i2c_hw_t *));

/// @brief Registers callback that will be called when all requested data is recieved
/// @param handle Handle for I2C interface
/// @param callback User function to be called
/// @return ASYNC_I2C_OK on success, ASYNC_I2C_ERR otherwise
AsyncI2CStatus async_i2c_register_rx_cplt_callback(AsyncI2CHandle *handle, void (*callback)(i2c_hw_t *));

/// @brief Registers callback that will be called when error occurs
/// @param handle Handle for I2C interface
/// @param callback User function to be called
/// @return ASYNC_I2C_OK on success, ASYNC_I2C_ERR otherwise
AsyncI2CStatus async_i2c_register_error_callback(AsyncI2CHandle *handle, void (*callback)(i2c_hw_t *));

/// @brief Handle I2C interrupt, call to this function must be placed in I2C interrupt handler
/// @param handle Handle for this I2C interface
void async_i2c_irq_handler(AsyncI2CHandle *handle);

#endif /* __ASYNC_I2C_H */