#include <stdbool.h>
#include <stdint.h>

#define WE_WIRELESS_CONNECTIVITY_SDK_VERSION {1,7,1}

/**
 * @brief Priority for UART interrupts (used for communicating with radio module)
 */
#define WE_PRIORITY_UART_RX 0

/**
 * @brief UART interface used for communicating with radio module.
 */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */

// extern void UartTransmitInternal(serial_handle *modemUartHandle, const uint8_t *data, uint16_t size);
extern void WE_UART_Transmit(uint32_t *handle, const char *buf, uint32_t len);


/**
 * @brief Sleep function.
 *
 * @param[in] sleepForMs Delay in milliseconds
 */
extern void WE_Delay(uint32_t sleepForMs);

/**
 * @brief Sleep function.
 *
 * Note that WE_MICROSECOND_TICK needs to be defined to enable microsecond timer resolution.
 *
 * @param[in] sleepForUsec Delay in microseconds
 */
extern void WE_DelayMicroseconds(uint32_t sleepForUsec);

/**
 * @brief Returns current tick value (in milliseconds).
 *
 * @return Current tick value (in milliseconds)
 */
extern uint32_t WE_GetTick();

/**
 * @brief Returns current tick value (in microseconds).
 *
 * Note that WE_MICROSECOND_TICK needs to be defined to enable microsecond timer resolution.
 *
 * @return Current tick value (in microseconds)
 */
extern uint32_t WE_GetTickMicroseconds();

