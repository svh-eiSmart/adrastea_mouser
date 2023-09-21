#include "global_adrastea.h"

void WE_UART_Transmit(uint32_t *handle, const char *buf, uint32_t len)
{	//printf("In 42 WETxUarT, buf : %s", buf);
	serial_write(handle, buf, len);
}

void WE_Delay(uint32_t sleepForMs)
{
	if (sleepForMs > 0)
	    {
		vTaskDelay((uint32_t) sleepForMs);
	    }
}

uint32_t WE_GetTick()
{
    return xTaskGetTickCount();
}

#ifndef WE_MICROSECOND_TICK
void WE_DelayMicroseconds(uint32_t sleepForUsec)
{
    /* Microsecond tick is disabled: round to ms */
    WE_Delay(((sleepForUsec + 500) / 1000));
}

uint32_t WE_GetTickMicroseconds()
{
    /* Microsecond tick is disabled: return ms tick * 1000 */
    return WE_GetTick() * 1000;
}
#endif /* WE_MICROSECOND_TICK */

bool WE_GetDriverVersion(uint8_t* version)
{
    uint8_t help[3] = WE_WIRELESS_CONNECTIVITY_SDK_VERSION;
    memcpy(version, help, 3);
    return true;
}

