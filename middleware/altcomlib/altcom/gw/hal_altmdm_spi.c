/****************************************************************************
 * modules/lte/altcom/gw/hal_altmdm_spi.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <nuttx/modem/altmdm.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "dbg_if.h"
#include "buffpoolwrapper.h"
#include "hal_altmdm_spi.h"
#include "altcom_osal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_MODEM_DEVICE_PATH
#define DEV_PATH CONFIG_MODEM_DEVICE_PATH
#else
#warning "CONFIG_MODEM_DEVICE_PATH not defined"
#define DEV_PATH "/dev/altmdm"
#endif

#if defined(CONFIG_MODEM_ALTMDM_MAX_PACKET_SIZE)
#define HAL_ALTMDM_SPI_BUFFER_SIZE_MAX (CONFIG_MODEM_ALTMDM_MAX_PACKET_SIZE)
#else
#define HAL_ALTMDM_SPI_BUFFER_SIZE_MAX (2064)
#endif

#define HAL_ALTMDM_SPI_DMA_TRANSACTION_ALIGN (4)

#define HAL_ALTMDM_SPI_ROUNDUP(datalen, align) (((datalen) + (align)-1u) & ~((align)-1u))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct hal_altmdm_spi_obj_s {
  struct hal_if_s hal_if;
  FAR uint8_t *buff;
  FAR uint8_t *rp;
  uint16_t datalen;
  altcom_sys_mutex_t objmtx;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Send data on SPI.
 *
 * Input Parameters:
 *   thiz      Interface of the gateway.
 *   data      A pointer to the buffer of data to be sent.
 *   datelen   The length of data to be sent.
 *
 * Returned Value:
 *   On success, the length of the sent data in bytes is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t hal_altmdm_spi_send(FAR struct hal_if_s *thiz, FAR const uint8_t *data,
                                   uint32_t len) {
  int32_t ret;
  int32_t fd;
  uint32_t sendlen = HAL_ALTMDM_SPI_ROUNDUP(len, HAL_ALTMDM_SPI_DMA_TRANSACTION_ALIGN);

  fd = open(DEV_PATH, O_WRONLY);
  if (fd < 0) {
    DBGIF_LOG2_ERROR("Device %s open failure. %d\n", DEV_PATH, fd);
    return -ENODEV;
  }

  ret = write(fd, data, sendlen);
  if (0 > ret) {
    ret = -errno;
    DBGIF_LOG1_ERROR("write() failed:%d\n", ret);
  } else {
    DBGIF_LOG1_DEBUG("write success:%d\n", ret);
    if (sendlen == ret) {
      ret = len;
    }
  }

  close(fd);

  return ret;
}

/****************************************************************************
 * Name: hal_altmdm_spi_recv
 *
 * Description:
 *   Receive data on SPI.
 *
 * Input Parameters:
 *   thiz      Interface of the HAL SPI.
 *   buffer    A pointer to the buffer in which to receive data.
 *   len       The length of the buffer to be received.
 *
 * Returned Value:
 *   On success, the length of the received data in bytes is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t hal_altmdm_spi_recv(FAR struct hal_if_s *thiz, FAR uint8_t *buffer, uint32_t len) {
  int32_t ret = 0;
  int32_t fd;
  FAR struct hal_altmdm_spi_obj_s *obj = NULL;

  if (!thiz || !buffer || !len) {
    DBGIF_LOG_ERROR("Incorrect argument.\n");
    return -EINVAL;
  }

  fd = open(DEV_PATH, O_RDONLY);
  if (fd < 0) {
    DBGIF_LOG2_ERROR("Device %s open failure. %d\n", DEV_PATH, fd);
    return -ENODEV;
  }

  obj = (FAR struct hal_altmdm_spi_obj_s *)thiz;
  if (!obj->datalen) {
    ret = read(fd, obj->buff, HAL_ALTMDM_SPI_BUFFER_SIZE_MAX);

    if (0 >= ret) {
      ret = -errno;
      if (-ECONNABORTED == ret) {
        DBGIF_LOG_INFO("read abort\n");
      } else {
        DBGIF_LOG1_ERROR("read() failed:%d\n", ret);
      }

      close(fd);
      return ret;
    }

    obj->datalen = ret;
    obj->rp = obj->buff;
  }

  if (obj->datalen) {
    ret = obj->datalen > len ? len : obj->datalen;
    memcpy(buffer, obj->rp, ret);
    obj->rp += ret;
    obj->datalen -= ret;
  }

  close(fd);

  return ret;
}

/****************************************************************************
 * Name: hal_altmdm_spi_abortrecv
 *
 * Description:
 *   Aborts a blocking hal_altmdm_spi_recv() call.
 *
 * Input Parameters:
 *   thiz      Interface of the gateway.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t hal_altmdm_spi_abortrecv(FAR struct hal_if_s *thiz) {
  int32_t ret;
  int32_t fd;

  fd = open(DEV_PATH, O_RDWR);
  if (fd < 0) {
    DBGIF_LOG2_ERROR("Device %s open failure. %d\n", DEV_PATH, fd);
    return -ENODEV;
  }

  ret = ioctl(fd, MODEM_IOC_READABORT, 0);
  if (ret < 0) {
    ret = -errno;
    DBGIF_LOG1_ERROR("ioctl failed:%d\n", ret);
  } else {
    DBGIF_LOG1_DEBUG("ioctl success:%d\n", ret);
  }

  close(fd);

  return ret;
}

/****************************************************************************
 * Name: hal_altmdm_spi_lock
 *
 * Description:
 *   Acquire lock on the HAL SPI.
 *
 * Input Parameters:
 *   thiz      Interface of the HAL SPI.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t hal_altmdm_spi_lock(FAR struct hal_if_s *thiz) {
  int32_t ret;
  FAR struct hal_altmdm_spi_obj_s *obj = NULL;

  if (!thiz) {
    DBGIF_LOG_ERROR("null parameter.\n");
    return -EINVAL;
  }

  obj = (FAR struct hal_altmdm_spi_obj_s *)thiz;
  ret = altcom_sys_lock_mutex(&obj->objmtx);
  if (ret < 0) {
    DBGIF_LOG1_ERROR("Failed to lock :%d\n", ret);
  }

  return ret;
}

/****************************************************************************
 * Name: hal_altmdm_spi_unlock
 *
 * Description:
 *   Release lock on the HAL SPI.
 *
 * Input Parameters:
 *   thiz      Interface of the HAL SPI.
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   On failure, negative value is returned.
 *
 ****************************************************************************/

static int32_t hal_altmdm_spi_unlock(FAR struct hal_if_s *thiz) {
  int32_t ret;
  FAR struct hal_altmdm_spi_obj_s *obj = NULL;

  if (!thiz) {
    DBGIF_LOG_ERROR("null parameter.\n");
    return -EINVAL;
  }

  obj = (FAR struct hal_altmdm_spi_obj_s *)thiz;
  ret = altcom_sys_unlock_mutex(&obj->objmtx);
  if (ret < 0) {
    DBGIF_LOG1_ERROR("Failed to unlock :%d\n", ret);
  }

  return ret;
}

/****************************************************************************
 * Name: hal_altmdm_spi_allocbuff
 *
 * Description:
 *   Allocat buffer for spi driver transaction message.
 *
 * Input Parameters:
 *   thiz     Instance of HAL SPI.
 *   len      Allocat memory size.
 *
 * Returned Value:
 *   If succeeds allocate buffer, start address of the data field
 *   is returned. Otherwise NULL is returned.
 *
 ****************************************************************************/

static FAR void *hal_altmdm_spi_allocbuff(FAR struct hal_if_s *thiz, uint32_t len) {
  uint16_t size = 0;

  if (!len) {
    DBGIF_LOG_INFO("alloc request size 0.\n");
  }

  size = HAL_ALTMDM_SPI_ROUNDUP(len, HAL_ALTMDM_SPI_DMA_TRANSACTION_ALIGN);

  if (HAL_ALTMDM_SPI_BUFFER_SIZE_MAX < size) {
    DBGIF_LOG1_ERROR("alloc request size err. size=[%d]\n", size);
    return NULL;
  }

  return BUFFPOOL_ALLOC(size);
}

/****************************************************************************
 * Name: hal_altmdm_spi_freebuff
 *
 * Description:
 *   Free buffer for spi driver transaction message.
 *
 * Input Parameters:
 *   buff      Allocated memory pointer.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

static int32_t hal_altmdm_spi_freebuff(FAR struct hal_if_s *thiz, FAR void *buff) {
  if (!buff) {
    DBGIF_LOG_INFO("free target is null.\n");
  }

  return BUFFPOOL_FREE(buff);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hal_altmdm_spi_create
 *
 * Description:
 *   Create an object of HAL SPI and get the instance.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   struct struct hal_if_s pointer(i.e. instance of HAL SPI).
 *   If can't create instance, returned NULL.
 *
 ****************************************************************************/

FAR struct hal_if_s *hal_altmdm_spi_create(void) {
  int32_t ret;
  FAR struct hal_altmdm_spi_obj_s *obj = NULL;
  altcom_sys_cremtx_s param = {0};

  obj = (FAR struct hal_altmdm_spi_obj_s *)BUFFPOOL_ALLOC(sizeof(struct hal_altmdm_spi_obj_s));
  if (obj == NULL) {
    DBGIF_LOG_ERROR("Failed to allocate memory\n");
    return NULL;
  }

  obj->hal_if.send = hal_altmdm_spi_send;
  obj->hal_if.recv = hal_altmdm_spi_recv;
  obj->hal_if.abortrecv = hal_altmdm_spi_abortrecv;
  obj->hal_if.lock = hal_altmdm_spi_lock;
  obj->hal_if.unlock = hal_altmdm_spi_unlock;
  obj->hal_if.allocbuff = hal_altmdm_spi_allocbuff;
  obj->hal_if.freebuff = hal_altmdm_spi_freebuff;

  ret = altcom_sys_create_mutex(&obj->objmtx, &param);
  if (ret < 0) {
    DBGIF_LOG1_ERROR("Failed to create mutex :%d\n", ret);
    goto errout;
  }

  obj->buff = (FAR uint8_t *)BUFFPOOL_ALLOC(HAL_ALTMDM_SPI_BUFFER_SIZE_MAX);
  if (!obj->buff) {
    DBGIF_LOG_ERROR("Failed to allocate memory\n");
    (void)altcom_sys_delete_mutex(&obj->objmtx);
    goto errout;
  }

  obj->datalen = 0;
  obj->rp = 0;

  return (FAR struct hal_if_s *)obj;

errout:
  (void)BUFFPOOL_FREE(obj);

  return NULL;
}

/****************************************************************************
 * Name: hal_altmdm_spi_delete
 *
 * Description:
 *   Delete instance of HAL SPI.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   If the process succeeds, it returns 0.
 *   Otherwise errno is returned.
 *
 ****************************************************************************/

int32_t hal_altmdm_spi_delete(FAR struct hal_if_s *thiz) {
  int32_t ret;
  FAR struct hal_altmdm_spi_obj_s *obj = NULL;

  if (!thiz) {
    DBGIF_LOG_ERROR("null parameter.\n");
    return -EINVAL;
  }

  obj = (FAR struct hal_altmdm_spi_obj_s *)thiz;
  ret = altcom_sys_delete_mutex(&obj->objmtx);
  if (ret < 0) {
    DBGIF_ASSERT(ret >= 0, "mutex delete failed.");
  }

  (void)BUFFPOOL_FREE(obj->buff);
  (void)BUFFPOOL_FREE(thiz);

  return 0;
}
