/****************************************************************************
 * modules/lte/include/net/altcom_select_ext.h
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

/**
 * @file altcom_select_ext.h
 */

#ifndef __MODULES_LTE_INCLUDE_NET_ALTCOM_SELECT_EXT_H
#define __MODULES_LTE_INCLUDE_NET_ALTCOM_SELECT_EXT_H

/**
 * @defgroup net NET Connector APIs
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "altcom_select.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/** Definition of callback function.
 *  Since altcom_select_async() is an asynchronous API,
 *  the result is notified by this function
 * @param[in] ret_code - Return code of the select request on specific fd.
 * @param[in] err_code - Error code of the request on specific fd.
 * @param[in] id - The ID to identify the select request.
 * @param[in] readset - The set of descriptions to monitor for read-ready events
 * @param[in] writeset - The set of descriptions to monitor for write-ready events
 * @param[in] exceptset - The set of descriptions to monitor for error events
 * @param[in] priv - User's private data.
 */

typedef void (*altcom_select_async_cb_t)(int32_t ret_code, int32_t err_code, int32_t id,
                                         altcom_fd_set *readset, altcom_fd_set *writeset,
                                         altcom_fd_set *exceptset, void *priv);

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @defgroup socket_funcs SOCKET APIs
 * @{
 */

/**
 * Name: altcom_select_async
 *
 *   altcom_select_async() is a function that operates select asynchronously.
 *   Notified by callback when one or more of the file descriptors become
 *   "ready" for some class of I/O operation (e.g., input possible). It will
 *   be notified until callback is called once or canceled
 *   by altcom_select_async_cancel().
 *
 *   @param [in] maxfdp1 - the maximum socket file descriptor number (+1) of any
 *             descriptor in any of the three sets.
 *   @param [in] readset - the set of descriptions to monitor for read-ready events
 *   @param [in] writeset - the set of descriptions to monitor for write-ready events
 *   @param [in] exceptset - the set of descriptions to monitor for error events
 *   @param [in] callback - Callback function that informs that one or more file
 *              descriptors are in the "ready" state
 *   @param [in] priv - For use by caller
 *
 *   @return A non-negative id on success; -1 on error with errno set
 *   appropriately.
 *
 */

int altcom_select_async(int maxfdp1, altcom_fd_set *readset, altcom_fd_set *writeset,
                        altcom_fd_set *exceptset, altcom_select_async_cb_t callback, void *priv);

/****************************************************************************
 * Name: altcom_select_async_exec_callback
 *
 * Description:
 *   Execute callback that registered by altcom_select_async().
 *
 * Input parameters:
 *   id - id returned by altcom_select_async()
 *   ret_code - result of altcom_select_async()
 *   err_code - error code when altcom_select_async() failure
 *   readset - the set of descriptions to monitor for read-ready events
 *   writeset - the set of descriptions to monitor for write-ready events
 *   exceptset - the set of descriptions to monitor for error events
 *
 *  Return:
 *   If the process succeeds, it returns 0.
 *   Otherwise negative value is returned.
 *
 ****************************************************************************/
/// @private
int altcom_select_async_exec_callback(int32_t id, int32_t ret_code, int32_t err_code,
                                      altcom_fd_set *readset, altcom_fd_set *writeset,
                                      altcom_fd_set *exceptset);

/**
 * Name: altcom_select_async_cancel
 *
 *   altcom_select_async_cancel() cancel altcom_select_async() based on the
 *   id.
 *
 *   @param [in] id - id returned by altcom_select_async()
 *
 *   @return 0 on success; -1 on error with errno set appropriately
 *
 */

int altcom_select_async_cancel(int id);

/** @} socket_funcs */

#undef EXTERN
#ifdef __cplusplus
}
#endif

/** @} net */

#endif /* __MODULES_LTE_INCLUDE_NET_ALTCOM_SELECT_EXT_H */
