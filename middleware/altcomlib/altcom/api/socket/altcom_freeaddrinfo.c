/****************************************************************************
 * modules/lte/altcom/api/socket/altcom_freeaddrinfo.c
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

#include "dbg_if.h"
#include "buffpoolwrapper.h"
#include "altcom_netdb.h"
#include "altcom_cc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_freeaddrinfo
 *
 * Description:
 *   The altcom_freeaddrinfo() function frees the memory that was allocated
 *   for the dynamically allocated linked list res.
 *
 * Parameters:
 *   res - The res returned by altcom_getaddrinfo()
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void altcom_freeaddrinfo(struct altcom_addrinfo* res) {
  FAR struct altcom_addrinfo* ai;
  FAR struct altcom_addrinfo* tmpai;

  if (res) {
    ai = res;
    while (ai) {
      tmpai = ai->ai_next;
      (void)BUFFPOOL_FREE(ai);
      ai = tmpai;
    }
    res = NULL;
  } else {
    DBGIF_LOG_INFO("altcom_freeaddrinfo(). input addrinfo is null.\n");
  }
}
