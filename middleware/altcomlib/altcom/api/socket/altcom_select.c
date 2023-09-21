/****************************************************************************
 * modules/lte/altcom/api/socket/altcom_select.c
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

#include <stdbool.h>

#include "dbg_if.h"
#include "altcom_sock.h"
#include "altcom_socket.h"
#include "altcom_select.h"
#include "altcom_seterrno.h"
#include "apicmd_select.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "altcom_cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SELECT_REQ_DATALEN (sizeof(struct apicmd_select_s))
#define SELECT_RES_DATALEN (sizeof(struct apicmd_selectres_s))

#define SELECT_REQ_FAILURE -1
#define SELECT_ID_GEN_AUTO -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct select_req_s {
  int32_t select_id;
  int maxfdp1;
  uint32_t request;
  FAR altcom_fd_set *readset;
  FAR altcom_fd_set *writeset;
  FAR altcom_fd_set *exceptset;
  int32_t timeout;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int32_t g_select_id = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: generate_selectid
 *
 * Description:
 *   Generate select ID.
 *
 ****************************************************************************/

static int32_t generate_selectid(void) {
  g_select_id++;

  return (g_select_id & 0x7fffffff);
}

/****************************************************************************
 * Name: fill_select_request_command
 *
 * Description:
 *   Fill the ALTCOM_SELECT_REQ command.
 *
 ****************************************************************************/

static void fill_select_request_command(FAR struct apicmd_select_s *cmd,
                                        FAR struct select_req_s *req) {
  memset(cmd, 0, sizeof(struct apicmd_select_s));

  cmd->request = htonl(req->request);
  cmd->id = htonl(req->select_id);
  cmd->maxfds = htonl(req->maxfdp1);
  if (req->readset) {
    memcpy(&cmd->readset, req->readset, sizeof(altcom_fd_set));
    cmd->used_setbit |= APICMD_SELECT_USED_BIT_READSET;
  }
  if (req->writeset) {
    memcpy(&cmd->writeset, req->writeset, sizeof(altcom_fd_set));
    cmd->used_setbit |= APICMD_SELECT_USED_BIT_WRITESET;
  }
  if (req->exceptset) {
    memcpy(&cmd->exceptset, req->exceptset, sizeof(altcom_fd_set));
    cmd->used_setbit |= APICMD_SELECT_USED_BIT_EXCEPTSET;
  }
  cmd->used_setbit = htons(cmd->used_setbit);

  DBGIF_LOG3_DEBUG("[select-req]request: %lu, id: %ld, maxfdp1: %d\n", req->request, req->select_id,
                   req->maxfdp1);
  DBGIF_LOG1_DEBUG("[select-req]used_setbit: %x\n", (unsigned int)ntohs(cmd->used_setbit));
  if (req->readset) {
    DBGIF_LOG2_DEBUG("[select-req]readset: %x,%x\n", req->readset->fd_bits[0],
                     req->readset->fd_bits[1]);
  }
  if (req->writeset) {
    DBGIF_LOG2_DEBUG("[select-req]writeset: %x,%x\n", req->writeset->fd_bits[0],
                     req->writeset->fd_bits[1]);
  }
  if (req->exceptset) {
    DBGIF_LOG2_DEBUG("[select-req]exceptset: %x,%x\n", req->exceptset->fd_bits[0],
                     req->exceptset->fd_bits[1]);
  }
}

/****************************************************************************
 * Name: select_request
 *
 * Description:
 *   Send ALTCOM_SELECT_REQ.
 *
 ****************************************************************************/

static int32_t select_request(FAR struct select_req_s *req) {
  int32_t ret;
  int32_t err;
  uint16_t reslen = 0;
  FAR struct apicmd_select_s *cmd = NULL;
  FAR struct apicmd_selectres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_sock_alloc_cmdandresbuff((FAR void **)&cmd, APICMDID_SOCK_SELECT, SELECT_REQ_DATALEN,
                                       (FAR void **)&res, SELECT_RES_DATALEN)) {
    err = ALTCOM_ENOMEM;
    goto errout;
  }

  if (req->select_id == SELECT_ID_GEN_AUTO) {
    req->select_id = generate_selectid();
  }

  /* Fill the data */

  fill_select_request_command(cmd, req);

  /* Send command and block until receive a response or timeout */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res, SELECT_RES_DATALEN, &reslen,
                      req->timeout);

  if (ret < 0) {
    DBGIF_LOG1_ERROR("[select]apicmdgw_send error: %ld\n", ret);

    if (ret == -ETIMEDOUT) {
      memset(cmd, 0, sizeof(struct apicmd_select_s));
      cmd->request = htonl(APICMD_SELECT_REQUEST_BLOCKCANCEL);
      cmd->id = htonl(req->select_id);

      DBGIF_LOG2_DEBUG("[select-req]request: %d, id: %ld\n", APICMD_SELECT_REQUEST_BLOCKCANCEL,
                       req->select_id);

      /* Send command */

      ret = apicmdgw_send((FAR uint8_t *)cmd, NULL, 0, NULL, 0);
      if (ret < 0) {
        DBGIF_LOG1_ERROR("[select-blockcancel]apicmdgw_send error: %ld\n", ret);
      }

      err = ALTCOM_ETIMEDOUT;
      goto errout_with_cmdfree;
    } else {
      err = -ret;
      goto errout_with_cmdfree;
    }
  }

  if (reslen != SELECT_RES_DATALEN) {
    DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
    err = ALTCOM_EFAULT;
    goto errout_with_cmdfree;
  }

  ret = ntohl(res->ret_code);
  err = ntohl(res->err_code);

  DBGIF_LOG2_DEBUG("[select-res]ret: %ld, err: %ld\n", ret, err);

  if (APICMD_SELECT_RES_RET_CODE_ERR == ret) {
    DBGIF_LOG1_ERROR("API command response is err :%ld.\n", err);
    goto errout_with_cmdfree;
  }

  if (req->readset) {
    memcpy(req->readset, &res->readset, sizeof(altcom_fd_set));
  }
  if (req->writeset) {
    memcpy(req->writeset, &res->writeset, sizeof(altcom_fd_set));
  }
  if (req->exceptset) {
    memcpy(req->exceptset, &res->exceptset, sizeof(altcom_fd_set));
  }

  altcom_sock_free_cmdandresbuff(cmd, res);

  return ret;

errout_with_cmdfree:
  altcom_sock_free_cmdandresbuff(cmd, res);

errout:
  altcom_seterrno(err);
  return SELECT_REQ_FAILURE;
}

/****************************************************************************
 * Name: select_request_async
 *
 * Description:
 *   Send ALTCOM_SELECT_REQ. Just send and do not wait for a response.
 *
 ****************************************************************************/

static int32_t select_request_async(FAR struct select_req_s *req) {
  int32_t ret;
  int32_t err;
  FAR struct apicmd_select_s *cmd = NULL;

  /* Allocate send command buffer only */

  if (!ALTCOM_SOCK_ALLOC_CMDBUFF(cmd, APICMDID_SOCK_SELECT, SELECT_REQ_DATALEN)) {
    err = ALTCOM_ENOMEM;
    goto errout;
  }

  if (req->select_id == SELECT_ID_GEN_AUTO) {
    req->select_id = generate_selectid();
  }

  /* Fill the data */

  fill_select_request_command(cmd, req);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, NULL, 0, NULL, ALTCOM_SYS_TIMEO_FEVR);

  if (ret < 0) {
    DBGIF_LOG1_ERROR("apicmdgw_send error: %ld\n", ret);

    err = -ret;
    goto errout_with_cmdfree;
  }

  ret = req->select_id;

  altcom_free_cmd((FAR uint8_t *)cmd);

  return ret;

errout_with_cmdfree:
  altcom_free_cmd((FAR uint8_t *)cmd);

errout:
  altcom_seterrno(err);
  return SELECT_REQ_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_select_request_asyncsend
 *
 * Description:
 *   Send select request.
 *
 * Input parameters:
 *   maxfdp1 - the maximum socket file descriptor number (+1) of any
 *             descriptor in any of the three sets.
 *   readset - the set of descriptions to monitor for read-ready events
 *   writeset - the set of descriptions to monitor for write-ready events
 *   exceptset - the set of descriptions to monitor for error events
 *
 *  Return:
 *  >0: The select id use select cancel request
 *  -1: An error occurred (errno will be set appropriately)
 *
 ****************************************************************************/

int altcom_select_request_asyncsend(int maxfdp1, altcom_fd_set *readset, altcom_fd_set *writeset,
                                    altcom_fd_set *exceptset) {
  int32_t result;
  struct select_req_s req;

  if (!altcom_isinit()) {
    DBGIF_LOG_ERROR("Not intialized\n");
    altcom_seterrno(ALTCOM_ENETDOWN);
    return -1;
  }

  req.select_id = SELECT_ID_GEN_AUTO;
  req.maxfdp1 = maxfdp1;
  req.request = APICMD_SELECT_REQUEST_BLOCK;
  req.readset = readset;
  req.writeset = writeset;
  req.exceptset = exceptset;
  req.timeout = ALTCOM_SYS_TIMEO_FEVR; /* ignore when do select_request_async */

  result = select_request_async(&req);

  if (result == SELECT_REQ_FAILURE) {
    return -1;
  }

  return result;
}

/****************************************************************************
 * Name: altcom_select_cancel_request_send
 *
 * Description:
 *   Send select cancel request.
 *
 * Input parameters:
 *   id - returned by altcom_select_request_asyncsend
 *
 *  Return:
 *   0: Send succeded
 *  -1: An error occurred (errno will be set appropriately)
 *
 ****************************************************************************/

int altcom_select_cancel_request_send(int id) {
  int32_t result;
  struct select_req_s req;

  if (!altcom_isinit()) {
    DBGIF_LOG_ERROR("Not intialized\n");
    altcom_seterrno(ALTCOM_ENETDOWN);
    return -1;
  }

  req.select_id = id;
  req.maxfdp1 = 0;
  req.request = APICMD_SELECT_REQUEST_BLOCKCANCEL;
  req.readset = NULL;
  req.writeset = NULL;
  req.exceptset = NULL;
  req.timeout = ALTCOM_SYS_TIMEO_FEVR; /* ignore when do select_request_async */

  result = select_request_async(&req);

  if (result == SELECT_REQ_FAILURE) {
    return -1;
  }

  return result;
}

/****************************************************************************
 * Name: altcom_select_nonblock
 *
 * Description:
 *   altcom_select_nonblock() is the same as altcom_select(),
 *   but set command parameter to nonblock.
 *
 * Input parameters:
 *   maxfdp1 - the maximum socket file descriptor number (+1) of any
 *             descriptor in any of the three sets.
 *   readset - the set of descriptions to monitor for read-ready events
 *   writeset - the set of descriptions to monitor for write-ready events
 *   exceptset - the set of descriptions to monitor for error events
 *
 *  Return:
 *   0: All descriptors in the three sets are not ready
 *  >0: The number of bits set in the three sets of descriptors
 *  -1: An error occurred (errno will be set appropriately)
 *
 ****************************************************************************/

int altcom_select_nonblock(int maxfdp1, altcom_fd_set *readset, altcom_fd_set *writeset,
                           altcom_fd_set *exceptset) {
  int32_t result;
  struct select_req_s req;

  if (!altcom_isinit()) {
    DBGIF_LOG_ERROR("Not intialized\n");
    altcom_seterrno(ALTCOM_ENETDOWN);
    return -1;
  }

  req.select_id = SELECT_ID_GEN_AUTO;
  req.maxfdp1 = maxfdp1;
  req.request = APICMD_SELECT_REQUEST_NONBLOCK;
  req.readset = readset;
  req.writeset = writeset;
  req.exceptset = exceptset;
  req.timeout = ALTCOM_SYS_TIMEO_FEVR;

  result = select_request(&req);

  if (result == SELECT_REQ_FAILURE) {
    return -1;
  }

  return result;
}

/****************************************************************************
 * Name: altcom_select_block
 *
 * Description:
 *   altcom_select_nonblock() is the same as altcom_select(),
 *   but set command parameter to block.
 *
 * Input parameters:
 *   maxfdp1 - the maximum socket file descriptor number (+1) of any
 *             descriptor in any of the three sets.
 *   readset - the set of descriptions to monitor for read-ready events
 *   writeset - the set of descriptions to monitor for write-ready events
 *   exceptset - the set of descriptions to monitor for error events
 *   timeout - Return at this time if none of these events of interest
 *             occur.
 *
 *  Return:
 *  >0: The number of bits set in the three sets of descriptors
 *  -1: An error occurred (errno will be set appropriately)
 *
 ****************************************************************************/

int altcom_select_block(int maxfdp1, altcom_fd_set *readset, altcom_fd_set *writeset,
                        altcom_fd_set *exceptset, struct altcom_timeval *timeout) {
  int32_t result;
  struct select_req_s req;

  if (!altcom_isinit()) {
    DBGIF_LOG_ERROR("Not intialized\n");
    altcom_seterrno(ALTCOM_ENETDOWN);
    return -1;
  }

  req.select_id = SELECT_ID_GEN_AUTO;
  req.maxfdp1 = maxfdp1;
  req.request = APICMD_SELECT_REQUEST_BLOCK;
  req.readset = readset;
  req.writeset = writeset;
  req.exceptset = exceptset;
  if (timeout) {
    req.timeout = ALTCOM_SOCK_TIMEVAL2MS(timeout);
  } else {
    req.timeout = ALTCOM_SYS_TIMEO_FEVR;
  }

  result = select_request(&req);

  if (result == SELECT_REQ_FAILURE) {
    return -1;
  }

  return result;
}

/****************************************************************************
 * Name: altcom_select
 *
 * Description:
 *   altcom_select() allows a program to monitor multiple file descriptors,
 *   waiting until one or more of the file descriptors become "ready" for
 *   some class of I/O operation (e.g., input possible).  A file descriptor
 *   is considered  ready if it is possible to perform the corresponding I/O
 *   operation (e.g., altcom_recv()) without blocking.
 *
 * Input parameters:
 *   maxfdp1 - the maximum socket file descriptor number (+1) of any
 *             descriptor in any of the three sets.
 *   readset - the set of descriptions to monitor for read-ready events
 *   writeset - the set of descriptions to monitor for write-ready events
 *   exceptset - the set of descriptions to monitor for error events
 *   timeout - Return at this time if none of these events of interest
 *             occur.
 *
 *  Return:
 *   0: Timer expired
 *  >0: The number of bits set in the three sets of descriptors
 *  -1: An error occurred (errno will be set appropriately)
 *
 ****************************************************************************/

int altcom_select(int maxfdp1, altcom_fd_set *readset, altcom_fd_set *writeset,
                  altcom_fd_set *exceptset, struct altcom_timeval *timeout) {
  int ret;

  if (timeout && (timeout->tv_sec == 0) && (timeout->tv_usec == 0)) {
    ret = altcom_select_nonblock(maxfdp1, readset, writeset, exceptset);
  } else {
    ret = altcom_select_block(maxfdp1, readset, writeset, exceptset, timeout);
  }

  if ((ret == SELECT_REQ_FAILURE) && (altcom_errno() == ALTCOM_ETIMEDOUT)) {
    ret = 0;
  }

  return ret;
}
