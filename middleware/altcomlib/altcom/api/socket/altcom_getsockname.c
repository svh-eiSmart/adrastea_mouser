/****************************************************************************
 * modules/lte/altcom/api/socket/altcom_getsockname.c
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

#include <string.h>
#include <stdbool.h>

#include "dbg_if.h"
#include "altcom_socket.h"
#include "altcom_select.h"
#include "altcom_sock.h"
#include "altcom_seterrno.h"
#include "apicmd_getsockname.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "altcom_cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GETSOCKNAME_REQ_DATALEN (sizeof(struct apicmd_getsockname_s))
#define GETSOCKNAME_RES_DATALEN (sizeof(struct apicmd_getsocknameres_s))

#define GETSOCKNAME_REQ_SUCCESS 0
#define GETSOCKNAME_REQ_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct getsockname_req_s {
  int sockfd;
  FAR struct altcom_sockaddr *addr;
  FAR altcom_socklen_t *addrlen;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getsockname_request
 *
 * Description:
 *   Send ALTCOM_GETSOCKNAME_REQ.
 *
 ****************************************************************************/

static int32_t getsockname_request(FAR struct altcom_socket_s *fsock,
                                   FAR struct getsockname_req_s *req) {
  int32_t ret;
  int32_t err;
  int32_t result;
  uint16_t reslen = 0;
  FAR struct apicmd_getsockname_s *cmd = NULL;
  FAR struct apicmd_getsocknameres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_sock_alloc_cmdandresbuff((FAR void **)&cmd, APICMDID_SOCK_GETSOCKNAME,
                                       GETSOCKNAME_REQ_DATALEN, (FAR void **)&res,
                                       GETSOCKNAME_RES_DATALEN)) {
    return GETSOCKNAME_REQ_FAILURE;
  }

  /* Fill the data */

  cmd->sockfd = htonl(req->sockfd);
  cmd->namelen = htonl(*req->addrlen);

  DBGIF_LOG2_DEBUG("[getsockname-req]sockfd: %d, addrlen: %lu\n", req->sockfd, *req->addrlen);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res, GETSOCKNAME_RES_DATALEN, &reslen,
                      ALTCOM_SYS_TIMEO_FEVR);
  if (ret < 0) {
    DBGIF_LOG1_ERROR("apicmdgw_send error: %ld\n", ret);
    err = -ret;
    goto errout_with_cmdfree;
  }

  if (reslen != GETSOCKNAME_RES_DATALEN) {
    DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
    err = ALTCOM_EFAULT;
    goto errout_with_cmdfree;
  }

  ret = ntohl(res->ret_code);
  err = ntohl(res->err_code);

  DBGIF_LOG2_DEBUG("[getsockname-res]ret: %ld, err: %ld\n", ret, err);

  if (APICMD_GETSOCKNAME_RES_RET_CODE_ERR == ret) {
    DBGIF_LOG1_ERROR("API command response is err :%ld.\n", err);
    goto errout_with_cmdfree;
  } else {
    DBGIF_LOG1_DEBUG("[getsockname-res]addrlen: %lu\n", ntohl(res->namelen));

    if (*req->addrlen < ntohl(res->namelen)) {
      DBGIF_LOG2_INFO("Input addrlen: %lu, Output addrlen: %lu\n", *req->addrlen,
                      ntohl(res->namelen));
    }
    memcpy(req->addr, &res->name, *req->addrlen);
    *req->addrlen = ntohl(res->namelen);
    result = GETSOCKNAME_REQ_SUCCESS;
  }

  altcom_sock_free_cmdandresbuff(cmd, res);

  return result;

errout_with_cmdfree:
  altcom_sock_free_cmdandresbuff(cmd, res);
  altcom_seterrno(err);
  return GETSOCKNAME_REQ_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_getsockname
 *
 * Description:
 *   The altcom_getsockname() function retrieves the locally-bound name of the
 *   specified socket, stores this address in the sockaddr structure pointed
 *   to by the 'addr' argument, and stores the length of this address in the
 *   object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket [in]
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address. Otherwise, -1 is returned and errno is set to indicate the error.
 *
 ****************************************************************************/

int altcom_getsockname(int sockfd, struct altcom_sockaddr *addr, altcom_socklen_t *addrlen) {
  int32_t result;
  FAR struct altcom_socket_s *fsock;
  struct getsockname_req_s req;

  if (!altcom_isinit()) {
    DBGIF_LOG_ERROR("Not intialized\n");
    altcom_seterrno(ALTCOM_ENETDOWN);
    return -1;
  }

  if ((!addr) || (!addrlen)) {
    DBGIF_LOG_ERROR("addr or addrlenis NULL\n");
    altcom_seterrno(ALTCOM_EINVAL);
    return -1;
  }

  if (*addrlen <= 0) {
    DBGIF_LOG1_ERROR("Invalide addrlen: %lu\n", *addrlen);
    altcom_seterrno(ALTCOM_EINVAL);
    return -1;
  }

  fsock = altcom_sockfd_socket(sockfd);
  if (!fsock) {
    altcom_seterrno(ALTCOM_EINVAL);
    return -1;
  }

  req.sockfd = sockfd;
  req.addr = addr;
  req.addrlen = addrlen;

  /* Send getsockname request */

  result = getsockname_request(fsock, &req);
  if (result == GETSOCKNAME_REQ_FAILURE) {
    return -1;
  }

  return 0;
}
