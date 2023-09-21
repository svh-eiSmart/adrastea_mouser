/****************************************************************************
 * modules/lte/altcom/api/socket/altcom_accept.c
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
#include "apicmd_accept.h"
#include "buffpoolwrapper.h"
#include "apiutil.h"
#include "altcom_cc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ACCEPT_REQ_DATALEN (sizeof(struct apicmd_accept_s))
#define ACCEPT_RES_DATALEN (sizeof(struct apicmd_acceptres_s))

#define ACCEPT_REQ_SUCCESS 0
#define ACCEPT_REQ_FAILURE -1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct accept_req_s {
  int sockfd;
  FAR struct altcom_sockaddr *addr;
  FAR altcom_socklen_t *addrlen;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: accept_request
 *
 * Description:
 *   Send ALTCOM_ACCEPT_REQ.
 *
 ****************************************************************************/

static int32_t accept_request(FAR struct altcom_socket_s *fsock, FAR struct accept_req_s *req) {
  int32_t ret;
  int32_t err;
  int32_t result;
  uint16_t reslen = 0;
  FAR struct apicmd_accept_s *cmd = NULL;
  FAR struct apicmd_acceptres_s *res = NULL;

  /* Allocate send and response command buffer */

  if (!altcom_sock_alloc_cmdandresbuff((FAR void **)&cmd, APICMDID_SOCK_ACCEPT, ACCEPT_REQ_DATALEN,
                                       (FAR void **)&res, ACCEPT_RES_DATALEN)) {
    return ACCEPT_REQ_FAILURE;
  }

  /* Fill the data */

  cmd->sockfd = htonl(req->sockfd);
  if (req->addrlen) {
    cmd->addrlen = htonl(*req->addrlen);
  } else {
    cmd->addrlen = htonl(0);
  }

  DBGIF_LOG2_DEBUG("[accept-req]sockfd: %d, addrlen: %lu\n", req->sockfd, cmd->addrlen);

  /* Send command and block until receive a response */

  ret = apicmdgw_send((FAR uint8_t *)cmd, (FAR uint8_t *)res, ACCEPT_RES_DATALEN, &reslen,
                      ALTCOM_SYS_TIMEO_FEVR);
  if (ret < 0) {
    DBGIF_LOG1_ERROR("apicmdgw_send error: %ld\n", ret);
    err = -ret;
    goto errout_with_cmdfree;
  }

  if (reslen != ACCEPT_RES_DATALEN) {
    DBGIF_LOG1_ERROR("Unexpected response data length: %d\n", reslen);
    err = ALTCOM_EFAULT;
    goto errout_with_cmdfree;
  }

  ret = ntohl(res->ret_code);
  err = ntohl(res->err_code);

  DBGIF_LOG2_DEBUG("[accept-res]ret: %ld, err: %ld\n", ret, err);

  if (APICMD_ACCEPT_RES_RET_CODE_ERR == ret) {
    DBGIF_LOG1_ERROR("API command response is err :%ld.\n", err);
    goto errout_with_cmdfree;
  } else {
    DBGIF_LOG1_DEBUG("[accept-res]addrlen: %lu\n", ntohl(res->addrlen));

    if (req->addr) {
      if (req->addrlen) {
        if (*req->addrlen < ntohl(res->addrlen)) {
          DBGIF_LOG2_INFO("Input addrlen: %lu, Output addrlen: %lu\n", *req->addrlen,
                          ntohl(res->addrlen));
        }
        memcpy(req->addr, &res->address, *req->addrlen);
      } else {
        DBGIF_LOG_ERROR("Unexpected. addrlen is NULL.\n");
      }
    }
    if (req->addrlen) {
      *req->addrlen = ntohl(res->addrlen);
    }
    result = ret;
  }

  altcom_sock_free_cmdandresbuff(cmd, res);

  return result;

errout_with_cmdfree:
  altcom_sock_free_cmdandresbuff(cmd, res);
  altcom_seterrno(err);
  return ACCEPT_REQ_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_accept
 *
 * Description:
 *   The altcom_accept function is used with connection-based socket types
 *   (ALTCOM_SOCK_STREAM). It extracts the first
 *   connection request on the queue of pending connections, creates a new
 *   connected socket with mostly the same properties as 'sockfd', and
 *   allocates a new socket descriptor for the socket, which is returned. The
 *   newly created socket is no longer in the listening state. The original
 *   socket 'sockfd' is unaffected by this call.  Per file descriptor flags
 *   are not inherited across an accept.
 *
 *   The 'sockfd' argument is a socket descriptor that has been created with
 *   altcom_socket(), bound to a local address with altcom_bind(), and is
 *   listening for connections after a call to altcom_listen().
 *
 *   On return, the 'addr' structure is filled in with the address of the
 *   connecting entity. The 'addrlen' argument initially contains the size
 *   of the structure pointed to by 'addr'; on return it will contain the
 *   actual length of the address returned.
 *
 *   If no pending connections are present on the queue, and the socket is
 *   not marked as non-blocking, accept blocks the caller until a connection
 *   is present. If the socket is marked non-blocking and no pending
 *   connections are present on the queue, accept returns EAGAIN.
 *
 * Parameters:
 *   sockfd   The listening socket descriptor
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr', Return: returned size of 'addr'
 *
 * Returned Value:
 *  Returns -1 on error. If it succeeds, it returns a non-negative integer
 *  that is a descriptor for the accepted socket.
 *
 ****************************************************************************/

int altcom_accept(int sockfd, struct altcom_sockaddr *addr, altcom_socklen_t *addrlen) {
  int32_t ret;
  int32_t result;
  FAR struct altcom_socket_s *fsock;
  struct altcom_fd_set_s readset;
  struct accept_req_s req;
  FAR struct altcom_timeval *recvtimeo;

  if (!altcom_isinit()) {
    DBGIF_LOG_ERROR("Not intialized\n");
    altcom_seterrno(ALTCOM_ENETDOWN);
    return -1;
  }

  if (addr && (!addrlen)) {
    DBGIF_LOG_ERROR("addrlen is NULL\n");
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

  if (fsock->flags & ALTCOM_O_NONBLOCK) {
    ALTCOM_FD_ZERO(&readset);
    ALTCOM_FD_SET(sockfd, &readset);

    ret = altcom_select_nonblock((sockfd + 1), &readset, NULL, NULL);
    if (ret > 0) {
      if (!ALTCOM_FD_ISSET(sockfd, &readset)) {
        altcom_seterrno(ALTCOM_EFAULT);
        DBGIF_LOG1_ERROR("select failed: %ld\n", altcom_errno());
        return -1;
      }

      /* Send accept request */

      result = accept_request(fsock, &req);
      if (result == ACCEPT_REQ_FAILURE) {
        return -1;
      }
    } else {
      if (ret == 0) {
        altcom_seterrno(ALTCOM_EAGAIN);
      } else {
        DBGIF_LOG1_ERROR("select failed: %ld\n", altcom_errno());
      }
      return -1;
    }
  } else {
    /* Wait until a connection is present */

    ALTCOM_FD_ZERO(&readset);
    ALTCOM_FD_SET(sockfd, &readset);

    recvtimeo = &fsock->recvtimeo;
    if ((fsock->recvtimeo.tv_sec == 0) && (fsock->recvtimeo.tv_usec == 0)) {
      recvtimeo = NULL;
    }

    ret = altcom_select_block((sockfd + 1), &readset, NULL, NULL, recvtimeo);
    if (ret <= 0) {
      if (ret == 0) {
        altcom_seterrno(ALTCOM_EFAULT);
      }

      if (altcom_errno() == ALTCOM_ETIMEDOUT) {
        altcom_seterrno(ALTCOM_EAGAIN);
      }
      DBGIF_LOG1_ERROR("select failed: %ld\n", altcom_errno());
      return -1;
    }

    if (!ALTCOM_FD_ISSET(sockfd, &readset)) {
      altcom_seterrno(ALTCOM_EFAULT);
      DBGIF_LOG1_ERROR("select failed: %ld\n", altcom_errno());
      return -1;
    }

    /* Send accept request */

    result = accept_request(fsock, &req);
    if (result == ACCEPT_REQ_FAILURE) {
      return -1;
    }
  }

  return result;
}
