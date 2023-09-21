/****************************************************************************
 * modules/lte/altcom/include/api/socket/altcom_sock.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_SOCKET_ALTCOM_SOCK_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_SOCKET_ALTCOM_SOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "altcom_socket.h"
#include "altcom_select.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALTCOM_SOCK_TIMEVAL2MS(ptv) ((ptv->tv_sec * 1000L) + (ptv->tv_usec / 1000L))

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct altcom_socket_s {
  uint8_t flags;
  struct altcom_timeval sendtimeo;
  struct altcom_timeval recvtimeo;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: altcom_sockfd_socket
 *
 * Description:
 *   Given a socket descriptor, return the underlying socket structure.
 *
 * Input Parameters:
 *   sockfd - The socket descriptor index to use.
 *
 * Returned Value:
 *   On success, a reference to the socket structure associated with
 *   the socket descriptor is returned.  NULL is returned on any failure.
 *
 ****************************************************************************/

struct altcom_socket_s *altcom_sockfd_socket(int sockfd);

/****************************************************************************
 * Name: altcom_sockaddr_to_sockstorage
 *
 * Description:
 *   Convert from sockaddr structure to sockaddr_storage structure
 *
 * Parameters:
 *   addr     sockaddr structure.
 *   storage  sockstorage structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void altcom_sockaddr_to_sockstorage(const struct altcom_sockaddr *addr,
                                    struct altcom_sockaddr_storage *storage);

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
                                    altcom_fd_set *exceptset);

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

int altcom_select_cancel_request_send(int id);

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
                           altcom_fd_set *exceptset);

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
                        altcom_fd_set *exceptset, struct altcom_timeval *timeout);

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_SOCKET_ALTCOM_SOCK_H */
