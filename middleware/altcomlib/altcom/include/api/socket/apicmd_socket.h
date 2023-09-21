/****************************************************************************
 * modules/lte/altcom/include/api/socket/apicmd_socket.h
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

#ifndef __MODULES_LTE_ALTCOM_INCLUDE_API_SOCKET_APICMD_SOCKET_H
#define __MODULES_LTE_ALTCOM_INCLUDE_API_SOCKET_APICMD_SOCKET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "altcom_socket.h"
#include "apicmd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_SOCKET_DOMAIN_UNSPEC (ALTCOM_AF_UNSPEC)
#define APICMD_SOCKET_DOMAIN_INET (ALTCOM_AF_INET)
#define APICMD_SOCKET_DOMAIN_INET6 (ALTCOM_AF_INET6)

#define APICMD_SOCKET_TYPE_SOCK_STREAM (ALTCOM_SOCK_STREAM)
#define APICMD_SOCKET_TYPE_SOCK_DGRAM (ALTCOM_SOCK_DGRAM)
#define APICMD_SOCKET_TYPE_SOCK_RAW (ALTCOM_SOCK_RAW)

#define APICMD_SOCKET_PROTOCOL_IP (ALTCOM_IPPROTO_IP)
#define APICMD_SOCKET_PROTOCOL_ICMP (ALTCOM_IPPROTO_ICMP)
#define APICMD_SOCKET_PROTOCOL_TCP (ALTCOM_IPPROTO_TCP)
#define APICMD_SOCKET_PROTOCOL_UDP (ALTCOM_IPPROTO_UDP)
#define APICMD_SOCKET_PROTOCOL_IPV6 (ALTCOM_IPPROTO_IPV6)
#define APICMD_SOCKET_PROTOCOL_ICMPV6 (ALTCOM_IPPROTO_ICMPV6)
#define APICMD_SOCKET_PROTOCOL_UDPLITE (ALTCOM_IPPROTO_UDPLITE)
#define APICMD_SOCKET_PROTOCOL_RAW (ALTCOM_IPPROTO_RAW)

#define APICMD_SOCKET_RES_RET_CODE_OK (0)
#define APICMD_SOCKET_RES_RET_CODE_ERR (-1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

begin_packed_struct struct apicmd_socket_s {
  int32_t domain;
  int32_t type;
  int32_t protocol;
} end_packed_struct;

begin_packed_struct struct apicmd_socketres_s {
  int32_t ret_code;
  int32_t err_code;
} end_packed_struct;

#endif /* __MODULES_LTE_ALTCOM_INCLUDE_API_SOCKET_APICMD_SOCKET_H */
