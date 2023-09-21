/****************************************************************************
 * modules/lte/include/net/altcom_netdb.h
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
 * @file altcom_netdb.h
 */

#ifndef __MODULES_LTE_INCLUDE_NET_ALTCOM_NETDB_H
#define __MODULES_LTE_INCLUDE_NET_ALTCOM_NETDB_H

/**
 * @defgroup net NET Connector APIs
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "altcom_socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
 * @defgroup aiflag DNS address info flag
 * input flags for struct altcom_addrinfo
 * @{
 */

#define ALTCOM_AI_PASSIVE 0x01
#define ALTCOM_AI_CANONNAME 0x02
#define ALTCOM_AI_NUMERICHOST 0x04
#define ALTCOM_AI_NUMERICSERV 0x08
#define ALTCOM_AI_V4MAPPED 0x10
#define ALTCOM_AI_ALL 0x20
#define ALTCOM_AI_ADDRCONFIG 0x40

/** @} aiflag */

/**
 * @defgroup errnoai Error number of DNS APIs
 * Errors used by the DNS API functions, h_errno can be one of them
 * @{
 */

#define ALTCOM_EAI_NONAME 200
#define ALTCOM_EAI_SERVICE 201
#define ALTCOM_EAI_FAIL 202
#define ALTCOM_EAI_MEMORY 203
#define ALTCOM_EAI_FAMILY 204

#define ALTCOM_HOST_NOT_FOUND 210
#define ALTCOM_NO_DATA 211
#define ALTCOM_NO_RECOVERY 212
#define ALTCOM_TRY_AGAIN 213

/** @} errnoai */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @struct altcom_hostent
 * Definition of host information entry.
 */

struct altcom_hostent {
  char *h_name;     /**< The official name of the host */
  char **h_aliases; /**< An array of alternative names for the host, terminated by a null pointer */
  int h_addrtype; /**< The type of address; always ALTCOM_AF_INET or ALTCOM_AF_INET6 at present. */
  int h_length;   /**< The length of the address in bytes */
  char **h_addr_list; /**< An array of pointers to network addresses for the host (in network byte
                         order), terminated by a null pointer */
#define h_addr h_addr_list[0]
};

/**
 * @struct altcom_addrinfo
 * Definition of address translation information.
 */

struct altcom_addrinfo {
  int ai_flags;  /**< This field specifies additional options, described below.  Multiple flags are
                    specified by bitwise OR-ing them together */
  int ai_family; /**< This field specifies the desired address family for the returned addresses.
                    Valid values for this field include ALTCOM_AF_INET and ALTCOM_AF_INET6 */
  int ai_socktype; /**< This field specifies the preferred socket type, for example
                      ALTCOM_SOCK_STREAM or ALTCOM_SOCK_DGRAM */
  int ai_protocol; /**< This field specifies the protocol for the returned socket addresses */
  altcom_socklen_t ai_addrlen;     /**< The length of the socket address ai_addr, in bytes */
  struct altcom_sockaddr *ai_addr; /**< A pointer to the socket address */
  char *ai_canonname; /**< The canonical name of the node corresponding to the  altcom_addrinfo */
  struct altcom_addrinfo *ai_next; /**< The next item in the linked list */
};

/**
 * @var altcom_h_errno
 * Declaration of application accessible error code set by the DNS API functions.
 */

extern int altcom_h_errno;

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
 * @defgroup dns_funcs DNS APIs
 * @{
 */

/**
 * Name: altcom_freeaddrinfo
 *
 *   The altcom_freeaddrinfo() function frees the memory that was allocated
 *   for the dynamically allocated linked list res.
 *
 *   @param [in] res - The res returned by altcom_getaddrinfo()
 *
 *   @return None.
 *
 */

void altcom_freeaddrinfo(struct altcom_addrinfo *res);

/**
 * Name: altcom_getaddrinfo
 *
 *   Given node and service, which identify an Internet host and a service,
 *   altcom_getaddrinfo() returns one or more addrinfo structures, each of
 *   which contains an Internet address that can be specified in a call to
 *   altcom_bind() or altcom_connect(). The altcom_getaddrinfo() is reentrant
 *   and allows programs to eliminate IPv4-versus-IPv6 dependencies.
 *
 *   @param [in] nodename - Specifies either a numerical network address, or a network
 *              hostname, whose network addresses are looked up and resolved
 *   @param [in] servname - Sets the port in each returned address structure
 *   @param [in] hints - Points to an addrinfo structure that specifies criteria for
 *           selecting the socket address structures returned in the list
 *           pointed to by res
 *   @param [out] res - Pointer to the start of the list
 *
 * Returned Value:
 *  altcom_getaddrinfo() returns 0 if it succeeds, or one of the following
 *  nonzero error codes:
 *
 *     ALTCOM_EAI_NONAME
 *     ALTCOM_EAI_SERVICE
 *     ALTCOM_EAI_FAIL
 *     ALTCOM_EAI_MEMORY
 *     ALTCOM_EAI_FAMILY
 *
 */

int altcom_getaddrinfo(const char *nodename, const char *servname,
                       const struct altcom_addrinfo *hints, struct altcom_addrinfo **res);

/**
 * Name: altcom_gethostbyname
 *
 *   The altcom_gethostbyname() function returns a structure of type hostent
 *   for the given host name. Here name is either a hostname, or an IPv4
 *   address in standard dot notation (as for inet_addr(3)), or an IPv6
 *   address in colon (and possibly dot) notation.
 *
 *   If name is an IPv4 or IPv6 address, no lookup is performed and
 *   altcom_gethostbyname() simply copies name into the h_name field
 *   and its struct in_addr equivalent into the h_addr_list[0] field of the
 *   returned hostent structure.
 *
 *   @param [in] name - The name of the host to find.
 *
 *   @return Upon successful completion, this function will return a pointer to a
 *   hostent structure if the requested entry was found, and a null pointer
 *   if the end of the database was reached or the requested entry was not
 *   found.
 *
 *   Upon unsuccessful completion, altcom_gethostbyname() will set h_errno to
 *   indicate the error
 *
 */

struct altcom_hostent *altcom_gethostbyname(const char *name);

/**
 * Name: altcom_gethostbyname_r
 *
 *   A reentrant version of altcom_gethostbyname()
 *   The altcom_gethostbyname_r() function returns a structure of type
 *   hostent for the given host name. Here name is either a hostname, or an
 *   IPv4 address in standard dot notation (as for inet_addr(3)), or an IPv6
 *   address in colon (and possibly dot) notation.
 *
 *   If name is an IPv4 or IPv6 address, no lookup is performed and
 *   altcom_gethostbyname_r() simply copies name into the h_name field
 *   and its struct in_addr equivalent into the h_addr_list[0] field of the
 *   returned hostent structure.
 *
 *   @param [in] name - The name of the host to find.
 *   @param [out] ret - Caller provided location to return the host data.
 *   @param [out] buf - Caller provided buffer to hold string data associated with the
 *     host data.
 *   @param [in] buflen - The size of the caller-provided buffer
 *   @param [out] result - Point to the result on success.
 *   @param [out] h_errnop - There h_errno value returned in the event of a failure.
 *
 * Returned Value:
 *   0 is returned on success, -1 is returned on a failure
 *   with the returned h_errno value provided the reason for the failure.
 *
 */

int altcom_gethostbyname_r(const char *name, struct altcom_hostent *ret, char *buf, size_t buflen,
                           struct altcom_hostent **result, int *h_errnop);

/** @} dns_funcs */

#undef EXTERN
#ifdef __cplusplus
}
#endif

/** @} net */

#endif /* __MODULES_LTE_INCLUDE_NET_ALTCOM_NETDB_H */
