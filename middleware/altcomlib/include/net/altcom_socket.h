/****************************************************************************
 * modules/lte/include/net/altcom_socket.h
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
 * @file altcom_socket.h
 */

#ifndef __MODULES_LTE_INCLUDE_NET_ALTCOM_SOCKET_H
#define __MODULES_LTE_INCLUDE_NET_ALTCOM_SOCKET_H

/**
 * @defgroup net NET Connector APIs
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
 * @defgroup nsockets Number of sockets
 * Define the number of sockets
 * @{
 */

#define ALTCOM_NSOCKET 10

/** @} nsockets */

/**
 * @defgroup addrfamily Address family
 * Define the address family
 * @{
 */

#define ALTCOM_AF_UNSPEC 0
#define ALTCOM_AF_INET 2
#define ALTCOM_AF_INET6 10
#define ALTCOM_PF_INET ALTCOM_AF_INET
#define ALTCOM_PF_INET6 ALTCOM_AF_INET6
#define ALTCOM_PF_UNSPEC ALTCOM_AF_UNSPEC

/** @} addrfamily */

/* Socket protocol type */

/**
 * @defgroup sockproto Socket protocol type
 * Define the socket protocol type
 * @{
 */

#define ALTCOM_SOCK_STREAM 1
#define ALTCOM_SOCK_DGRAM 2
#define ALTCOM_SOCK_RAW 3

/** @} sockproto */

/**
 * @defgroup ipproto IP protocol
 * Define the IP protocol
 * @{
 */

#define ALTCOM_IPPROTO_IP 0
#define ALTCOM_IPPROTO_ICMP 1
#define ALTCOM_IPPROTO_TCP 6
#define ALTCOM_IPPROTO_UDP 17
#define ALTCOM_IPPROTO_IPV6 41
#define ALTCOM_IPPROTO_ICMPV6 58
#define ALTCOM_IPPROTO_UDPLITE 136
#define ALTCOM_IPPROTO_RAW 255

/** @} ipproto */

/* Flags */

#define ALTCOM_MSG_PEEK 0x01
#define ALTCOM_MSG_WAITALL 0x02
#define ALTCOM_MSG_OOB 0x04
#define ALTCOM_MSG_DONTWAIT 0x08
#define ALTCOM_MSG_MORE 0x10

/* How */

/**
 * @defgroup shuthow How to shutdown
 * Define how to shutdown through altcom_shutdown()
 * @{
 */

#define ALTCOM_SHUT_RD 0
#define ALTCOM_SHUT_WR 1
#define ALTCOM_SHUT_RDWR 2

/** @} shuthow */

/* Level */

/**
 * @defgroup olsocket Socket level option
 * Define the socket level option used in altcom_getsockopt()/altcom_setsockopt()
 * @{
 */

#define ALTCOM_SOL_SOCKET 0xfff

/** @} olsocket */

/* Option flags per-socket */

/**
 * @defgroup soflag Option flags per-socket
 * Definition of option flags per-socket
 * @{
 */

#define ALTCOM_SO_REUSEADDR 0x0004
#define ALTCOM_SO_KEEPALIVE 0x0008
#define ALTCOM_SO_BROADCAST 0x0020

/** @} soflag */

/**
 * @defgroup asoflag Additional options, not kept in so_options
 * Definition of additional options, not kept in so_options
 * @{
 */

#define ALTCOM_SO_ACCEPTCONN 0x0002
#define ALTCOM_SO_LINGER 0x0080
#define ALTCOM_SO_RCVBUF 0x1002
#define ALTCOM_SO_SNDTIMEO 0x1005
#define ALTCOM_SO_RCVTIMEO 0x1006
#define ALTCOM_SO_ERROR 0x1007
#define ALTCOM_SO_TYPE 0x1008
#define ALTCOM_SO_NO_CHECK 0x100a

/** @} asoflag */

/**
 * @defgroup olipprotoip Options for level IPPROTO_IP
 * Definition of options for level IPPROTO_IP
 * @{
 */

#define ALTCOM_IP_TOS 1
#define ALTCOM_IP_TTL 2

/** @} olipprotoip */

/**
 * @defgroup ipmember Options and types related to multicast membership
 * Definition of options and types related to multicast membership
 * @{
 */

#define ALTCOM_IP_ADD_MEMBERSHIP 3
#define ALTCOM_IP_DROP_MEMBERSHIP 4

/** @} ipmember */

/**
 * @defgroup ipmulticast Options and types for UDP multicast traffic handling
 * Definition of options and types for UDP multicast traffic handling
 * @{
 */

#define ALTCOM_IP_MULTICAST_TTL 5
#define ALTCOM_IP_MULTICAST_IF 6
#define ALTCOM_IP_MULTICAST_LOOP 7

/** @} ipmulticast */

/**
 * @defgroup olipprototcp Options for level ALTCOM_IPPROTO_TCP
 * Definition of options for level ALTCOM_IPPROTO_TCP
 * @{
 */

#define ALTCOM_TCP_NODELAY 0x01
#define ALTCOM_TCP_KEEPALIVE 0x02
#define ALTCOM_TCP_KEEPIDLE 0x03
#define ALTCOM_TCP_KEEPINTVL 0x04
#define ALTCOM_TCP_KEEPCNT 0x05

/** @} olipprototcp */

/**
 * @defgroup olipprotoipv6 Options for level ALTCOM_IPPROTO_IPV6
 * Definition of options for level ALTCOM_IPPROTO_IPV6
 * @{
 */

#define ALTCOM_IPV6_CHECKSUM 7
#define ALTCOM_IPV6_V6ONLY 27

/** @} olipprotoipv6 */

/**
 * @defgroup olipprotoudplite Options for level ALTCOM_IPPROTO_UDPLITE
 * Definition of options for level ALTCOM_IPPROTO_UDPLITE
 * @{
 */

#define ALTCOM_UDPLITE_SEND_CSCOV 0x01
#define ALTCOM_UDPLITE_RECV_CSCOV 0x02

/** @} olipprotoudplite */

/* Macros for fcntl */

/**
 * @defgroup fcntl Macros for fcntl
 * Definition of macros for fcntl
 * @{
 */

#define ALTCOM_GETFL 3
#define ALTCOM_SETFL 4

/** @} fcntl */

/**
 * @defgroup nonblock Non-blocking option
 * Definition of non-blocking option
 * @{
 */

#define ALTCOM_O_NONBLOCK 0x01

/** @} nonblock */

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint8_t altcom_sa_family_t;
typedef uint32_t altcom_socklen_t;
typedef uint16_t altcom_in_port_t;

/**
 * @struct altcom_sockaddr
 * Define the raw strucuture wrapper of @ref altcom_sockaddr_in
 */

struct altcom_sockaddr {
  uint8_t sa_len;
  altcom_sa_family_t sa_family;
  char sa_data[14];
};

struct altcom_sockaddr_storage {
  uint8_t s2_len;
  altcom_sa_family_t ss_family;
  char s2_data1[2];
  uint32_t s2_data2[3];
  uint32_t s2_data3[3];
};

struct altcom_linger {
  int l_onoff;
  int l_linger;
};

/**
 * @struct altcom_timeval
 * Define the time interval used in altcom_getsockopt()/altcom_setsockopt()/altcom_select()
 */

struct altcom_timeval {
  long tv_sec;
  long tv_usec;
};

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
 * Name: altcom_accept
 *
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
 *   @param [in] sockfd   The listening socket descriptor
 *   @param [out] addr     Receives the address of the connecting client
 *   @param [inout] addrlen  Input: allocated size of 'addr', Return: returned size of 'addr'
 *
 * @return
 *  Returns -1 on error. If it succeeds, it returns a non-negative integer
 *  that is a descriptor for the accepted socket.
 */

int altcom_accept(int sockfd, struct altcom_sockaddr *addr, altcom_socklen_t *addrlen);

/**
 * Name: altcom_bind
 *
 *   altcom_bind() gives the socket 'sockfd' the local address 'addr'.
 *   'addr' is 'addrlen' bytes long. Traditionally, this is called
 *   "assigning a name to a socket." When a socket is created with socket,
 *   it exists in a name space (address family) but has no name assigned.
 *
 *   @param [in] sockfd   Socket descriptor of the socket to bind
 *   @param [in] addr     Socket local address
 *   @param [in] addrlen  Length of 'addr'
 *
 * @return
 *   0 on success; -1 on error with errno set appropriately
 *
 */

int altcom_bind(int sockfd, const struct altcom_sockaddr *addr, altcom_socklen_t addrlen);

/**
 * Name: altcom_close
 *
 *   altcom_close() closes a file descriptor of the socket.
 *
 *   @param [in] sockfd   Socket descriptor to close
 *
 * @return
 *   0 on success; -1 on error with errno set appropriately.
 *
 */

int altcom_close(int sockfd);

/**
 * Name: altcom_connect
 *
 *   altcom_connect() connects the socket referred to by the file descriptor
 *   'sockfd' to the address specified by 'addr'. The addrlen argument
 *   specifies the size of 'addr'.  The format of the address in 'addr' is
 *   determined by the address space of the socket 'sockfd'.
 *
 *   If the socket 'sockfd' is of type ALTCOM_SOCK_DGRAM then 'addr' is the
 *   address to which datagrams are sent by default, and the only address
 *   from which datagrams are received. If the socket is of type
 *   ALTCOM_SOCK_STREAM this call attempts to make a connection to the socket
 *   that is bound to the address specified by 'addr'.
 *
 *   Generally, connection-based protocol sockets may successfully connect()
 *   only once; connectionless protocol sockets may use connect() multiple
 *   times to change their association.  Connectionless sockets may dissolve
 *   the association by connecting to an address with the sa_family member of
 *   sockaddr set to ALTCOM_AF_UNSPEC.
 *
 *   @param [in] sockfd    Socket descriptor returned by altcom_socket()
 *   @param [in] addr      Server address (form depends on type of socket)
 *   @param [in] addrlen   Length of actual 'addr'
 *
 * @return
 *   0 on success; -1 on error with errno set appropriately
 *
 */

int altcom_connect(int sockfd, const struct altcom_sockaddr *addr, altcom_socklen_t addrlen);

/**
 * Name: altcom_fcntl
 *
 *   Performs fcntl operations on socket
 *
 *   @param [in] sockfd - Socket descriptor of the socket to operate on
 *   @param [in] cmd    - The fcntl command.
 *   @param [in] val    - Command-specific arguments
 *
 * @return
 *   0 is returned on success; -1 is returned on failure and
 *   the errno value is set appropriately.
 *
 */

int altcom_fcntl(int sockfd, int cmd, int val);

/**
 * Name: altcom_getsockname
 *
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
 *   @param [in] sockfd   Socket descriptor of socket [in]
 *   @param [in] addr     sockaddr structure to receive data [out]
 *   @param [inout] addrlen  Length of sockaddr structure [in/out]
 *
 * @return
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address. Otherwise, -1 is returned and errno is set to indicate the error.
 *
 */

int altcom_getsockname(int sockfd, struct altcom_sockaddr *addr, altcom_socklen_t *addrlen);

/**
 * Name: altcom_getsockopt
 *
 *   altcom_getsockopt() retrieve thse value for the option specified by the
 *   'option' argument for the socket specified by the 'sockfd' argument. If
 *   the size of the option value is greater than 'value_len', the value
 *   stored in the object pointed to by the 'value' argument will be silently
 *   truncated. Otherwise, the length pointed to by the 'value_len' argument
 *   will be modified to indicate the actual length of the'value'.
 *
 *   The 'level' argument specifies the protocol level of the option. To
 *   retrieve options at the socket level, specify the level argument as
 *   ALTCOM_SOL_SOCKET.
 *
 *   @param [in] sockfd    Socket descriptor of socket
 *   @param [in] level     Protocol level to set the option
 *   @param [in] option    identifies the option to get
 *   @param [out] value     Points to the argument value
 *   @param [inout] value_len The length of the argument value
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 */

int altcom_getsockopt(int sockfd, int level, int option, void *value, altcom_socklen_t *value_len);

/**
 * Name: altcom_ioctl
 *
 *   Perform device specific operations.
 *
 *   @param [in] sockfd   Socket descriptor of device
 *   @param [in] req      The ioctl command
 *   @param [inout] argp     A third argument of type unsigned long is expected
 *
 * Return:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   -1 on failure with errno set properly.
 *
 */

int altcom_ioctl(int sockfd, long req, void *argp);

/**
 * Name: altcom_listen
 *
 *   To accept connections, a socket is first created with altcom_socket(), a
 *   willingness to accept incoming connections and a queue limit for incoming
 *   connections are specified with listen(), and then the connections are
 *   accepted with altcom_accept(). The altcom_listen() call applies only to
 *   sockets of type ALTCOM_SOCK_STREAM.
 *
 *   @param [in] sockfd   Socket descriptor of the bound socket
 *   @param [in] backlog  The maximum length the queue of pending connections may grow.
 *            If a connection request arrives with the queue full, the client
 *            may receive an error with an indication of ECONNREFUSED or,
 *            if the underlying protocol supports retransmission, the request
 *            may be ignored so that retries succeed.
 *
 * @return
 *   On success, zero is returned. On error, -1 is returned, and errno is set
 *   appropriately.
 *
 */

int altcom_listen(int sockfd, int backlog);

/**
 * Name: altcom_read
 *
 *   The altcom_read() is the same as altcom_recv() with the flags parameter
 *   set to zero.
 *
 *   @param [in] sockfd   File structure instance
 *   @param [out] buf      Buffer to receive data
 *   @param [in] len      Length of buffer
 *
 * @return
 *   On success, returns the number of characters received. On error,
 *   -1 is returned, and errno is set appropriately.
 *
 */

int altcom_read(int sockfd, void *buf, size_t len);

/**
 * Name: altcom_recv
 *
 *   The altcom_recv() call is identical to altcom_recvfrom() with a NULL
 *   from parameter.
 *
 *   @param [in] sockfd   Socket descriptor of socket
 *   @param [out] buf      Buffer to receive data
 *   @param [in] len      Length of buffer
 *   @param [in] flags    Receive flags
 *
 * Returned Value:
 *  (see recvfrom)
 *
 */

int altcom_recv(int sockfd, void *buf, size_t len, int flags);

/**
 * Name: altcom_recvfrom
 *
 *   altcom_recvfrom() receives messages from a socket, and may be used to
 *   receive data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 *   @param [in] sockfd   Socket descriptor of socket
 *   @param [out] buf      Buffer to receive data
 *   @param [in] len      Length of buffer
 *   @param [in] flags    Receive flags
 *   @param [in] from     Address of source (may be NULL)
 *   @param [in] fromlen  The length of the address structure
 *
 * @return
 *   On success, returns the number of characters received. On error,
 *   -1 is returned, and errno is set appropriately.
 *
 */

int altcom_recvfrom(int sockfd, void *buf, size_t len, int flags, struct altcom_sockaddr *from,
                    altcom_socklen_t *fromlen);

/**
 * Name: altcom_send
 *
 *   The altcom_send() call may be used only when the socket is in
 *   a connected state (so that the intended recipient is known). The only
 *   difference between altcom_send() and altcom_write() is the presence of
 *   flags. With zero flags parameter, altcom_send() is equivalent to
 *   altcom_write(). Also, send(sockfd,buf,len,flags) is equivalent to
 *   altcom_sendto(sockfd,buf,len,flags,NULL,0).
 *
 *   @param [in] sockfd   Socket descriptor of socket
 *   @param [in] buf      Data to send
 *   @param [in] len      Length of data to send
 *   @param [in] flags    Send flags
 *
 * @return
 *   On success, returns the number of characters sent. On error,
 *   -1 is returned, and errno is set appropriately.
 *
 */

int altcom_send(int sockfd, const void *buf, size_t len, int flags);

/**
 * Name: altcom_sendto
 *
 *   If altcom_sendto() is used on a connection-mode (ALTCOM_SOCK_STREAM)
 *   socket, the parameters to and 'tolen' are ignored (and the error EISCONN
 *   may be returned when they are not NULL and 0), and the error ENOTCONN is
 *   returned when the socket was not actually connected.
 *
 *   @param [in] sockfd   Socket descriptor of socket
 *   @param [in] buf      Data to send
 *   @param [in] len      Length of data to send
 *   @param [in] flags    Send flags
 *   @param [in] to       Address of recipient
 *   @param [in] tolen    The length of the address structure
 *
 * @return
 *   On success, returns the number of characters sent. On error,
 *   -1 is returned, and errno is set appropriately.
 *
 */

int altcom_sendto(int sockfd, const void *buf, size_t len, int flags,
                  const struct altcom_sockaddr *to, altcom_socklen_t tolen);

/**
 * Name: altcom_setsockopt
 *
 *   altcom_setsockopt() sets the option specified by the 'option' argument,
 *   at the protocol level specified by the 'level' argument, to the value
 *   pointed to by the 'value' argument for the socket associated with the
 *   file descriptor specified by the 'sockfd' argument.
 *
 *   The 'level' argument specifies the protocol level of the option. To set
 *   options at the socket level, specify the level argument
 *   as ALTCOM_SOL_SOCKET.
 *
 *   @param [in] sockfd    Socket descriptor of socket
 *   @param [in] level     Protocol level to set the option
 *   @param [in] option    identifies the option to set
 *   @param [in] value     Points to the argument value
 *   @param [in] value_len The length of the argument value
 *
 * @return
 *  0 on success; -1 on failure
 *
 */

int altcom_setsockopt(int sockfd, int level, int option, const void *value,
                      altcom_socklen_t value_len);

/**
 * Name: altcom_shutdown
 *
 *   The altcom_shutdown() function will cause all or part of a full-duplex
 *   connection on the socket associated with the file descriptor socket to
 *   be shut down.
 *
 *   The altcom_shutdown() function disables subsequent send and/or receive
 *   operations on a socket, depending on the value of the how argument.
 *
 *   @param [in] sockfd - Specifies the file descriptor of the socket.
 *   @param [in] how    - Specifies the type of shutdown. The values are as follows:
 *
 *     ALTCOM_SHUT_RD   - Disables further receive operations.
 *     ALTCOM_SHUT_WR   - Disables further send operations.
 *     ALTCOM_SHUT_RDWR - Disables further send and receive operations.
 *
 * @return
 *   Upon successful completion, altcom_shutdown() will return 0; otherwise,
 *   -1 will be returned and errno set to indicate the error.
 *
 */

int altcom_shutdown(int sockfd, int how);

/**
 * Name: altcom_socket
 *
 *   altcom_socket() creates an endpoint for communication and returns
 *   a descriptor.
 *
 *   @param [in] domain   See Address family.
 *   @param [in] type     See Socket protocol type.
 *   @param [in] protocol See Protocol.
 *
 * @return
 *   A non-negative socket descriptor on success; -1 on error with errno set
 *   appropriately.
 *
 */

int altcom_socket(int domain, int type, int protocol);

/**
 * Name: altcom_write
 *
 *   The altcom_write() is the same as altcom_send() with the flags parameter
 *   set to zero.
 *
 *   @param [in] sockfd   Socket descriptor of socket
 *   @param [in] buf      Data to write
 *   @param [in] len      Length of data to send
 *
 * @return
 *   On success, returns the number of characters sent. On error,
 *   -1 is returned, and errno is set appropriately.
 *
 */

int altcom_write(int sockfd, const void *buf, size_t len);

/** @} socket_funcs */

#undef EXTERN
#ifdef __cplusplus
}
#endif

/** @} net */

#endif /* __MODULES_LTE_INCLUDE_NET_ALTCOM_SOCKET_H */
