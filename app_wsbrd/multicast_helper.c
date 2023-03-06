#include <stdbool.h>
#include <stdint.h>
#include <sys/time.h>
#include "common/log.h"

#include "multicast_helper.h"
#include "nanostack/socket_api.h"
#include "nanostack/net_interface.h"
#include "common/os_types.h"
#include "common/os_timer.h"
#include "mbed-client-libservice/mbed-client-libservice/ip6string.h"

#define UDP_PORT 1234

uint8_t receive_buffer[20];
uint8_t sid = 0;
ns_address_t addr_send;
uint8_t multi_cast_addr[16] = {0};

#define mulapp "mulapp"


void handle_incoming_msg() {
    // WARN("Multicast msg hnadler called %s", receive_buffer);
    // Read data from the socket
    ns_address_t source_addr;
    memset(receive_buffer, 0, sizeof(receive_buffer));
    bool something_in_socket=true;
    // read all messages
    while (something_in_socket) {
        int length = socket_recvfrom(sid, &receive_buffer, sizeof(receive_buffer) - 1,NS_MSG_TRUNC, &source_addr);
        if (length > 0) {
            char ip6str[40];
            ip6tos(source_addr.address, ip6str);
            INFO("%s Packet from %s", mulapp, ip6str);
            struct timeval tv;
            gettimeofday(&tv,NULL);
            int embedded_tm;
            sscanf((char*)receive_buffer,"%d",&embedded_tm);
            int time_diff = (int)tv.tv_sec - embedded_tm;
            INFO("%s Received: %s time_now: %d time_diff: %d", mulapp, receive_buffer, (int)tv.tv_sec, time_diff);
        }
        else if (length!=NS_EWOULDBLOCK) {
            INFO("%s Error happened when receiving %d", mulapp, length);
            something_in_socket=false;
        }
        else {
            // there was nothing to read.
            something_in_socket=false;
        }
    }
}

static void send_message() {
    INFO("%s Create multicast msg", mulapp);

    int length;
    static int internal_seq = 0;
    struct timeval tv;
    gettimeofday(&tv,NULL);
    char buf[12+4];
    length = snprintf(buf, sizeof(buf), "%09d %04x", (int)tv.tv_sec, internal_seq);
    internal_seq++;

    INFO("%s Sending multicast message, %d bytes: %s", mulapp, length, buf);
    socket_sendto(sid, &addr_send, buf, length);
    //After message is sent, it is received from the network
}


void multicast_start() {
    stoip6(multicast_addr_str, strlen(multicast_addr_str), multi_cast_addr);
    sid = socket_open(SOCKET_UDP, UDP_PORT, &handle_incoming_msg);
    addr_send.type = ADDRESS_IPV6;  addr_send.identifier = UDP_PORT;
    memcpy(&addr_send.address, &multi_cast_addr, 16);
   
    ns_address_t bind_port = {.type = ADDRESS_IPV6, .identifier = UDP_PORT, .address = {0}};
    WARN("%s Openning a socket is %s", mulapp, sid > -1 ? "Successful" : "failed");
    int bind_ret = socket_bind(sid, &bind_port);
    WARN("%s Binding a socket is %s %d", mulapp, bind_ret > -1 ? "Successful" : "failed", bind_ret);
    static const int16_t multicast_hops = 10;
    socket_setsockopt(sid, SOCKET_IPPROTO_IPV6, SOCKET_IPV6_MULTICAST_HOPS, &multicast_hops, sizeof(multicast_hops));
    ns_ipv6_mreq_t mreq;
    memcpy(mreq.ipv6mr_multiaddr, multi_cast_addr, 16);
    mreq.ipv6mr_interface = 0;
    socket_setsockopt(sid, SOCKET_IPPROTO_IPV6, SOCKET_IPV6_JOIN_GROUP, &mreq, sizeof(mreq));
    static const int32_t rcvbuf_size = 2048;
    socket_setsockopt(sid, SOCKET_SOL_SOCKET, SOCKET_SO_RCVBUF, &rcvbuf_size, sizeof rcvbuf_size);
}

int init = 0;
void mul_tasklet(int timer_id, uint16_t slots)
{
    eventOS_callback_timer_start_sec(timer_id, 30);
    if(!init) {
        init = 1;
        multicast_start();
    }
    send_message();
}