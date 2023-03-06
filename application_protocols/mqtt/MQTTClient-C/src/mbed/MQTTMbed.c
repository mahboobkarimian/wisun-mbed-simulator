/**
 * \author Eric Tran
*/

#include "MQTTMbed.h"
#include "mbed-client-libservice/ip6string.h"
#include "common/log.h"
#include "mbed-client-libservice/ns_trace.h"

int rpl_socket_id = -1;
uint8_t rpl_tcp_buf[20];



void TimerInit(Timer* timer)
{
	timer->end_time = (struct timeval){0, 0};
}

char TimerIsExpired(Timer* timer)
{
	struct timeval now, res;
	gettimeofday(&now, NULL);
	timersub(&timer->end_time, &now, &res);
	return res.tv_sec < 0 || (res.tv_sec == 0 && res.tv_usec <= 0);
}


void TimerCountdownMS(Timer* timer, unsigned int timeout)
{
	struct timeval now;
	gettimeofday(&now, NULL);
	struct timeval interval = {timeout / 1000, (timeout % 1000) * 1000};
	timeradd(&now, &interval, &timer->end_time);
}


void TimerCountdown(Timer* timer, unsigned int timeout)
{
	struct timeval now;
	gettimeofday(&now, NULL);
	struct timeval interval = {timeout, 0};
	timeradd(&now, &interval, &timer->end_time);
}


int TimerLeftMS(Timer* timer)
{
	struct timeval now, res;
	gettimeofday(&now, NULL);
	timersub(&timer->end_time, &now, &res);
	//printf("left %d ms\n", (res.tv_sec < 0) ? 0 : res.tv_sec * 1000 + res.tv_usec / 1000);
	return (res.tv_sec < 0) ? 0 : res.tv_sec * 1000 + res.tv_usec / 1000;
}


int mbed_read(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
	struct timeval interval = {timeout_ms / 1000, (timeout_ms % 1000) * 1000};
	if (interval.tv_sec < 0 || (interval.tv_sec == 0 && interval.tv_usec <= 0))
	{
		interval.tv_sec = 0;
		interval.tv_usec = 100;
	}

	//socket_setsockopt(n->my_socket, SOCKET_SOL_SOCKET, SO_RCVTIMEO, (char *)&interval, sizeof(struct timeval));

	int bytes = 0;
	while (bytes < len)
	{
		//int rc = recv(n->my_socket, &buffer[bytes], (size_t)(len - bytes), 0);
		int rc = socket_recv(n->my_socket, &buffer[bytes], (size_t)(len - bytes), 0);
		//INFO("Mbed data read size : %d", rc);
		if (rc == -1)
		{
			if (errno != EAGAIN && errno != EWOULDBLOCK)
			  bytes = -1;
			break;
		}
		else if (rc == 0)
		{
			bytes = 0;
			break;
		}/*else if(rc == NS_EWOULDBLOCK){
			continue;
			//INFO("Mbed : No data");
		}*/
		else
			bytes += rc;

	}
	return bytes;
}


int mbed_write(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
	struct timeval tv;

	tv.tv_sec = 0;  /* 30 Secs Timeout */
	tv.tv_usec = timeout_ms * 1000;  // Not init'ing this can cause strange errors
	//setsockopt(n->my_socket, SOL_SOCKET, SO_SNDTIMEO, (char *)&tv,sizeof(struct timeval));
	//socket_setsockopt(n->my_socket, SOCKET_SOL_SOCKET, SO_SNDTIMEO, (char *)&tv,sizeof(struct timeval));
	//int rc = socket_send_to(n->my_socket, &n->dest_add, buffer, len);
	int	rc = socket_send(n->my_socket, buffer, len);
	INFO("mbed_write : %d", rc);
	if(rc == 0) {
		return len;
	}
	return rc;
}


void NetworkInit(Network* n)
{
	n->my_socket = 0;
	n->mqttread = mbed_read;
	n->mqttwrite = mbed_write;
}

int NetworkConnect6(Network* n, char* addr, int port)
{
	int rc = -1;
	uint8_t ipv6Address[16] = {0};
	unsigned char buffer[10] = {0};

	if(!stoip6(addr, strlen(addr), &ipv6Address)){
		INFO("Cannot convert IPv6 string to uint8 array");
		return -1;
	}
	INFO("MQTT Broker IP copied : %s", trace_array(ipv6Address, 16));
	INFO("MQTT Broker IP: %s", addr);

	ns_address_t ns_address = {
		.type = ADDRESS_IPV6,
		.address = {0},
		.identifier = port
	};

	memcpy(ns_address.address, &ipv6Address, 16);

	n->dest_add = &ns_address.address;
	n->my_socket = socket_open(SOCKET_TCP, 0, rpl_handle_tcp_msg);
	rpl_socket_id = n->my_socket;
	INFO("MQTT Broker IP in struct : %s", trace_array(ns_address.address, 16)); 
	INFO("Socket ID : %d", n->my_socket);
	if( (rc = socket_connect(n->my_socket, &ns_address, NULL)) < 0){
		INFO("Cannot connect to remote host, code : %d ", rc);
		return -1;
	}
	INFO("Remote connection to host : %d, port : %d", rc, ns_address.identifier);
	//sleep(2);
	//rc = socket_send(n->my_socket, buffer, strlen(buffer));
	//INFO("Send package after TCP conection : %d", rc);
	
	return 0;
}


/*
int NetworkConnect(Network* n, char* addr, int port)
{
	int type = SOCK_STREAM;
	struct sockaddr_in address;
	int rc = -1;
	sa_family_t family = AF_INET;
	struct addrinfo *result = NULL;
	struct addrinfo hints = {0, AF_UNSPEC, SOCK_STREAM, IPPROTO_TCP, 0, NULL, NULL, NULL};

	if ((rc = getaddrinfo(addr, NULL, &hints, &result)) == 0)
	{
		struct addrinfo* res = result;

		
		while (res)
		{
			if (res->ai_family == AF_INET)
			{
				result = res;
				break;
			}
			res = res->ai_next;
		}

		if (result->ai_family == AF_INET)
		{
			address.sin_port = htons(port);
			address.sin_family = family = AF_INET;
			address.sin_addr = ((struct sockaddr_in*)(result->ai_addr))->sin_addr;
		}
		else
			rc = -1;

		freeaddrinfo(result);
	}

	if (rc == 0)
	{
		n->my_socket = socket(family, type, 0);
		if (n->my_socket != -1)
			rc = connect(n->my_socket, (struct sockaddr*)&address, sizeof(address));
		else
			rc = -1;
	}

	return rc;
}
*/
/*
int NetworkConnect6(Network* n, char* addr, int port)
{
	int type = SOCK_STREAM;
	struct sockaddr_in6 address6;
	int rc = -1;
	sa_family_t family = AF_INET6;
	struct addrinfo *result = NULL;
	struct addrinfo hints = {0, AF_UNSPEC, SOCK_STREAM, IPPROTO_TCP, 0, NULL, NULL, NULL};

	if ((rc = getaddrinfo(addr, NULL, &hints, &result)) == 0)
	{
		struct addrinfo* res = result;

		if (result->ai_family == AF_INET6)
		{
			address6.sin6_port = htons(port);
			address6.sin6_family = family = AF_INET6;
			address6.sin6_addr = ((struct sockaddr_in6*)(result->ai_addr))->sin6_addr;
		}
		else
			rc = -1;

		freeaddrinfo(result);
	}

	if (rc == 0)
	{
		n->my_socket = socket(family, type, 0);
		if (n->my_socket != -1)
			rc = connect(n->my_socket, (struct sockaddr*)&address6, sizeof(address6));
		else
			rc = -1;
	}

	return rc;
}
*/


void NetworkDisconnect(Network* n)
{
	close(n->my_socket);
}

