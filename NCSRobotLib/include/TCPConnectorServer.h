#ifndef TCP_CONNECTORSERVER_H
#define TCP_CONNECTORSERVER_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <errno.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <limits.h>


int getTCPFileDescriptor(const char* address, int port)
{
	// open socket
	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if(sockfd <= 0) {
		printf("dynapse_open: socket error %d\n", sockfd);
		return -1;
	}
	// prepare address
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	// inet pton
	int i = inet_pton(AF_INET, address, &addr.sin_addr);
	if(i <= 0) {
		printf("dynapse_open: inet p_ton error %d\n", i);
		return -1;
	}
	// connect
	if(connect(sockfd, (struct sockaddr*)&addr, sizeof(addr))) {
		printf("dynapse_open: connect error\n");
		return -1;
	}
	// return handle
	return sockfd;
}
#endif


