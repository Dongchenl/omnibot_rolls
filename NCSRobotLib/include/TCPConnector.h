#ifndef TCP_CONNECTOR_H
#define TCP_CONNECTOR_H

int getTCPFileDescriptor(const char* hostname, unsigned int portno) {
    int sockfd;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)  {
      std::cout<<"ERROR opening socket"<<std::endl;
      return -1;
    }
    server = gethostbyname(hostname);
    if (server == NULL) {
      std::cout<<"ERROR, no such host"<<std::endl;
      return -1;
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
            (char *)&serv_addr.sin_addr.s_addr,
            server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) {
        std::cout<<"Can't create Filedescriptor"<<std::endl;
        }
        return -1;

    return sockfd;
}
#endif
