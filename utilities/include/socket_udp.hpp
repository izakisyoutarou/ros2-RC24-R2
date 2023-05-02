#pragma once
#include <sys/socket.h>
#include <netinet/in.h>
#include <thread>

class RecvUDP{
public:
    RecvUDP(const int16_t port);
    const bool is_recved();
    const unsigned char *data(unsigned char *data, size_t data_len);

private:
    void recv(int sockfd);

    int timeout_ms = 17;
    bool is_recved_ = false;

    int sockfd;
    socklen_t clilen;
    unsigned char buffer[1024];
    struct sockaddr_in servaddr, cliaddr;
    std::thread recv_thread;
};
