#include "socket_udp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sys/time.h>
#include <sys/types.h>

RecvUDP::RecvUDP(const int16_t port){
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(port);
    bind(sockfd, (struct sockaddr *) &servaddr, sizeof(servaddr));
    recv_thread = std::thread(&RecvUDP::recv, this, sockfd);

}

const unsigned char *RecvUDP::data(unsigned char *data){
    memcpy(data, buffer, sizeof(data));
    return data;
}

const bool RecvUDP::is_recved(){
    if(is_recved_){
        is_recved_ = false;
        return true;
    }
    return false;
}

void RecvUDP::recv(int sockfd){
    struct timeval tv;
    tv.tv_usec = timeout_ms;

    while(rclcpp::ok())
    {
        clilen = sizeof(cliaddr);

        // ノンブロッキングモードでrecvfromを呼び出す
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(sockfd, &read_fds);
        int sel = select(sockfd + 1, &read_fds, NULL, NULL, &tv);
        if (sel == -1){
            perror("select");
            continue;
        }
        else if (sel == 0){
            // タイムアウトした場合、再試行
            continue;
        }

        // bufferに受信したデータが格納されている
        int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *) &cliaddr, &clilen);

        if(n > 0) is_recved_ = true;

        if (n < 0){
            perror("recvfrom");
            continue;
        }

        if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv)) < 0){
            perror("setsockopt");
            continue;
        }
    }
}
