#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <signal.h>
#include <atomic>

#define SERVER_IP "192.168.18.173"  // 服务器IP地址（本地测试用环回地址）
#define PORT 8080               // 服务器端口号
#define BUFFER_SIZE 1024

class UdpClient {
public:
    int client_fd;
    struct sockaddr_in server_addr;
    socklen_t server_len;
    char buffer[BUFFER_SIZE] = {0};
    UdpClient()
    {
        if (!open_socket()) {
            std::cout << "打开套接字失败" << std::endl;
            return;
        }
        std::cout << "打开套接字成功" << std::endl;
    }
    ~UdpClient()
    {
        close(client_fd);
    }

    int open_socket()
    {
        server_len = sizeof(server_addr);
        std::string message;

        // 1. 创建UDP套接字
        if ((client_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            perror("socket creation failed");
            return 0;
            // exit(EXIT_FAILURE);
        }

        // 2. 配置服务器地址结构
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(PORT);

        // 转换IP地址（字符串转网络字节序）
        if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) {
            perror("invalid server IP address");
            close(client_fd);
            // exit(EXIT_FAILURE);
            return 0;
        }
        return 1;

    }

    int send_msg(std::string message)
    {
        // std::string message;
        // std::string message = std::to_string(send_msg);

        // clock_gettime(CLOCK_REALTIME, &start); // 获取当前时间
        // struct tm tm_info;
        // char time_str[30];
        // localtime_r(&start.tv_sec, &tm_info); // 转换为本地时间
        // strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &tm_info);
        // int32_t sec = static_cast<int32_t>(start.tv_sec);
        // int32_t nsec = static_cast<uint32_t>(start.tv_nsec);

        // 4. 发送数据到服务器
        ssize_t send_len = sendto(
            client_fd,
            message.c_str(),
            message.length(),
            0,
            (const struct sockaddr *)&server_addr,
            server_len
        );
        if (send_len < 0) {
            perror("sendto failed");
            return 0;
        }

        memset(buffer, 0, BUFFER_SIZE);
        return 1;
    }
};

