#ifndef UDPSOCKET_H
#define UDPSOCKET_H

#include <string>
#include <cstdint>
#include <memory>

#include <stdexcept>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
typedef int socklen_t;
#else
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>
#endif
using namespace std;

class UdpSocket {
public:

    UdpSocket();
    ~UdpSocket();

    /**
     * @brief 初始化UDP通信
     *
     * @param localPort 本地监听端口
     * @param remoteIp 远程目标IP地址
     * @param remotePort 远程目标端口
     * @return true 初始化成功 false 初始化失败，可通过GetLastError获取详情
     */

    bool InitUdp(uint16_t localPort = 0, string remoteIp = "", uint16_t remotePort = 0);


    /**
     * @brief 初始化UDP广播通信
     *
     * @param broadcastPort 本地监听广播端口
     * @param broadcastIp 广播IP地址
     * @return true 初始化成功 false 初始化失败，可通过GetLastError获取详情
     */
    
    bool InitBroadcastUdp(string broadcastIp, uint16_t broadcastPort);
    /**
     * @brief 关闭UDP通信
     *
     * @note 调用后需重新InitUdp才能继续通信
     */

    void CloseUdp();

    /**
     * @brief 发送数据到默认目标
     *
     * @param data 待发送数据指针
     * @param size 数据长度（字节）
     * @return true 发送成功 false 发送失败
     */

    bool Send(const char *data, size_t size);

    /**
     * @brief 发送数据到指定目标
     *
     * @param remoteIp 远程目标IP地址
     * @param remotePort 远程目标端口
     * @param data 待发送数据指针
     * @param size 数据长度（字节）
     * @return true 发送成功 false 发送失败
     */

    bool Send(const std::string &remoteIp, uint16_t remotePort, const char *data, size_t size);

    /**
     * @brief 接收UDP数据
     *
     * @param buffer 接收缓冲区
     * @param bufferSize 缓冲区大小
     * @param remoteIp 可选，用于存储发送方IP地址
     * @param remotePort 可选，用于存储发送方端口
     * @return int 实际接收的字节数，-1表示错误 ，-2表示文件描述符关闭
     */

    int Receive(char *buffer, size_t bufferSize, std::string *remoteIp = nullptr, uint16_t *remotePort = nullptr);

    /**
     * @brief 获取最后一次错误信息
     *
     * @return std::string 错误描述信息
     */

    std::string GetLastError() const;

private:
    int socketHandle;
    string lastError;
    sockaddr_in defaultRemoteAddr;
    bool hasDefaultRemote;
    bool sendToAddr(const sockaddr_in& addr, const char* data, size_t size);
};

#endif // UDPSOCKET_H    