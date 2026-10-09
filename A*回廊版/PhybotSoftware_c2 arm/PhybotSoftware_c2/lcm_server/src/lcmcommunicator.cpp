#include "lcmcommunicator.h"

#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <iostream>

void PHYBOT_TOOL::LcmCommunicator::MessageHandler::onMessage(const lcm::ReceiveBuffer *rbuf, const std::string &ch)
{
    if (self != nullptr)
    {
        self->onMessageImpl(rbuf, ch);
    }
}

PHYBOT_TOOL::LcmCommunicator::LcmCommunicator(int send_port, char mode, std::string net_dev)
    : _run(true)
    , _handler()
{

    // if (isPortUsed(send_port))
    // {
    //     std::string str_error = "发送端口 " + send_port;
    //     str_error += " 被占用, 程序退出...";
    //     throw std::runtime_error(str_error);
    //     exit(-1);
    // }

    if(mode == 0x01)
        m_url = "udpm://239.255.76.67:" + std::to_string(send_port) + "?iface=" + net_dev;
    else if(mode == 0x02)
        m_url = "udpm://239.255.76.67:" + std::to_string(send_port) + "?ttl=1&ip==" + net_dev;
    else
    {
        throw std::runtime_error("绑定模式不正确,程序退出...");
        exit(-1);
    }

    //! \关联当前实例
    _handler.self = this;

    //! \构造lcm对象，并传入url参数
    _lcm = new lcm::LCM(m_url);

    if (!_lcm->good())
    {
        throw std::runtime_error("[LcmCommunicator] LCM初始化失败！");
        delete _lcm;
        exit(-1);
    }

    std::cout << "lcm url : " << m_url << std::endl;

    _recv_thread = std::thread(&LcmCommunicator::recv_loop, this);
    _dispatch_thread = std::thread(&LcmCommunicator::dispatch_loop, this);
}

PHYBOT_TOOL::LcmCommunicator::~LcmCommunicator()
{
    //! \停止线程
    _run.store(false, std::memory_order_relaxed);
    _cv.notify_all();
    if (_recv_thread.joinable()) {
        _recv_thread.join();
    }
    if (_dispatch_thread.joinable()) {
        _dispatch_thread.join();
    }

    std::lock_guard<std::mutex> lock(_mtx);
    for (std::unordered_map<std::string, lcm::Subscription*>::iterator it = _subscriptions.begin();
         it != _subscriptions.end(); ++it)
    {
        //! \取消订阅
        _lcm->unsubscribe(it->second);
    }
    _subscriptions.clear();

    delete _lcm;
    _lcm = nullptr;
}

void PHYBOT_TOOL::LcmCommunicator::subscribe(const std::string &channel)
{
    // std::lock_guard<std::mutex> lock(_mtx);

    //! \重复订阅检测
    if (_subscriptions.find(channel) != _subscriptions.end())
    {
        return;
    }

    //! \lcm 消息订阅并加载回调函数
    lcm::Subscription* sub = _lcm->subscribe(
                channel,
                &MessageHandler::onMessage,
                &_handler
                );

    //! \判定是否失败
    if (!sub) {
        throw std::runtime_error("[LcmCommunicator] 订阅失败：" + channel);
    }

    //! \保存订阅句柄
    _subscriptions[channel] = sub;

}

void PHYBOT_TOOL::LcmCommunicator::set_global_callback(GlobalLcmCallback cb)
{
    std::lock_guard<std::mutex> lock(_cb_mtx);
    _global_cb = std::move(cb);
}

void PHYBOT_TOOL::LcmCommunicator::unsubscribe(const std::string &channel)
{
    std::lock_guard<std::mutex> lock(_mtx);
    std::unordered_map<std::string, lcm::Subscription*>::iterator it = _subscriptions.find(channel);
    if (it != _subscriptions.end()) {
        _lcm->unsubscribe(it->second);
        _subscriptions.erase(it);
    }
}

bool PHYBOT_TOOL::LcmCommunicator::is_good() const
{
    std::lock_guard<std::mutex> lock(_mtx);
    return _lcm->good();
}

void PHYBOT_TOOL::LcmCommunicator::onMessageImpl(const lcm::ReceiveBuffer *rbuf, const std::string &ch)
{
    const uint8_t* data_ptr = static_cast<const uint8_t*>(rbuf->data);
    std::vector<uint8_t> lcm_msg(data_ptr, data_ptr + rbuf->data_size);
    std::lock_guard<std::mutex> q_lock(_q_mtx);
    _queue.emplace(ch, std::move(lcm_msg));
    _cv.notify_one();
}

void PHYBOT_TOOL::LcmCommunicator::recv_loop()
{
    while (_run.load(std::memory_order_relaxed)) {
        std::lock_guard<std::mutex> lock(_mtx);
        //! \非阻塞等待，避免线程卡死
        _lcm->handleTimeout(50);
    }
}

void PHYBOT_TOOL::LcmCommunicator::dispatch_loop()
{
    while (_run.load(std::memory_order_relaxed)) {
        std::unique_lock<std::mutex> lock(_q_mtx);
        _cv.wait(lock, [this]() {
            return !_run.load(std::memory_order_relaxed) || !_queue.empty();
        });

        if (!_run.load(std::memory_order_relaxed)) {
            return;
        }

        //! \取出消息并触发回调
        LcmMsgItem item = std::move(_queue.front());
        _queue.pop();
        lock.unlock();

        std::lock_guard<std::mutex> cb_lock(_cb_mtx);

        //! \回调发送
        if (_global_cb) {
            _global_cb(item.channel, item.data.data(), item.data.size());
        }
    }
}

bool PHYBOT_TOOL::LcmCommunicator::isPortUsed(int port)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return true;

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    //! \尝试绑定端口，失败则说明被占用
    bool used = bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0;
    close(sock);
    return used;
}

