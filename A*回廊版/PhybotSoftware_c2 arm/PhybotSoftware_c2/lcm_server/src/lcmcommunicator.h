//! \brief          lcm发收封装类
//! \name           sxl
//! \date           2026年3月2日
///////////////////////////////////////////////////////////////
//
//                  2026年3月14日
//                  更新绑定固定网卡功能        sxl
//-----------------------------------------------------------------
//

#ifndef __LCMCOMMUNICATOR_H__
#define __LCMCOMMUNICATOR_H__

#include <lcm/lcm-cpp.hpp>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <functional>
#include <atomic>
#include <vector>
#include <string>
#include <unordered_map>
#include <stdexcept>
#include <chrono>
#include <utility>


//! \brief      全局回调类型：用于解析自定义LCM消息
typedef std::function<void(const std::string& channel, const uint8_t* buf, size_t len)> GlobalLcmCallback;

//! \brief      消息队列项：承载LCM二进制消息
struct LcmMsgItem {
    std::string channel;
    std::vector<uint8_t> data;
    LcmMsgItem(std::string ch, std::vector<uint8_t> d)
        : channel(std::move(ch)), data(std::move(d)) {}
};

namespace PHYBOT_TOOL {

class LcmCommunicator {
public:
    //! \brief      接收消息回调类
    class MessageHandler {
    public:
        LcmCommunicator* self;
        //! \brief      回调函数
        void onMessage(const lcm::ReceiveBuffer* rbuf, const std::string& ch);
    };

public:
    //! \brief      构造函数：初始化LCM+启动线程
    //!                 lcm 底层默认绑定端口7667
    //! \param      mode        -  0x01:网卡名称， 0x02:绑定ip
    LcmCommunicator(int send_port = 7667, char mode = 0x01, std::string net_dev = std::string("ens33"));

    //! \brief      析构函数
    ~LcmCommunicator();


    LcmCommunicator(const LcmCommunicator&) = delete;
    LcmCommunicator& operator=(const LcmCommunicator&) = delete;

public:

     //! \brief      订阅主题，完全匹配LCM官方类成员回调写法
     //! \param      channel 订阅主题名

    void subscribe(const std::string& channel);

    //! \brief      发布接口
    //! \param      channel         - 消息topicId
    //! \param      msg             - 消息
    //! \return     0为成功， -1为失败
    template <typename MsgType>
    bool publish(const std::string& channel, const MsgType& msg)
    {
        if (!_lcm->good())
        {
            return false;
        }
        return _lcm->publish<MsgType>(channel, &msg);
    }

    //! \brief      注册全局消息解析回调
    void set_global_callback(GlobalLcmCallback cb);

    //! \brief      取消订阅
    void unsubscribe(const std::string& channel);

    //! \brief      检查LCM状态
    bool is_good() const;

private:
    //! \brief      实际的消息处理逻辑，从Handler中转发过来
    void onMessageImpl(const lcm::ReceiveBuffer* rbuf, const std::string& ch);

    //! \brief      接收线程：处理LCM消息循环
    void recv_loop();

    //! \brief      分发线程：触发全局回调解析消息
    void dispatch_loop();


    //! \brief      发送端口检查是否占用
    bool isPortUsed(int port);

private:

    lcm::LCM * _lcm = nullptr;
    std::thread _recv_thread;
    std::thread _dispatch_thread;
    std::string m_url;

    std::atomic<bool> _run{true};

    //! \brief      所有数据接收公用一个接收类
    MessageHandler _handler;

    mutable std::mutex _mtx;
    mutable std::mutex _q_mtx;
    mutable std::mutex _cb_mtx;

    std::queue<LcmMsgItem> _queue;
    std::condition_variable _cv;
    GlobalLcmCallback _global_cb;

    //! \brief      订阅管理map，只存订阅句柄
    std::unordered_map<std::string, lcm::Subscription*> _subscriptions;
};

} // namespace PHYBOT_TOOL


#endif // __LCMCOMMUNICATOR_H__

