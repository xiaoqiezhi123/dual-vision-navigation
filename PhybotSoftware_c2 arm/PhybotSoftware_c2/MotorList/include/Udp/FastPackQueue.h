#ifndef FASTPACKQUEUE_H
#define FASTPACKQUEUE_H

#include <string>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>

using namespace std;


class FastPackQueue {

public:
    FastPackQueue();
    ~FastPackQueue();

    /**
     * @brief 将数据放入接收队列
     *
     * @param data 待入队的数据包
     * @note 线程安全，使用互斥锁保护队列操作
     * @attention 队列长度参数capacity_，满时阻塞
     */

    void PutPack(const std::string& data);

    /**
     * @brief 从队列取出数据
     *
     * @return string 出队的数据包
     * @note 线程安全，队列为空时阻塞
     */

    string GetPack();

private:
    mutable std::mutex mutex_;
    std::condition_variable cvNotEmpty_;
    std::condition_variable cvNotFull_;
    size_t capacity_ = 90;
    std::queue<string> Que_Buffer;

};



#endif // PACKQUEUE_H    