//
// Created by xiang on 2022/2/9.
//

#ifndef ASYNC_MESSAGE_PROCESS_H
#define ASYNC_MESSAGE_PROCESS_H

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>

#include "common/log.h"

namespace lightning::sys {
using UL = std::unique_lock<std::mutex>;

/**
 * 异步消息处理类
 * 内部有线程和队列机制，保证回调是串行的
 * @tparam T
 *
 * NOTE skip设为1的时候实际不会跳帧。。设为2的时候实际走一帧跳一帧
 */
template <typename T>
class AsyncMessageProcess {
   public:
    using ProcFunc = std::function<void(const T&)>;  // 消息回调函数
    AsyncMessageProcess() = default;
    AsyncMessageProcess(ProcFunc proc_func, std::string name = "");

    /// 设置处理函数
    void SetProcFunc(ProcFunc proc_func) { custom_func_ = proc_func; }

    /// 设置队列最大长度
    void SetMaxSize(size_t size) { max_size_ = size; }

    /// 开始处理消息
    void Start();

    /// 添加一条消息
    void AddMessage(const T& msg);

    /// 退出
    void Quit();

    /// 清空跳帧计数器，下一个数据会立即执行
    void CleanSkipCnt();

    void SetName(std::string name) { name_ = std::move(name); }
    void SetSkipParam(bool enable_skip, int skip_num) { enable_skip_ = enable_skip, skip_num_ = skip_num; }

    AsyncMessageProcess(const AsyncMessageProcess&) = delete;
    void operator=(const AsyncMessageProcess&) = delete;

   private:
    void ProcLoop();

    std::thread proc_;
    std::mutex mutex_;
    std::condition_variable cv_msg_;
    std::deque<T> msg_buffer_;
    // 两个标志都会被“持锁线程”与“非持锁线程”同时访问（Start/Quit 不持锁），
    // 因此用 atomic；仅在持锁块内做的复合判断仍由 mutex_ 保护。
    std::atomic_bool update_flag_{false};
    std::atomic_bool exit_flag_{false};
    size_t max_size_ = 100;  // 40
    std::string name_;

    /// 跳帧
    bool enable_skip_ = false;
    int skip_num_ = 0;
    int skip_cnt_ = 0;

    ProcFunc custom_func_;
};

template <typename T>
void AsyncMessageProcess<T>::CleanSkipCnt() {
    UL lock(mutex_);
    skip_cnt_ = 0;
}

template <typename T>
AsyncMessageProcess<T>::AsyncMessageProcess(AsyncMessageProcess::ProcFunc proc_func, std::string name) {
    custom_func_ = std::move(proc_func);
    name_ = name;
}

template <typename T>
void AsyncMessageProcess<T>::Start() {
    // 防止重复 Start：若旧线程还在运行，直接赋值新的 std::thread 会在旧线程对象析构时
    // 因未 join 而触发 std::terminate。
    if (proc_.joinable()) {
        LLOG_WARN(logging::kUtil, "{}: Start() ignored, thread already running", name_);
        return;
    }

    {
        UL lock(mutex_);
        msg_buffer_.clear();  // 丢掉上一次生命周期残留的消息
        update_flag_ = false;
    }
    exit_flag_ = false;
    proc_ = std::thread([this]() { ProcLoop(); });
}

template <typename T>
void AsyncMessageProcess<T>::ProcLoop() {
    // 注意：谓词里必须显式 .load()。若直接 `return update_flag_;`，
    // lambda 的 auto 返回类型推导会尝试按值拷贝 std::atomic_bool（拷贝构造已删除）而编译失败。
    while (!exit_flag_.load()) {
        UL lock(mutex_);
        cv_msg_.wait(lock, [this]() { return update_flag_.load(); });

        // take the message and process it
        auto buffer = msg_buffer_;
        msg_buffer_.clear();
        update_flag_ = false;
        lock.unlock();

        // Quit() 可能刚把 exit_flag_ 置位（它不持锁，无法阻止本轮进入循环体）。
        // 此时上游可能已经在做 Finish()，绝不能再调 custom_func_，否则会对已经
        // 结束（甚至已析构）的 lidar_loc_ / map_ 再操作一次。
        if (exit_flag_.load()) {
            break;
        }

        // 处理之
        for (const auto& msg : buffer) {
            custom_func_(msg);
        }
    }
}

template <typename T>
void AsyncMessageProcess<T>::AddMessage(const T& msg) {
    UL lock(mutex_);
    if (enable_skip_) {
        if (skip_cnt_ != 0) {
            skip_cnt_++;
            skip_cnt_ = skip_cnt_ % skip_num_;
            return;
        }

        skip_cnt_++;
        skip_cnt_ = skip_cnt_ % skip_num_;
    }

    msg_buffer_.push_back(msg);
    while (msg_buffer_.size() > max_size_) {
        LLOG_WARN_THROTTLE(logging::kUtil, 1000, "{} exceeds largest size: {}", name_, max_size_);
        msg_buffer_.pop_front();
    }

    update_flag_ = true;
    cv_msg_.notify_one();
}

template <typename T>
void AsyncMessageProcess<T>::Quit() {
    {
        UL lock(mutex_);
        // 先清空残留队列：否则 worker 被唤醒后会把旧消息再跑一轮，
        // 而那正是 Finish() 之后、custom_func_ 已失效的时间窗口。
        msg_buffer_.clear();
        update_flag_ = true;
    }
    exit_flag_ = true;
    cv_msg_.notify_one();

    if (proc_.joinable()) {
        proc_.join();
    }
}

}  // namespace lightning::sys

#endif
