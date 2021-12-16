#include <iostream>
#include <string>
#include <deque>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <functional>
#include <memory>
#include <chrono>

using Function = std::function<void ()>;

// 回调队列，保存进来的数据
// 背后的线程，不断对队列数据进行处理, 和生产者消费者类似
// 使用std::bind 构造无参可调用对象，构造回调队列, 主要是为了保存通用的可调用对象，并用容器存储
// bind 用于生成一个新的可调用对象，比如把参数和函数绑定（保存）在一起，改变参数的个数等

class Executor
{
public:
  Executor()
  : terminated_(false)
  {
    thread_ = std::make_shared<std::thread>(&Executor::spin, this);
  }

  ~Executor()
  {
    std::cout << "~Executor" << std::endl;
    terminated_ = true;
    cv_.notify_one();
    if (thread_) {
      thread_->join();
    }
  }

  void addCallback(Function f)
  {
    mutex_.lock();
    queue_.push_back(f);
    mutex_.unlock();

    cv_.notify_one();
  }

  void spin()
  {
    while (!terminated_) {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(
        lock, [this]() {
          if (terminated_) {
            return true;  // used for ctrl-c exit
          } else if (queue_.empty()) {
            return false;
          } else {
            return true;
          }
        });

      if(terminated_) continue;

      auto & t = queue_.front();
      t();
      queue_.pop_front();
      lock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "SPINTHREADEXIT" << std::endl;
  }
  void kill() {terminated_ = true;}

private:
  bool terminated_;
  std::deque<Function> queue_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::shared_ptr<std::thread> thread_;
};

void one(int i)
{
  std::cout << "one " << i << std::endl;
}

void two(int i, int j)
{
  std::cout << "two " << i << ", " << j << std::endl;
}

// int main() {
//     Executor e;

//     for(int i = 0; i < 10; ++i) {
//       std::cout << "add " << i << std::endl;
//       e.addCallback(std::bind(one, i));
//     }

//     // std::this_thread::sleep_for(std::chrono::seconds(22));

//     for(int i = 5; i < 10; ++i) {
//       std::cout << "add " << i << std::endl;
//       e.addCallback(std::bind(two, i, i * 10));
//     }
//     std::this_thread::sleep_for(std::chrono::seconds(18));

//     // e.kill();

//     for(int i = 100; i < 105; ++i) {
//       std::cout << "add " << i << std::endl;
//       e.addCallback(std::bind(one, i));
//     }

//     std::this_thread::sleep_for(std::chrono::seconds(6));

//     std::cout << "finish" << std::endl;


// }
