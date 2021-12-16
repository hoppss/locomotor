#include <iostream>
#include <string>
#include <map>
#include <memory>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"


class TimerTest : public rclcpp::Node
{
public:
  TimerTest()
  : Node("timer_test")
  {
    cnt = 0;
    g_timer =
      create_wall_timer(
      std::chrono::milliseconds(300), std::bind(
        &TimerTest::LoopCallback,
        this));
  }

  void LoopCallback()
  {
    std::cout << "timer " << ++cnt << std::endl;

    if (cnt == 10) {
      g_timer->cancel();
      std::cout << "cancel & reset " << std::endl;
      g_timer->reset();
    }
  }

private:
  rclcpp::TimerBase::SharedPtr g_timer;
  int cnt = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimerTest>());
  rclcpp::shutdown();
}
