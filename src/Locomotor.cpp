#include <iostream>
#include <string>
#include <map>
#include <memory>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "./Executor_3.hpp"

using GlobalPlanCallback = std::function<void ()>;
using GlobalPlanExceptionCallback = std::function<void ()>;
using LocalPlanCallback = std::function<void ()>;
using LocalPlanExceptionCallback = std::function<void ()>;
using PlannerExceptionCallback = std::function<void ()>;


class Locomotor : public rclcpp::Node
{
public:
  Locomotor()
  : Node("locomotor"), g_ex("global"), l_ex("local")
  {
    g_cnt = 0;
    l_cnt = 0;

    goal_sub =
      create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&Locomotor::topic_callback, this, std::placeholders::_1));

    g_timer =
      create_wall_timer(
      std::chrono::milliseconds(1000), std::bind(
        &Locomotor::planLoopCallback,
        this));
    g_timer->cancel();

    l_timer = create_wall_timer(
      std::chrono::milliseconds(250), std::bind(
        &Locomotor::controlLoopCallback,
        this));
    l_timer->cancel();
  }

  ~Locomotor()
  {
    if (g_timer) {
      g_timer->cancel();
      g_timer.reset();
    }

    if (l_timer) {
      l_timer->cancel();
      l_timer.reset();
    }
  }

  void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr p)
  {
    RCLCPP_INFO(get_logger(), "goal receive! start");
    g_cnt = 0;
    l_cnt = 0;
    g_timer->reset();
  }

  void planLoopCallback()
  {
    requestGlobalPlan(
      g_ex, l_ex, std::bind(&Locomotor::onNewGlobalPlan, this),
      std::bind(&Locomotor::onGlobalPlanningException, this));
  }

  void controlLoopCallback()
  {
    requestLocalPlan(
      g_ex, l_ex, std::bind(&Locomotor::onNewLocalPlan, this),
      std::bind(&Locomotor::onLocalPlanningException, this));
  }

  void requestGlobalPlan(
    Executor & work_ex, Executor & result_ex, GlobalPlanCallback cb = nullptr,
    GlobalPlanExceptionCallback fail_cb = nullptr)
  {
    work_ex.addCallback(
      std::bind(
        &Locomotor::makeGlobalPlan, this, std::ref(
          result_ex), cb, fail_cb));
  }

  void makeGlobalPlan(Executor & result_ex, GlobalPlanCallback cb, PlannerExceptionCallback fail_cb)
  {
    std::cout << "makeGlobalPlan =>>" << std::endl;
    if (cb) {
      result_ex.addCallback(cb);
    }
  }


  void onNewGlobalPlan()
  {
    std::cout << "GP YES " << g_cnt++ << std::endl;
    if (g_cnt > 10) {
      std::cout << "GP FINISH " << g_cnt << std::endl;
      g_timer->cancel();
      // g_timer->reset();
      // g_timer.reset();  // exit when reset ??
      if (!g_timer->is_canceled()) {
        std::cout << "g_timer canceled, but ??" << std::endl;
      }
    }

    if (l_timer->is_canceled()) {
      std::cout << "local timer start!" << std::endl;
      l_timer->reset();
    }
  }

  void onGlobalPlanningException()
  {
    std::cerr << "GP FAILED xx" << std::endl;
  }

  void requestLocalPlan(
    Executor & work_ex, Executor & result_ex, LocalPlanCallback cb = nullptr,
    LocalPlanExceptionCallback fail_cb = nullptr)
  {
    work_ex.addCallback(
      std::bind(
        &Locomotor::makeLocalPlan, this, std::ref(result_ex), cb, fail_cb));
  }

  void makeLocalPlan(Executor & result_ex, LocalPlanCallback cb, PlannerExceptionCallback fail_cb)
  {
    std::cout << "makeLocalPlan =>>" << std::endl;
    if (cb) {
      result_ex.addCallback(cb);
    }
  }

  void onNewLocalPlan()
  {
    std::cout << "LP YES " << l_cnt++ << std::endl;
    if (l_cnt > 50) {
      std::cout << "LP FINISH " << l_cnt << std::endl;
      l_timer->cancel();
      // l_timer.reset();   // whey EXIT ??
    }
  }

  void onLocalPlanningException()
  {
    std::cerr << "LP FAILED xx" << std::endl;
  }

private:
  rclcpp::TimerBase::SharedPtr g_timer, l_timer;
  rclcpp::SubscriptionBase::SharedPtr goal_sub;

  int g_cnt;
  int l_cnt;

  Executor g_ex;
  Executor l_ex;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Locomotor>());
  rclcpp::shutdown();
}
