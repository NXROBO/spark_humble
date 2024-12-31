
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include "swiftpro/msg/status.hpp"
#include "swiftpro/msg/position.hpp"

class MotionTest : public rclcpp::Node
{
private:
  geometry_msgs::msg::Twist cmdvel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<swiftpro::msg::Status>::SharedPtr pump_pub;
  rclcpp::Publisher<swiftpro::msg::Position>::SharedPtr pos_pub;
  boost::thread t;
  swiftpro::msg::Status onoff;
  swiftpro::msg::Position pos;
public:
  MotionTest() : Node("spark_test_node")
  {
    pump_pub = this->create_publisher<swiftpro::msg::Status>("pump_topic", 5);
    pos_pub = this->create_publisher<swiftpro::msg::Position>("position_write_topic", 5);
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    t = boost::thread(boost::bind(&MotionTest::testMotion, this));
  }

  ~MotionTest()
  {
    onoff.status = 0;
    pump_pub->publish(onoff);
    t.join();
  }

  bool testMotion()
  {
    rclcpp::Time current_time = this->now();
    double current_time_sec = current_time.seconds();
    double prev_sec = current_time_sec;
    int sec = 10;
    rclcpp::Rate loop_rate(0.2);
    onoff.status = 1;
    pump_pub->publish(onoff);
    pos.x = 120;
    pos.y = 0;
    pos.z = 35;
    sleep(5);
    pos_pub->publish(pos);
    while (1)
    {
      if (this->now().seconds() - prev_sec < sec)
      {
        testTurnBody(0, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        onoff.status = 1;
        pump_pub->publish(onoff);
      }
      else if (this->now().seconds() - prev_sec > sec && this->now().seconds() - prev_sec < 20)
      {
        testTurnBody(0, -2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        pos.x = 120;
        pos.y = 0;
        pos.z = 35;
        pos_pub->publish(pos);        
      }
      else
      {
        prev_sec = this->now().seconds();
      }
    }
  }

  void testTurnBody(float linearx, float angularz)
  {
    geometry_msgs::msg::Twist cmdvel_;
    cmdvel_.linear.x = linearx;
    cmdvel_.angular.z = angularz;
    pub_->publish(cmdvel_);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto test_node = std::make_shared<MotionTest>();
  rclcpp::spin(test_node);
  rclcpp::shutdown();
  return 0;
}
