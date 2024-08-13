#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "my_pkg/msg/address_book.hpp"

using namespace std::chrono_literals;

class AddressBookPublisher : public rclcpp::Node
{
public:
  AddressBookPublisher(std::string k_v, int num)
  : Node("address_book_publisher")
  {
    std::cout << k_v << num << std::endl;

    address_book_publisher_ =
      this->create_publisher<my_pkg::msg::AddressBook>("address_book", 10);

    auto publish_msg = [this]() -> void {
        auto message = my_pkg::msg::AddressBook();

        message.first_name = "John";
        message.last_name = "Doe";
        message.age = 30;
        message.gender = message.MALE;
        message.address = "unknown";

        std::cout << "Publishing Contact\nFirst:" << message.first_name <<
          "  Last:" << message.last_name << std::endl;

        this->address_book_publisher_->publish(message);
      };
    //timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<my_pkg::msg::AddressBook>::SharedPtr address_book_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::string str1 = argv[1];
  rclcpp::spin(std::make_shared<AddressBookPublisher>(str1, 12));
  rclcpp::shutdown();

  return 0;
}