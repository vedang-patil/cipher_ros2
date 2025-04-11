#include <string>
#include "rclcpp/rclcpp.hpp"
#include "cipher_interfaces/msg/cipher_message.hpp"
#include "cipher_interfaces/srv/cipher_answer.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class GameVerifier: public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Service<cipher_interfaces::srv::CipherAnswer>::SharedPtr service_;

    std::string original;
public:
    GameVerifier()
        :Node("game_verifier")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>("original_message", 10,
            std::bind(&GameVerifier::callbackSub, this, _1));
        service_ = this->create_service<cipher_interfaces::srv::CipherAnswer>("verify_message",
            std::bind(&GameVerifier::callbackSer, this, _1, _2));
    }
private:
    void callbackSub(const std_msgs::msg::String::SharedPtr msg)
    {
        original = msg->data;
    }

    void callbackSer(const cipher_interfaces::srv::CipherAnswer::Request::SharedPtr request,
        const cipher_interfaces::srv::CipherAnswer::Response::SharedPtr response)
    {
        response->result = (request->answer == original);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GameVerifier>());
    rclcpp::shutdown();
    return 0;
}