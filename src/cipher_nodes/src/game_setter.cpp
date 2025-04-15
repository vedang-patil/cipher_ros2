#include <string>
#include "rclcpp/rclcpp.hpp"
#include "cipher_interfaces/msg/cipher_message.hpp"
#include "cipher_interfaces/srv/cipher_answer.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;

std::string encode(const std::string& input, int8_t key)
{
    std::string result = input;
    key %= 26;

    for (char &c: result) {
        if (isalpha(c)) {
            char offset = islower(c) ? 'a' : 'A';
            c = (c - offset + key) % 26 + offset;
        }
    }

    return result;
}

class GameSetter : public rclcpp::Node
{
private:
    rclcpp::Publisher<cipher_interfaces::msg::CipherMessage>::SharedPtr publisher_;
    rclcpp::Service<cipher_interfaces::srv::CipherAnswer>::SharedPtr service_;

    std::string _original;
    int8_t _key;
public:
    GameSetter()
        :Node("game_setter")
    {
        publisher_ = this->create_publisher<cipher_interfaces::msg::CipherMessage>("message_to_encode", 10);
        service_ = this->create_service<cipher_interfaces::srv::CipherAnswer>("verify_message",
            std::bind(&GameSetter::callbackService, this, _1, _2));

        std::cout << "Message to Encode: ";
        getline(std::cin, _original);
        std::cout << "Key: ";
        std::cin >> _key;

        auto message = cipher_interfaces::msg::CipherMessage();
        message.message = encode(_original, _key);
        message.key = _key;

        publisher_->publish(message);
    }

    void callbackService(const cipher_interfaces::srv::CipherAnswer::Request::SharedPtr request,
        const cipher_interfaces::srv::CipherAnswer::Response::SharedPtr response)
    {
        response->result = (request->answer == _original);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GameSetter>());
    rclcpp::shutdown();
    return 0;
}