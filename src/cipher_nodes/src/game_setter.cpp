#include <string>
#include "rclcpp/rclcpp.hpp"
#include "cipher_interfaces/msg/cipher_message.hpp"
#include "cipher_interfaces/srv/cipher_answer.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>


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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher2_;
public:
    GameSetter()
        :Node("game_setter")
    {
        publisher_ = this->create_publisher<cipher_interfaces::msg::CipherMessage>("message_to_encode", 10);
        publisher2_ = this->create_publisher<std_msgs::msg::String>("original_message", 10);

        std::string messageToEncrypt;
        int8_t key;

        std::cout << "Message to Encode: ";
        getline(std::cin, messageToEncrypt);
        std::cout << "Key: ";
        std::cin >> key;

        auto message = cipher_interfaces::msg::CipherMessage();
        message.message = encode(messageToEncrypt, key);
        message.key = key;

        publisher_->publish(message);
        auto original = std_msgs::msg::String();
        original.data = messageToEncrypt;
        publisher2_->publish(original);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GameSetter>());
    rclcpp::shutdown();
    return 0;
}