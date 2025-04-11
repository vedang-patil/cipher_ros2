#include <string>
#include "rclcpp/rclcpp.hpp"
#include "cipher_interfaces/msg/cipher_message.hpp"
#include "cipher_interfaces/srv/cipher_answer.hpp"


std::string encode(const std::string& input, int8_t key)
{
    std::string result = input;
    key %= 26;

    for (char &c: result) {
        if (isalpha(c)) {
            char offset = islower(c) ? 'a' : 'A';
            c = (c - offset + key + 26) % 26 + offset;
        } else {
            result += c;
        }
    }

    return result;
}

class GameSetter : public rclcpp::Node
{
private:
    rclcpp::Publisher<cipher_interfaces::msg::CipherMessage>::SharedPtr publisher_;
public:
    GameSetter()
        :Node("game_setter")
    {
        publisher_ = this->create_publisher<cipher_interfaces::msg::CipherMessage>("message_to_encode", 10);

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
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GameSetter>());
    rclcpp::shutdown();
    return 0;
}