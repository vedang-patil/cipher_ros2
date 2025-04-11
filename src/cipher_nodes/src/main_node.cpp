#include <string>
#include "rclcpp/rclcpp.hpp"
#include "cipher_interfaces/msg/cipher_message.hpp"
#include "cipher_interfaces/srv/cipher_answer.hpp"

using std::placeholders::_1;


std::string decode(const std::string& input, int8_t key)
{
    std::string result = input;
    key %= 26;

    for (char &c: result) {
        if (isalpha(c)) {
            char offset = islower(c) ? 'a' : 'A';
            c = (c - offset - key + 26) % 26 + offset;
        } else {
            result += c;
        }
    }

    return result;
}

class MainNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<cipher_interfaces::msg::CipherMessage>::SharedPtr subscription_;
    rclcpp::Client<cipher_interfaces::srv::CipherAnswer>::SharedPtr client_;
public:
    MainNode()
        :Node("main_node")
    {
        subscription_ = this->create_subscription<cipher_interfaces::msg::CipherMessage>("message_to_encode", 10,
            std::bind(&MainNode::callback, this, _1));
    }
private:
    void callback(const cipher_interfaces::msg::CipherMessage::SharedPtr msg) {
        std::string decodedMessage = decode(msg->message, msg->key);
        RCLCPP_INFO(this->get_logger(), "Decoded message: %s", decodedMessage.c_str());

        auto request = std::make_shared<cipher_interfaces::srv::CipherAnswer::Request>();
        request->answer = decodedMessage;

        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto result = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
            if (result.get()->result) {
                RCLCPP_INFO(this->get_logger(), "Decoded message matches original.");
            } else {
                RCLCPP_INFO(this->get_logger(), "Decoded message doesn't match original.");
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainNode>());
    rclcpp::shutdown();
    return 0;
}