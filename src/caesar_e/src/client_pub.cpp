#include "rclcpp/rclcpp.hpp"
#include "cipher_interfaces/srv/cipher_answer.hpp"
#include "cipher_interfaces/msg/cipher_message.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

using namespace std::chrono_literals;

/*

This is a test client node:
it will encode a message then publish it to cipher_interfaces/msg/CipherMessage

Then it will receive a response calling the service cipher_interfaces/srv/CipherAnswer
Then it will vaildate that the call was correct
*/

std::string caesar_encrypt(std::string plaintext, int shift);

void confirm_solution(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cipher_interfaces::srv::CipherAnswer::Request> request,
    std::shared_ptr<cipher_interfaces::srv::CipherAnswer::Response> response)
    {
    response->result = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
    "Yay full marks! '%s' was right!", request->answer.c_str());
}
    
    

class ClientNode : public rclcpp::Node
{
    public:
        ClientNode(std::string plaintext, int shift)
        : Node("client_node")
        {
            std::shared_ptr<rclcpp::Node> new_node = rclcpp::Node::make_shared("cipher_answer_server");
            rclcpp::Service<cipher_interfaces::srv::CipherAnswer>::SharedPtr service =
            new_node->create_service<cipher_interfaces::srv::CipherAnswer>("cipher_answer", &confirm_solution);

            publisher_ = this->create_publisher<cipher_interfaces::msg::CipherMessage>("cipher_message", 10);
            auto message = cipher_interfaces::msg::CipherMessage();
            message.message = caesar_encrypt(plaintext, shift);
            message.key = (short)(26 - shift);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'\nKey: %d", message.message.c_str(), message.key);
            publisher_->publish(message);
            rclcpp::spin(new_node);
            rclcpp::shutdown();
        }
    private:
        rclcpp::Publisher<cipher_interfaces::msg::CipherMessage>::SharedPtr publisher_;       
};

int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 3) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage client_pub PLAINTEXT SHIFT");
        return 1;
    }
    std::string plaintext = argv[1];
    int shift = std::stoi(argv[2]);
    shift = ((shift % 26) + 26) % 26;

    rclcpp::spin(std::make_shared<ClientNode>(plaintext, shift));
    rclcpp::shutdown();
    return 0;
}

std::string caesar_encrypt(std::string plaintext, int shift) {
    std::string ret_str = "";
    for (int i = 0; i < (int)plaintext.size(); i++) {
        if ('a' <= plaintext[i] && plaintext[i] <= 'z') {
            ret_str += (plaintext[i] - 'a' + shift) % 26 + 'a';
            continue;
        }
        if ('A' <= plaintext[i] && plaintext[i] <= 'Z') {
            ret_str += (plaintext[i] - 'A' + shift) % 26 + 'A';
            continue;
        }
        ret_str += plaintext[i];
    }
    return ret_str;
}

