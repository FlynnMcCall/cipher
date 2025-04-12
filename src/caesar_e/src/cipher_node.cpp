/*
This thing listens to the topic cipher_interfaces/msg/CipherMessage

When it hears activity, it decodes the message and calls the service 
cipher_interfaces/srv/CipherAnswer 

when it recieves a correct response, it self-terminates
*/

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "cipher_interfaces/srv/cipher_answer.hpp"
#include "cipher_interfaces/msg/cipher_message.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

std::string caesar_encrypt(std::string plaintext, int shift);

class CipherNode : public rclcpp::Node
{
    public:
        CipherNode()
        : Node("cipher_node")
        {
            subscription_ = this->create_subscription<cipher_interfaces::msg::CipherMessage>(
                "cipher_message", 10, std::bind(&CipherNode::topic_callback, this, _1)
            );
            RCLCPP_INFO(this->get_logger(), "CipherNode initialised");
        }
    private:
        void topic_callback(const cipher_interfaces::msg::CipherMessage & msg)
        {
            std::string plaintext = caesar_encrypt(msg.message.c_str(), msg.key);
            RCLCPP_INFO(this->get_logger(), "Received key %d, text '%s'", msg.key, msg.message.c_str());
            RCLCPP_INFO(this->get_logger(), "Decrypted to '%s'", plaintext.c_str());

            rclcpp::Client<cipher_interfaces::srv::CipherAnswer>::SharedPtr client = 
            this->create_client<cipher_interfaces::srv::CipherAnswer>("cipher_answer");

            auto request = std::make_shared<cipher_interfaces::srv::CipherAnswer::Request>();
            request->answer = plaintext.c_str();

            while (!client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                  return;
                } 
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }
            auto result = client->async_send_request(request);

            if (result.get()->result){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "That is the correct answer!");
            } else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Something is amiss.");
            }

            rclcpp::shutdown();

        }
        rclcpp::Subscription<cipher_interfaces::msg::CipherMessage>::SharedPtr subscription_;
};

int main (int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CipherNode>());
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