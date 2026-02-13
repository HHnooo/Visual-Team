#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"      
#include "std_msgs/msg/int64.hpp"       
#include "rm_server/srv/el_gamal_encrypt.hpp"
#include "rm_server/msg/get_el_gamal_params.hpp"
#include <random>
#include <string>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<rm_server::msg::GetElGamalParams>(
            "/elgamal_params", 10,
            std::bind(&MinimalSubscriber::topic_callback, this, _1));
        result_publisher_ = this->create_publisher<std_msgs::msg::Int64>("/elgamal_result", 10);
        debug_publisher_ = this->create_publisher<std_msgs::msg::String>("answer", 10);
        rng_ = std::mt19937(std::random_device{}());
        client_ = this->create_client<rm_server::srv::ElGamalEncrypt>("elgamal_service");

        RCLCPP_INFO(this->get_logger(), "Node started, waiting for parameters...");
    }

private:
    void topic_callback(const rm_server::msg::GetElGamalParams::SharedPtr msg)
    {
        int64_t p = msg->p;
        int64_t a = msg->a;
        RCLCPP_INFO(this->get_logger(), "Received p=%ld, a=%ld", p, a);
        std::uniform_int_distribution<int64_t> dist(1, p - 2);
        int64_t n = dist(rng_);
        RCLCPP_INFO(this->get_logger(), "Generated private key n=%ld", n);
        int64_t b = mod_pow(a, n, p);
        RCLCPP_INFO(this->get_logger(), "Computed public key b=%ld", b);
        RCLCPP_INFO(this->get_logger(), "Waiting for service...");
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Service 'elgamal_service' not available after 1 second");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service is available");
        auto request = std::make_shared<rm_server::srv::ElGamalEncrypt::Request>();
        request->public_key = b;
        RCLCPP_INFO(this->get_logger(), "Sending request with public_key=%ld", b);
        auto callback = [this, p, n](rclcpp::Client<rm_server::srv::ElGamalEncrypt>::SharedFuture future) {
            RCLCPP_INFO(this->get_logger(), "!!! CALLBACK ENTERED !!!");
            try {
                auto response = future.get();
                int64_t y1 = response->y1;
                int64_t y2 = response->y2;
                RCLCPP_INFO(this->get_logger(), "Received response: y1=%ld, y2=%ld", y1, y2);
                int64_t y1_pow_n = mod_pow(y1, n, p);
                int64_t inv = mod_inverse(y1_pow_n, p);
                int64_t x = (y2 % p) * (inv % p) % p;
                RCLCPP_INFO(this->get_logger(), "Decrypted x=%ld", x);
                auto result_msg = std_msgs::msg::Int64();
                result_msg.data = x;
                result_publisher_->publish(result_msg);
                RCLCPP_INFO(this->get_logger(), "Result published to /elgamal_result");
                auto debug_msg = std_msgs::msg::String();
                debug_msg.data = std::to_string(x);
                debug_publisher_->publish(debug_msg);
                RCLCPP_INFO(this->get_logger(), "Result also published to /answer for debugging");
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Exception in service callback: %s", e.what());
            }
        };
        client_->async_send_request(request, callback);
        RCLCPP_INFO(this->get_logger(), "Request sent, waiting for callback...");
    }
    int64_t mod_pow(int64_t base, int64_t exp, int64_t mod) const
    {
        int64_t res = 1 % mod;
        base %= mod;
        while (exp > 0) {
            if (exp & 1)
                res = (res * base) % mod;
            base = (base * base) % mod;
            exp >>= 1;
        }
        return res;
    }
    int64_t mod_inverse(int64_t a, int64_t p) const
    {
        return mod_pow(a, p - 2, p);
    }
    rclcpp::Subscription<rm_server::msg::GetElGamalParams>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr result_publisher_;  
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_publisher_;  
    std::mt19937 rng_;
    rclcpp::Client<rm_server::srv::ElGamalEncrypt>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}