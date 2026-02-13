#include "rclcpp/rclcpp.hpp"
#include "rm_server/srv/el_gamal_encrypt.hpp"

class TestClient : public rclcpp::Node
{
public:
    TestClient() : Node("test_client")
    {
        client_ = this->create_client<rm_server::srv::ElGamalEncrypt>("elgamal_service");
        RCLCPP_INFO(this->get_logger(), "Client created, waiting for service...");
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Service not available");
            rclcpp::shutdown();
            return;
        }
        auto request = std::make_shared<rm_server::srv::ElGamalEncrypt::Request>();
        request->public_key = 12345;
        RCLCPP_INFO(this->get_logger(), "Sending request...");
        auto callback = [this](rclcpp::Client<rm_server::srv::ElGamalEncrypt>::SharedFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Received response: y1=%ld, y2=%ld", response->y1, response->y2);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
            }
            rclcpp::shutdown();
        };
        client_->async_send_request(request, callback);
    }

private:
    rclcpp::Client<rm_server::srv::ElGamalEncrypt>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestClient>());
    rclcpp::shutdown();
    return 0;
}