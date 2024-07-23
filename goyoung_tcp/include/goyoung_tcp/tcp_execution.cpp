#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <unistd.h>
#include <arpa/inet.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "goyoung_msgs/msg/mode.hpp"
#include "std_msgs/msg/int8.hpp"
using namespace std::chrono_literals;
using namespace std::placeholders;

class TcpNode : public rclcpp::Node
{
public:
    TcpNode() : Node("tcp_node")
    {
        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
        {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
        {
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }

        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(PORT);

        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
        {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }
        if (listen(server_fd, 5) < 0)
        {
            perror("listen");
            exit(EXIT_FAILURE);
        }

        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        publisher = this->create_publisher<goyoung_msgs::msg::Mode>("robot_motion_mode", qos_profile);
        origin_publisher = this->create_publisher<goyoung_msgs::msg::Mode>("back_origin", qos_profile);
        video_publisher = this->create_publisher<std_msgs::msg::Int8>("change_video", qos_profile);
        tcp_execution();
    }
    ~TcpNode()
    {
        close(new_socket);
        close(server_fd);
    }

private:
    uint16_t PORT = 12345;
    int server_fd;
    int new_socket;

    struct sockaddr_in address;
    
    int opt = 1;
    int addrlen = sizeof(address);

    char buffer[1024] = {0};

    rclcpp::Publisher<goyoung_msgs::msg::Mode>::SharedPtr publisher;
    rclcpp::Publisher<goyoung_msgs::msg::Mode>::SharedPtr origin_publisher;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr video_publisher;
    void tcp_execution()
    {
        while (rclcpp::ok())
        {
            if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
            {
                perror("accept");
                exit(EXIT_FAILURE);
            }
            bool tcp_activate = true;
            while (rclcpp::ok() && tcp_activate)
            {
                int valread = read(new_socket, buffer, 1024);
                if (valread == 0)
                {
                    close(new_socket);
                    break;
                }
                else if (valread < 0)
                {
                    perror("read");
                    close(new_socket);
                    break;
                }
                else
                    tcp_activate = false;

                std::string data(buffer, strnlen(buffer, 1024));
                RCLCPP_INFO(this->get_logger(), "Received: %s", buffer);
                data = buffer;
                data = data.substr(0, data.find("\n"));
                
                int8_t mode = 0;
                if (data == "IN")
                {
                    mode = 0;
                }
                else if (data == "A1")
                {
                    mode = 1;
                }
                else if (data == "A2")
                {
                    mode = 2;
                }
                else if (data == "B1")
                {
                    mode = 3;
                }
                else if (data == "B2")
                {
                    mode = 4;
                }
                else if (data == "B3")
                {
                    mode = 5;
                }
                else if (data == "B4")
                {
                    mode = 6;
                }
                else if (data == "B5")
                {
                    mode = 7;
                }
                else if (data == "B6")
                {
                    mode = 8;
                }
                auto robot_mode = goyoung_msgs::msg::Mode();
                robot_mode.mode = mode;
                auto video_mode = std_msgs::msg::Int8();

                if (mode == 0)
                {
                    video_mode.data = 0;
                }
                else if (mode == 9)
                {
                    video_mode.data = 1;
                }
                else if (mode == 10)
                {
                    video_mode.data = 3;
                }
                else
                {
                    video_mode.data = 2;
                }

                if (mode == 0 || 8)
                {
                    origin_publisher->publish(robot_mode);
                }

                publisher->publish(robot_mode);
                video_publisher->publish(video_mode);

                
                send(new_socket, buffer, strlen(buffer), 0);

            }
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }
};
