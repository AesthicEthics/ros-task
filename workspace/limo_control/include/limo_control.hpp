#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "transformations.hpp"

class limoController: public rclcpp::Node {
    public:
        limoController();

        // modifiable buffer sizes
        int SUBSCRIBER_BUFFER_SIZE = 20;
        int PUBLISHER_BUFFER_SIZE = 20;

        // Linear Constants
        const float LINEAR_KP = 0.075;
        const float LINEAR_KI = 0.001;

        // Angular Constants
        const float ANGULAR_KP = 1.1;
        const float ANGULAR_KI = 1.2;

        PIDController linearController_{LINEAR_KP, ANGULAR_KP, LINEAR_KI, ANGULAR_KI};
    private:
        bool arrived{false};

        float linearOutput;
        float angularOutput;
        float desiredHeading;

        // publisher, subscriber, and timer nodes
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;        
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        geometry_msgs::msg::Point goalPoint;
        float goalPose;

        /// @brief Odometry Callback, called back every 10ms
        /// @param msg 
        void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        /// @brief Call back to publish data to twist message
        void publisherCallback();
};