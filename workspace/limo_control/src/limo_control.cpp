#include "limo_control.hpp"

limoController::limoController() : Node("limo_controller"){
    // intialize subscriber node
    subscriber_ =  this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", SUBSCRIBER_BUFFER_SIZE,
        std::bind(&limoController::odometryCallback, this, std::placeholders::_1)
    );
    // initialize publisher node
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", PUBLISHER_BUFFER_SIZE
    );
    // intialize timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&limoController::publisherCallback, this)
    );

    // set up goal point 
    goalPoint.x = 4;
    goalPoint.y = 5;
    goalPoint.z = 0;
    // set up goal pose
    goalPose = 0.1; 
}


void limoController::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg){

    // extract current coordinates and pose    
    geometry_msgs::msg::Point currentPoint = msg->pose.pose.position;   
    geometry_msgs::msg::Pose  currentPose = msg->pose.pose;

    // if we aren't at (x,y) continue moving towards the point (update required heading)
    if (!arrived){
        linearController_.computeLinearOutputSignal(
            std::make_shared<geometry_msgs::msg::Point>(currentPoint), 
            std::make_shared<geometry_msgs::msg::Point>(goalPoint), 
            linearOutput, desiredHeading, arrived
        );  

        linearController_.computeAngularOutputSignal(
            std::make_shared<geometry_msgs::msg::Pose>(currentPose), 
            desiredHeading, angularOutput
        );
        return;
    }

    // otherwise we've arrived and can focus on hitting the theta_g goal
    linearController_.computeAngularOutputSignal(
        std::make_shared<geometry_msgs::msg::Pose>(currentPose), 
        goalPose,angularOutput
    );
}

void limoController::publisherCallback(){
    geometry_msgs::msg::Twist rotationMessage;
    // populate twist message with PI calculated outputs
    rotationMessage.linear.x = linearOutput;
    rotationMessage.angular.z = angularOutput;
    // publish the message
    publisher_->publish(rotationMessage);
}

int main(int argc, char *argv[]) {
    // initialize and advertise service
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<limoController>());
    rclcpp::shutdown();
    
    return 0;
}