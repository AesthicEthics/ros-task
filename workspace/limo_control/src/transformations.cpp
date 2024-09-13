#include "transformations.hpp"

PIDController::PIDController(const float& inputkP_LINEAR, const float& inputkP_ANGULAR, 
                             const float& inputkI_LINEAR, const float& inputkI_ANGULAR
) {
    // create CSV files
    csv_file_.open("robot_data.csv", std::ios::out);
    if (csv_file_.is_open()) {
        csv_file_ << "Time,x,y,DistanceError,OrientationError,LinearVelocity,AngularVelocity\n"; 
    }

    KP_LINEAR = inputkP_LINEAR;
    KI_LINEAR = inputkI_LINEAR;

    KP_ANGULAR = inputkP_ANGULAR;
    KI_ANGULAR = inputkI_ANGULAR;
}

void PIDController::computeLinearOutputSignal(
    const geometry_msgs::msg::Point::SharedPtr currentPoint,
    const geometry_msgs::msg::Point::SharedPtr goalPoint,
    float &outputSignal, float &desiredHeading, bool &arrived
){
    float error;
    modifiedEuclidianDistance(currentPoint, goalPoint, error, desiredHeading);
    prevErrorLinear += error;
    
    // PI Computation based on angle drift and euclidian error threshold
    if (std::abs(prevErrorAngular) < 0.03 && std::abs(error) > 0.03){
        float integral = prevErrorLinear * 0.01;
        float porportionalOutput = KP_LINEAR * error + KI_LINEAR * integral;    
        outputSignal = porportionalOutput;
    }
    else{
        outputSignal = 0;
    }
    // set arrived flag
    arrived = std::abs(error) <= 0.03 ? true : false;

    if (csv_file_.is_open()) {
        csv_file_ << rclcpp::Clock().now().nanoseconds()/1e6 << std::fixed 
                  <<"," << currentPoint->x << "," << currentPoint->y << ","
                  << error << ",,"  // Log distance error
                  << outputSignal << "," << " \n";  // Log linear output
    }
};

void PIDController::computeAngularOutputSignal(
    const geometry_msgs::msg::Pose::SharedPtr currentPose,
    float &goalPose,float &outputSignal
){
    // convert quaternion angles to Euler angles
    double yaw, pitch, roll;
    quaternionToeuler(currentPose, yaw, roll, pitch);

    // caclulate error terms, update for record + integral
    double error = goalPose - yaw;
    angularErrorSum += error;

    if(firstRun){
        yaw += 0.30;
        firstRun = false;
    }


    // // Normalize the error to be within [-pi, pi]
    // if (error > M_PI) {
    //     error -= 2 * M_PI;
    // } else if (error < -M_PI) {
    //     error += 2 * M_PI;
    // }

    prevErrorAngular = error;

    // if we're within 0.01 radians, we can stop
    if (std::abs(error) > 0.001){
        float integral = angularErrorSum * 0.01;
        float porportionalOutput = KP_ANGULAR * error + KI_ANGULAR * integral;  
        outputSignal = porportionalOutput;
    }
    else{
        outputSignal = 0;
    }

    if (csv_file_.is_open()) {
        csv_file_ << rclcpp::Clock().now().nanoseconds()/1e6 << std::fixed
                  << ",,,,"
                  << error << ",,"   // Log orientation error
                  << outputSignal << "\n";  // Log angular output
    }

};

void PIDController::modifiedEuclidianDistance(
    const geometry_msgs::msg::Point::SharedPtr currentPoint, 
    const geometry_msgs::msg::Point::SharedPtr goalPoint,
    float &distance, float &heading
) {
    float delta_x_squared = std::pow(goalPoint->x - currentPoint->x,2);
    float delta_y_squared = std::pow(goalPoint->y - currentPoint->y,2);
    float delta_z_squared = std::pow(goalPoint->z - currentPoint->z,2);

    float summation = delta_x_squared + delta_y_squared + delta_z_squared;
    distance = std::sqrt(summation);

    // continously update heading to (x_g, y_g) from current point
    heading = std::atan2((goalPoint->y - currentPoint ->y), (goalPoint -> x - currentPoint -> x));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Heading to goal: %f radians", heading);

    if (csv_file_.is_open()){
    csv_file_ << rclcpp::Clock().now().nanoseconds()/1e6 << std::fixed
             << ",,,"
              << distance << "," << heading << ",,\n";
    }
}


void quaternionToeuler(
    const geometry_msgs::msg::Pose::SharedPtr quatPose,
    double &yaw,double &roll, double &pitch
){
    tf2::Quaternion q(
        quatPose->orientation.x,
        quatPose->orientation.y,
        quatPose->orientation.z,
        quatPose->orientation.w
    );

    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current heading (yaw): %f radians", yaw);
};

PIDController::~PIDController(){
    if (csv_file_.is_open()) {
        csv_file_.close();
    }
}