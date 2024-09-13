#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/pose.hpp"
#include <fstream>
#include <iomanip>
#include "math.h"

/// @brief PIDController Class used to manage state across PID Calls + Introduce High Modularity
class PIDController {
    public:
        PIDController(const float& inputkP_LINEAR, const float& inputkP_ANGULAR, 
                      const float &inputkI_LINEAR, const float& inputkI_ANGULAR
        );

        ~PIDController();
        
        //LINEAR GAINS
        float KP_LINEAR;
        float KI_LINEAR;
        
        //ANGULAR GAINS
        float KP_ANGULAR;
        float KI_ANGULAR;

        // ERROR CONSTANTS
        float prevErrorLinear{1000};
        float prevErrorAngular{1000};
        float angularErrorSum{};

        bool firstRun{true};

        /// @brief Compute PI Signal for Linear parameters
        /// @param geometry_msgs::msg::Pose::SharedPtr currentPoint 
        /// @param geometry_msgs::msg::Pose::SharedPtr goalPoint 
        /// @param float &outputSignal 
        /// @param float &desiredHeading 
        /// @param bool &arrived 
        void computeLinearOutputSignal(const geometry_msgs::msg::Point::SharedPtr currentPoint,
                                    const geometry_msgs::msg::Point::SharedPtr goalPoint,
                                    float &outputSignal, float &desiredHeading, bool &arrived
        );

        /// @brief Compute PI Signal for Angular Parameters
        /// @param geometry_msgs::msg::Pose::SharedPtr currentPose 
        /// @param float& goalPose 
        /// @param float& outputSignal 
        /// @param bool& arrived 
        void computeAngularOutputSignal(const geometry_msgs::msg::Pose::SharedPtr currentPose,
                                    float &goalPose,float &outputSignal
        );

        /// @brief Function to calculate Euclidean distance b/w two Point msgs
        /// @param geometry_msgs::msg::Pose::SharedPtr currentPoint  
        /// @param geometry_msgs::msg::Pose::SharedPtr goalPoint
        /// @param float& distance
        void modifiedEuclidianDistance(const geometry_msgs::msg::Point::SharedPtr currentPoint, 
                                const geometry_msgs::msg::Point::SharedPtr goalPoint,
                                float &distance, float &heading
        );
        std::ofstream csv_file_;
};


/// @brief Convert Quaternion Poses To Euler Poses 
/// @param geometry_msgs::msg::Pose::SharedPtr quatPose 
/// @param double& roll 
/// @param double& pitch 
/// @param double& yaw 
void quaternionToeuler(const geometry_msgs::msg::Pose::SharedPtr quatPose,
                       double &roll, double &pitch, double &yaw);