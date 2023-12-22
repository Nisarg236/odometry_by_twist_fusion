#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <math.h>
#include <cmath>

// Kalman Filter to remove noise from the data and smooth the inputs
class KalmanFilter
{
private:
    float r;     // noisy covariance
    float h;     // measurement map scalar
    float q;     // initial estimated covariance
    float p;     // initial error covariance
    float u_hat; // initial estimated state
    float k;     // static kalman gain

public:
    KalmanFilter(float noisyCovariance, float measurementScalar, float initialCovariance, float initialState, float initialErrorCovariance)
        : r(noisyCovariance), h(measurementScalar), q(initialCovariance), p(initialErrorCovariance), u_hat(initialState), k(0) {}
    float filter(float u)
    {
        // Kalman filter algorithm
        k = p * h / (h * p * h + r);         // update kalman gain
        u_hat = u_hat + k * (u - h * u_hat); // update estimated
        p = (1 - k * h) * p + q;             // update error covariance
        // estimate of u_hat
        return u_hat;
    }
    float filter(float u1, float u2)
        {
            // Kalman filter algorithm for fusing two measurements
    
            // Calculate weights based on measurement variances
            float weight1 = 1 / (h * p * h + r);   // weight for u1
            float weight2 = 1 / (h * p * h + r);   // weight for u2
    
            // Combine measurements using weights
            float combinedMeasurement = (weight1 * u1 + weight2 * u2) / (weight1 + weight2);
    
            // Kalman gain for the combined measurement
            float k_combined = p * h / (h * p * h + r);
    
            // Update estimated state and error covariance for the combined measurement
            u_hat = u_hat + k_combined * (combinedMeasurement - h * u_hat);
            p = (1 - k_combined * h) * p + q;
    
            // Return the fused estimate
            return u_hat;
        }
};

class FusionNode : public rclcpp::Node
{
public:
    // arrays that store subscribed data
    float odomLinear[3];   // x,y,x
    float odomAngular[3];  // x,y,z
    float imuLinear[3];    // x,y,z
    float imuAngular[3];   // x,y,z
    float fusedLinear[3];  // x,y,z
    float fusedAngular[3]; // x,y,z
    float odom[3];         // here we will store our calculated odometry x,y,theta
    float old_odom[3] = {0, 0, 0};
    float delta_odom[3];

    KalmanFilter kfDeltaT{100, 1, 10, 0, 0};       // Kalman filter object for errors in measurement of delta_t
    KalmanFilter kfImuAngularZ{300, 1, 10, 0, 0};  // Kalman filter object for errors in measurement of IMU angular Z velocity
    KalmanFilter kfOdomAngularZ{300, 1, 10, 0, 0}; // Kalman filter object for errors in measurement of Odom angular Z velocity
    KalmanFilter kfLinearX{300, 1, 10, 0, 0};      // Kalman filter object for errors in measurement of Linear X velocity from Odom

    // start measuring time for calculation of delta_t
    rclcpp::Time old_time = now();

    FusionNode() : Node("fusion_node")
    {
        // Subscribe to odometry topic and bind the callback function processOdometry
        odometry_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "/odom_velocity", 10, [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
            { processOdometry(msg); });
        // subscribe to imu and bind the callback function processIMU
        imu_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "/imu_velocity", 10, [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
            { processIMU(msg); });
        // publish fused velocities at 70hz
        timer_ = create_wall_timer(std::chrono::milliseconds(1000 / 70),
                                   [this]()
                                   {
                                       fuseAndPublish();
                                   });

        fused_velocity_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("/fused_velocity", 10);
        odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    }

private:
    void processOdometry(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        this->odomLinear[0] = kfLinearX.filter(msg->twist.linear.x); // apply kalman filter
        this->odomLinear[1] = msg->twist.linear.y;
        this->odomLinear[2] = msg->twist.linear.z;
        this->odomAngular[0] = msg->twist.angular.x;
        this->odomAngular[1] = msg->twist.angular.y;
        this->odomAngular[2] = kfOdomAngularZ.filter(msg->twist.angular.z); // apply kalman filter
    }

    void processIMU(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        this->imuLinear[0] = msg->twist.linear.x;
        this->imuLinear[1] = msg->twist.linear.y;
        this->imuLinear[2] = msg->twist.linear.z;
        this->imuAngular[0] = msg->twist.angular.x;
        this->imuAngular[1] = msg->twist.angular.y;
        this->imuAngular[2] = kfImuAngularZ.filter(msg->twist.angular.z); // apply kalman filter
    }

    void fuseAndPublish()
    {
        fusedLinear[0] = odomLinear[0];
        fusedLinear[1] = odomLinear[1];
        fusedLinear[2] = odomLinear[2];
        fusedAngular[0] = imuAngular[0];
        fusedAngular[1] = imuAngular[1];
        fusedAngular[2] = imuAngular[2] * 0.7 + odomAngular[2] * 0.3; // complimentary filter

        // Create a TwistStamped message for the fused velocity
        std::shared_ptr<geometry_msgs::msg::TwistStamped> fused_velocity_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
        // get the current time
        fused_velocity_msg->header.stamp = now();
        fused_velocity_msg->twist.linear.x = fusedLinear[0];
        fused_velocity_msg->twist.linear.y = fusedLinear[1];
        fused_velocity_msg->twist.linear.z = fusedLinear[2];
        fused_velocity_msg->twist.angular.x = fusedAngular[0];
        fused_velocity_msg->twist.angular.y = fusedAngular[1];
        fused_velocity_msg->twist.angular.z = fusedAngular[2];

        // Publish the fused velocity
        fused_velocity_publisher_->publish(fused_velocity_msg);
        auto current_time = now();
        // calculate delta_t
        float delta_t = kfDeltaT.filter((current_time.nanoseconds() - old_time.nanoseconds()) * 0.000000001); // convert nano seconds to seconds

        // make current time as old time for next iteration
        old_time = current_time;

        // calculate pose (integrate velocities wrt time)
        odom[2] += fusedAngular[2] * delta_t;                    // theta
        odom[0] += fusedLinear[0] * delta_t * std::cos(odom[2]); // x
        odom[1] += fusedLinear[0] * delta_t * std::sin(odom[2]); // y
        delta_odom[0] = odom[0] - old_odom[0];
        delta_odom[1] = odom[1] - old_odom[1];
        delta_odom[2] = odom[2] - old_odom[2];
        float x = old_odom[0] + delta_odom[0] * delta_t * std::cos(delta_odom[2]);
        float y = old_odom[1] + delta_odom[1] * delta_t * std::cos(delta_odom[2]);
        old_odom[0] = x;
        old_odom[1] = y;
        old_odom[2] = odom[2];

        // publish state
        std::shared_ptr<nav_msgs::msg::Odometry> odometry_msg = std::make_shared<nav_msgs::msg::Odometry>();
        odometry_msg->header.stamp = now();
        odometry_msg->header.frame_id = "odom";
        odometry_msg->child_frame_id = "base_link";

        odometry_msg->pose.pose.position.x = x;
        odometry_msg->pose.pose.position.y = y;
        odometry_msg->pose.pose.position.z = 0.0;

        // Set orientation
        odometry_msg->pose.pose.orientation.x = 0.0;
        odometry_msg->pose.pose.orientation.y = 0.0;
        odometry_msg->pose.pose.orientation.z = sin(0.5 * odom[2]);
        odometry_msg->pose.pose.orientation.w = cos(0.5 * odom[2]);

        // Set linear and angular velocities
        odometry_msg->twist.twist.linear.x = fusedLinear[0];
        odometry_msg->twist.twist.linear.y = fusedLinear[1];
        odometry_msg->twist.twist.angular.z = fusedAngular[2];

        odometry_publisher_->publish(odometry_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr odometry_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr imu_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr fused_velocity_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FusionNode>());
    rclcpp::shutdown();
    return 0;
}
