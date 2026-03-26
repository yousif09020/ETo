#include <algorithm>
#include <string>
#include <memory>
#include <mutex>
#include <vector>

#include "alglib/ap.h"
#include "alglib/interpolation.h"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_feedback_linearization_controller/feedback_linearization_controller.hpp"
using nav2_util::declare_parameter_if_not_declared;


namespace feedback_linearization_controller
{
void FeedbackLinearizationController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) return;

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  clock_ = node->get_clock();
  declare_parameter_if_not_declared(node, plugin_name_ + ".Test_feedback_linearization", 
                                  rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, plugin_name_ + ".vpx_test", 
                                  rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".vpy_test", 
                                  rclcpp::ParameterValue(0.0));                                                                
  declare_parameter_if_not_declared(node, plugin_name_ + ".linearization_frequency", 
                                  rclcpp::ParameterValue(5.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".epsilon_", 
                                  rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_max_", 
                                  rclcpp::ParameterValue(2.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".wheel_radius_", 
                                  rclcpp::ParameterValue(0.1));                                
  declare_parameter_if_not_declared(node, plugin_name_ + ".acc_lin_max_", 
                                  rclcpp::ParameterValue(1.0)); 
  declare_parameter_if_not_declared(node, plugin_name_ + ".pid_Kp_", 
                                  rclcpp::ParameterValue(1.0));                                                               

  node->get_parameter(plugin_name_ + ".Test_feedback_linearization", Test_feedback_linearization);
  node->get_parameter(plugin_name_ + ".vpx_test", vpx_test);
  node->get_parameter(plugin_name_ + ".vpy_test", vpy_test);
  node->get_parameter(plugin_name_ + ".linearization_frequency", linearization_frequency);
  node->get_parameter(plugin_name_ + ".epsilon_", epsilon_);
  node->get_parameter(plugin_name_ + ".w_max_", w_max_);
  node->get_parameter(plugin_name_ + ".wheel_radius_", wheel_radius_);
  node->get_parameter(plugin_name_ + ".acc_lin_max_", acc_lin_max_);
  node->get_parameter(plugin_name_ + ".pid_Kp_", Kp_);
  
  RCLCPP_INFO(logger_, "Controller %s configured", plugin_name_.c_str());
} 

void FeedbackLinearizationController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up controller: %s", plugin_name_.c_str());
}

void FeedbackLinearizationController::activate()
{
  RCLCPP_INFO(logger_, "Activating controller: %s", plugin_name_.c_str());

  auto node = node_.lock();
  if (!node)
  {
    RCLCPP_ERROR(logger_, "Failed to lock node in activate");
    return;
  }
  // Timer for the callback function running
  auto period = std::chrono::duration<double, std::milli>(1000.0 / linearization_frequency);
  timer1_ = node->create_wall_timer(
    period,
    [this]() { this->timerCallback1(); }
  );
  // Publisher used to output xP, yP, v, and w from the linearization loop for testing
  test_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "test_pub", 10);

  RCLCPP_INFO(logger_, "Controller %s activated", plugin_name_.c_str());
}

void FeedbackLinearizationController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating controller: %s", plugin_name_.c_str());

  if (timer1_)
  {
    timer1_->cancel();
    timer1_.reset();
  }

  RCLCPP_INFO(logger_, "Controller %s deactivated", plugin_name_.c_str());
}

void FeedbackLinearizationController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  (void)speed_limit;
  (void)percentage;
  RCLCPP_INFO(logger_, "Speed limit set called, but not implemented.");
}

geometry_msgs::msg::TwistStamped FeedbackLinearizationController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)velocity;
  (void)goal_checker;
  // Mutex protecting the robot pose shared with the callback loop

  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_ = pose;
  }
  // Mutex protecting vPx and vPy variables received from the callback function
  double vPx, vPy;
  {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    vPx = vx_;
    vPy = vy_;
  }
  
  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double theta = tf2::getYaw(pose.pose.orientation);
  
  // Feedback linearization cancelling equation
  double linear_vel = vPx*std::cos(theta) + vPy*std::sin(theta);
  double angular_vel = (vPy*std::cos(theta) - vPx*std::sin(theta)) / epsilon_;
  // Publish cmd_vel from the controller plugin
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  // Compute the (xP, yP) point used for testing the feedback linearization
  double xP = x + epsilon_ * cos(theta);
  double yP = y + epsilon_ * sin(theta);
  
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {xP, yP, theta,vPx,vPy,linear_vel,angular_vel};
  // Publish xP, yP, v, and w for testing purposes
  test_pub_->publish(msg);
  return cmd_vel;
}
// Callback function triggered by the timer at 5 Hz for the linear control loop
void FeedbackLinearizationController::timerCallback1()
{
    geometry_msgs::msg::PoseStamped robot_pose;
    nav_msgs::msg::Path path_P;
    // Retrieve the current robot pose
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        robot_pose = current_pose_;
    }
    // Interpolated path converted from the global path into a trajectory
    {
        std::lock_guard<std::mutex> lock(mutex_);
         path_P.poses=interpolated_path_;
    }

    if (robot_pose.header.frame_id.empty()) {
        RCLCPP_WARN(logger_, "Robot pose frame is empty!");
        return;
    }
    static rclcpp::Time last_time = clock_->now();
    rclcpp::Time now = clock_->now();
    static geometry_msgs::msg::PoseStamped last_pose_cached;
    const auto & current_last_pose = interpolated_path_.back();
    // Simple check: if position changed
    if (last_pose_cached.pose.position.x != current_last_pose.pose.position.x ||
       last_pose_cached.pose.position.y != current_last_pose.pose.position.y ||
       last_pose_cached.pose.position.z != current_last_pose.pose.position.z)
    {
       RCLCPP_INFO(logger_,"im here");
       dt = 0;             // reset
       last_pose_cached = current_last_pose;  // update cache
        }
    dt = (now - last_time).seconds()+dt;
    last_time = now;
    if (!Test_feedback_linearization){
        if(false){
            xP_ref = 0.7 * dt + 0.0; 
            yP_ref = 0.0 * dt + -2.5;  
        }
        else {
            index_plan++;
            if (index_plan >= interpolated_path_.size()) {
              index_plan = interpolated_path_.size() - 1; // clamp to last element
            }

            geometry_msgs::msg::PoseStamped ref_pose = interpolated_path_[index_plan];

            double x_ref = ref_pose.pose.position.x;
            double y_ref = ref_pose.pose.position.y;
            double theta_ref = tf2::getYaw(ref_pose.pose.orientation);
            interpolateTrajectory(dt, path_sp_x_, path_sp_y_, path_sp_yaw_, x_ref, y_ref, theta_ref);
            xP_ref = x_ref + epsilon_ * cos(theta_ref);
            yP_ref = y_ref + epsilon_ * sin(theta_ref);        
        }
        double x = robot_pose.pose.position.x;
        double y = robot_pose.pose.position.y;
        double theta = tf2::getYaw(robot_pose.pose.orientation);
        
        double xP = x + epsilon_ * cos(theta);
        double yP = y + epsilon_ * sin(theta);
        
        double ex = xP_ref - xP;
        double ey = yP_ref - yP;   
        RCLCPP_INFO(logger_,
        "robot=(%.3f, %.3f, %.3f) ref=(%.3f, %.3f)",
        x, y, theta, xP_ref, yP_ref);
        vPx = Kp_ * ex;
        vPy = Kp_ * ey;
    }
    else{
    // Send the step command to test the linearization controller
    vPx = vpx_test;
    vPy = vpy_test;
    }
    
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        vx_ = vPx;
        vy_ = vPy;
    }
    
}

void FeedbackLinearizationController::setPlan(const nav_msgs::msg::Path & path){
    plan_ = path.poses;   
    nav_msgs::msg::Path path_P;
    // Store the coordinates and orientation (yaw) of each waypoint
    std::vector<double> path_x(path.poses.size(), 0.0);
    std::vector<double> path_y(path.poses.size(), 0.0);
    std::vector<double> path_yaw(path.poses.size(), 0.0);
    std::vector<double> path_s(path.poses.size(), 0.0);
    // Initial point of the path
    path_x[0] = path.poses[0].pose.position.x;
    path_y[0] = path.poses[0].pose.position.y;
    path_yaw[0] = tf2::getYaw(path.poses[0].pose.orientation);
    // Calculate the cumulative distance at each pose for interpolation
    for (size_t k = 1; k < path.poses.size(); k++) {
        path_s[k] = path_s[k-1] + 
                   std::hypot(path.poses[k].pose.position.x - path.poses[k-1].pose.position.x,
                              path.poses[k].pose.position.y - path.poses[k-1].pose.position.y);
        path_x[k] = path.poses[k].pose.position.x;
        path_y[k] = path.poses[k].pose.position.y;
        path_yaw[k] = tf2::getYaw(path.poses[k].pose.orientation);
    }
    
    
    // Cubic spline interpolation using ALGLIB to generate a smooth path
    try {
        alglib::real_1d_array s, x, y, yaw;
        s.setcontent(path_s.size(), path_s.data());
        x.setcontent(path_x.size(), path_x.data());
        y.setcontent(path_y.size(), path_y.data());
        yaw.setcontent(path_yaw.size(), path_yaw.data());

        spline1dbuildcubic(s, x, path_sp_x_);
        spline1dbuildcubic(s, y, path_sp_y_);
        spline1dbuildcubic(s, yaw, path_sp_yaw_);
    } catch(alglib::ap_error &e) {
        RCLCPP_ERROR(logger_, "ALGLIB exception: %s", e.msg.c_str());
    }
    // Calculate the total path length and trajectory duration
    path_length_ = path_s.back();
    path_duration_ = std::max(1.5 * path_length_ / (w_max_ * wheel_radius_), 
                      std::sqrt(6.0 * path_length_ / acc_lin_max_));
    path_time_ = 0;
    // Step size based on the callback frequency (5 Hz)
    double step = 1/linearization_frequency;
    
    for (auto time = 0.0; time <= path_duration_; time += step) {
       geometry_msgs::msg::Pose pose_P;
       geometry_msgs::msg::PoseStamped pose_stamped;
       double x_interp_step, y_interp_step, yaw_interp_step;

       // Interpolate the trajectory pose at each time step to generate a smooth trajectory
       if (time < path_duration_){
            interpolateTrajectory(time + step, path_sp_x_, path_sp_y_, path_sp_yaw_, x_interp_step, y_interp_step, yaw_interp_step);
       } else {
            interpolateTrajectory(time - step, path_sp_x_, path_sp_y_, path_sp_yaw_, x_interp_step, y_interp_step, yaw_interp_step);
       }

       pose_P.position.x = x_interp_step;
       pose_P.position.y = y_interp_step;
       pose_P.position.z = 0.0;

       tf2::Quaternion q;
       q.setRPY(0.0, 0.0, yaw_interp_step);
       pose_P.orientation = tf2::toMsg(q);
       pose_stamped.pose = pose_P;
       path_P.poses.push_back(pose_stamped);
       // Mutex protecting the trajectory poses vector
       {
        std::lock_guard<std::mutex> lock(mutex_);
        interpolated_path_ = path_P.poses;
        }
    }
    
        
}
// Interpolation function converting the path into a time-parameterized trajectory using ALGLIB
void FeedbackLinearizationController::interpolateTrajectory(const double& t, const alglib::spline1dinterpolant& path_sp_x_, 
    const alglib::spline1dinterpolant& path_sp_y_, const alglib::spline1dinterpolant& path_sp_yaw_, double& x, double& y, double& yaw) {
    double s = path_length_ * (3.0 * std::pow(std::min(t/path_duration_, 1.0), 2.0) - 
                               2.0 * std::pow(std::min(t/path_duration_, 1.0), 3.0));

    x = spline1dcalc(path_sp_x_, s);
    y = spline1dcalc(path_sp_y_, s);
    yaw = spline1dcalc(path_sp_yaw_, s);

}

}  // namespace feedback_linearization_controller

PLUGINLIB_EXPORT_CLASS(feedback_linearization_controller::FeedbackLinearizationController, nav2_core::Controller)
