/**
 * Simple Pick and Place with MoveIt2
 * 
 * This node demonstrates how to:
 * 1. Receive 3D object positions from the camera
 * 2. Plan arm movements to the object
 * 3. Execute pick and place operations
 * 
 * It uses the MoveGroupInterface which is simpler than MTC
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class SimplePickPlace : public rclcpp::Node
{
public:
  SimplePickPlace() : Node("simple_pick_place")
  {
    // Parameters
    this->declare_parameter("arm_group", "manipulator");
    this->declare_parameter("gripper_group", "gripper");
    this->declare_parameter("world_frame", "world");
    this->declare_parameter("ee_frame", "ee_link");
    this->declare_parameter("place_x", 0.2);
    this->declare_parameter("place_y", 0.2);
    this->declare_parameter("place_z", 0.15);
    this->declare_parameter("approach_distance", 0.1);
    this->declare_parameter("lift_distance", 0.1);
    
    // Camera-to-robot transform (if camera is not in TF tree)
    // These represent the camera position in robot base frame
    this->declare_parameter("use_manual_camera_transform", true);
    this->declare_parameter("camera_x", 0.3);   // Camera is 0.3m in front of robot base
    this->declare_parameter("camera_y", 0.0);   // Camera is centered
    this->declare_parameter("camera_z", 0.5);   // Camera is 0.5m above robot base
    // Camera is looking down (typical setup)
    this->declare_parameter("camera_pitch", -1.57);  // -90 degrees (looking down)
    
    arm_group_name_ = this->get_parameter("arm_group").as_string();
    gripper_group_name_ = this->get_parameter("gripper_group").as_string();
    world_frame_ = this->get_parameter("world_frame").as_string();
    ee_frame_ = this->get_parameter("ee_frame").as_string();
    place_x_ = this->get_parameter("place_x").as_double();
    place_y_ = this->get_parameter("place_y").as_double();
    place_z_ = this->get_parameter("place_z").as_double();
    approach_distance_ = this->get_parameter("approach_distance").as_double();
    lift_distance_ = this->get_parameter("lift_distance").as_double();
    
    use_manual_camera_transform_ = this->get_parameter("use_manual_camera_transform").as_bool();
    camera_x_ = this->get_parameter("camera_x").as_double();
    camera_y_ = this->get_parameter("camera_y").as_double();
    camera_z_ = this->get_parameter("camera_z").as_double();
    camera_pitch_ = this->get_parameter("camera_pitch").as_double();
    
    // TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Subscribe to detected object pose
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/detected_object/pose", 10,
      std::bind(&SimplePickPlace::poseCallback, this, std::placeholders::_1));
    
    // Service to trigger pick and place
    pick_place_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/pick_and_place",
      std::bind(&SimplePickPlace::pickPlaceCallback, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    // Timer for continuous operation mode (optional)
    // timer_ = this->create_wall_timer(5s, std::bind(&SimplePickPlace::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Simple Pick Place Node Started");
    RCLCPP_INFO(this->get_logger(), "  Arm group: %s", arm_group_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Place position: (%.2f, %.2f, %.2f)", place_x_, place_y_, place_z_);
    RCLCPP_INFO(this->get_logger(), "Call service ~/pick_and_place to execute");
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    try {
      if (use_manual_camera_transform_) {
        // Manual transform from camera frame to robot frame
        // Camera optical frame: Z forward, X right, Y down
        // Robot frame: X forward, Y left, Z up
        
        // Get position in camera frame
        double cam_x = msg->pose.position.x;  // right in camera frame
        double cam_y = msg->pose.position.y;  // down in camera frame
        double cam_z = msg->pose.position.z;  // forward in camera frame
        
        // Transform to robot base frame
        // Assuming camera is looking down (pitched -90 degrees):
        // Camera Z (forward) -> Robot -Z (down)
        // Camera X (right) -> Robot Y (left, negated)
        // Camera Y (down) -> Robot X (forward, negated)
        
        latest_pose_.header.frame_id = world_frame_;
        latest_pose_.header.stamp = msg->header.stamp;
        latest_pose_.pose.position.x = camera_x_ - cam_y;  // cam down -> robot forward
        latest_pose_.pose.position.y = camera_y_ - cam_x;  // cam right -> robot left (negated)
        latest_pose_.pose.position.z = camera_z_ - cam_z;  // cam forward -> robot down (negated)
        latest_pose_.pose.orientation.w = 1.0;
        
        has_pose_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Object detected! Camera: (%.3f, %.3f, %.3f) -> Robot: (%.3f, %.3f, %.3f)",
                   cam_x, cam_y, cam_z,
                   latest_pose_.pose.position.x,
                   latest_pose_.pose.position.y,
                   latest_pose_.pose.position.z);
      }
      else if (msg->header.frame_id != world_frame_) {
        // Use TF transform
        latest_pose_ = tf_buffer_->transform(*msg, world_frame_, tf2::durationFromSec(0.5));
        has_pose_ = true;
      } else {
        latest_pose_ = *msg;
        has_pose_ = true;
      }
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
    }
  }
  
  void pickPlaceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    
    if (!has_pose_) {
      response->success = false;
      response->message = "No object detected yet";
      return;
    }
    
    bool success = executePickAndPlace();
    response->success = success;
    response->message = success ? "Pick and place completed" : "Pick and place failed";
  }
  
  bool executePickAndPlace()
  {
    RCLCPP_INFO(this->get_logger(), "Starting pick and place sequence...");
    
    // Create MoveGroupInterface for arm
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), arm_group_name_);
    
    // Set planning parameters
    move_group->setPlanningTime(10.0);
    move_group->setNumPlanningAttempts(10);
    move_group->setMaxVelocityScalingFactor(0.5);
    move_group->setMaxAccelerationScalingFactor(0.5);
    
    // Planning scene interface for collision objects
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    // Get current object position
    geometry_msgs::msg::PoseStamped object_pose = latest_pose_;
    
    RCLCPP_INFO(this->get_logger(), "Object position: (%.3f, %.3f, %.3f)",
                object_pose.pose.position.x,
                object_pose.pose.position.y,
                object_pose.pose.position.z);
    
    // ========== STEP 1: Move to Ready Position ==========
    RCLCPP_INFO(this->get_logger(), "Step 1: Moving to ready position...");
    move_group->setNamedTarget("ready");
    if (!executePlan(move_group)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to ready position");
      return false;
    }
    
    // ========== STEP 2: Move to Pre-Grasp (Above Object) ==========
    RCLCPP_INFO(this->get_logger(), "Step 2: Moving to pre-grasp position...");
    geometry_msgs::msg::Pose pre_grasp_pose;
    pre_grasp_pose.position.x = object_pose.pose.position.x;
    pre_grasp_pose.position.y = object_pose.pose.position.y;
    pre_grasp_pose.position.z = object_pose.pose.position.z + approach_distance_;
    // Point gripper down (adjust based on your robot's orientation)
    pre_grasp_pose.orientation.x = 0.0;
    pre_grasp_pose.orientation.y = 0.707;  // 90 degrees around Y
    pre_grasp_pose.orientation.z = 0.0;
    pre_grasp_pose.orientation.w = 0.707;
    
    move_group->setPoseTarget(pre_grasp_pose);
    if (!executePlan(move_group)) {
      RCLCPP_WARN(this->get_logger(), "Failed to reach pre-grasp, trying alternative approach");
      // Try with joint space planning instead
      return false;
    }
    
    // ========== STEP 3: Approach (Move Down) ==========
    RCLCPP_INFO(this->get_logger(), "Step 3: Approaching object...");
    geometry_msgs::msg::Pose grasp_pose = pre_grasp_pose;
    grasp_pose.position.z = object_pose.pose.position.z + 0.02;  // Slightly above object
    
    // Use Cartesian path for smooth approach
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(grasp_pose);
    
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    
    if (fraction < 0.9) {
      RCLCPP_WARN(this->get_logger(), "Cartesian approach incomplete (%.1f%%)", fraction * 100);
      // Fallback to pose target
      move_group->setPoseTarget(grasp_pose);
      if (!executePlan(move_group)) {
        return false;
      }
    } else {
      move_group->execute(trajectory);
    }
    
    // ========== STEP 4: Close Gripper (Simulated) ==========
    RCLCPP_INFO(this->get_logger(), "Step 4: Closing gripper...");
    // For a real gripper, you would call the gripper action here
    // For now, we just add a delay to simulate gripper closing
    rclcpp::sleep_for(500ms);
    
    // Add collision object to simulate attached object
    addCollisionObject(planning_scene_interface, object_pose);
    rclcpp::sleep_for(200ms);
    
    // Attach object to gripper
    move_group->attachObject("target_object", ee_frame_);
    
    // ========== STEP 5: Lift Object ==========
    RCLCPP_INFO(this->get_logger(), "Step 5: Lifting object...");
    geometry_msgs::msg::Pose lift_pose = grasp_pose;
    lift_pose.position.z += lift_distance_;
    
    waypoints.clear();
    waypoints.push_back(lift_pose);
    fraction = move_group->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    
    if (fraction < 0.9) {
      move_group->setPoseTarget(lift_pose);
      if (!executePlan(move_group)) {
        return false;
      }
    } else {
      move_group->execute(trajectory);
    }
    
    // ========== STEP 6: Move to Place Position ==========
    RCLCPP_INFO(this->get_logger(), "Step 6: Moving to place position...");
    geometry_msgs::msg::Pose place_pose;
    place_pose.position.x = place_x_;
    place_pose.position.y = place_y_;
    place_pose.position.z = place_z_ + lift_distance_;
    place_pose.orientation = pre_grasp_pose.orientation;
    
    move_group->setPoseTarget(place_pose);
    if (!executePlan(move_group)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to place position");
      return false;
    }
    
    // ========== STEP 7: Lower Object ==========
    RCLCPP_INFO(this->get_logger(), "Step 7: Lowering object...");
    geometry_msgs::msg::Pose lower_pose = place_pose;
    lower_pose.position.z = place_z_;
    
    waypoints.clear();
    waypoints.push_back(lower_pose);
    fraction = move_group->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    
    if (fraction >= 0.9) {
      move_group->execute(trajectory);
    } else {
      move_group->setPoseTarget(lower_pose);
      executePlan(move_group);
    }
    
    // ========== STEP 8: Open Gripper & Release ==========
    RCLCPP_INFO(this->get_logger(), "Step 8: Opening gripper...");
    move_group->detachObject("target_object");
    rclcpp::sleep_for(500ms);
    
    // ========== STEP 9: Retreat ==========
    RCLCPP_INFO(this->get_logger(), "Step 9: Retreating...");
    geometry_msgs::msg::Pose retreat_pose = lower_pose;
    retreat_pose.position.z += lift_distance_;
    
    waypoints.clear();
    waypoints.push_back(retreat_pose);
    fraction = move_group->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    
    if (fraction >= 0.9) {
      move_group->execute(trajectory);
    }
    
    // ========== STEP 10: Return Home ==========
    RCLCPP_INFO(this->get_logger(), "Step 10: Returning home...");
    move_group->setNamedTarget("home");
    executePlan(move_group);
    
    // Clean up collision object
    planning_scene_interface.removeCollisionObjects({"target_object"});
    
    RCLCPP_INFO(this->get_logger(), "Pick and place completed successfully!");
    return true;
  }
  
  bool executePlan(std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Plan found, executing...");
      move_group->execute(plan);
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
      return false;
    }
  }
  
  void addCollisionObject(
    moveit::planning_interface::PlanningSceneInterface& psi,
    const geometry_msgs::msg::PoseStamped& object_pose)
  {
    // Create collision object representing the target
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = world_frame_;
    collision_object.id = "target_object";
    
    // Define object shape (cylinder approximating the object)
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.08;  // height
    primitive.dimensions[1] = 0.025; // radius
    
    // Object pose
    geometry_msgs::msg::Pose obj_pose;
    obj_pose.position = object_pose.pose.position;
    obj_pose.position.z += 0.04;  // Center of cylinder
    obj_pose.orientation.w = 1.0;
    
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(obj_pose);
    collision_object.operation = collision_object.ADD;
    
    psi.applyCollisionObject(collision_object);
  }
  
  // Member variables
  std::string arm_group_name_;
  std::string gripper_group_name_;
  std::string world_frame_;
  std::string ee_frame_;
  double place_x_, place_y_, place_z_;
  double approach_distance_;
  double lift_distance_;
  
  // Manual camera transform parameters
  bool use_manual_camera_transform_;
  double camera_x_, camera_y_, camera_z_;
  double camera_pitch_;
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pick_place_srv_;
  
  geometry_msgs::msg::PoseStamped latest_pose_;
  bool has_pose_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<SimplePickPlace>();
  
  // Use multi-threaded executor for MoveIt
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
