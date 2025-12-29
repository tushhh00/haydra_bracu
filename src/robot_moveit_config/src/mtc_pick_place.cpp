#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_pick_place");
namespace mtc = moveit::task_constructor;

class MTCPickPlaceNode
{
public:
  MTCPickPlaceNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();
  void setupPlanningScene();

private:
  mtc::Task createTask();
  void objectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  
  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Subscriber for detected object pose
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  
  // Latest detected object pose
  geometry_msgs::msg::PoseStamped::SharedPtr latest_object_pose_;
  std::mutex pose_mutex_;
  
  // Robot configuration
  std::string arm_group_name_;
  std::string gripper_group_name_;
  std::string ee_frame_;
  std::string world_frame_;
  
  // Object parameters
  double object_radius_;
  double object_height_;
  
  // Place position
  double place_x_;
  double place_y_;
  double place_z_;
};

MTCPickPlaceNode::MTCPickPlaceNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_pick_place", options) }
{
  // Declare parameters
  node_->declare_parameter("arm_group", "manipulator");
  node_->declare_parameter("gripper_group", "gripper");
  node_->declare_parameter("ee_frame", "ee_link");
  node_->declare_parameter("world_frame", "world");
  node_->declare_parameter("object_radius", 0.025);
  node_->declare_parameter("object_height", 0.1);
  node_->declare_parameter("place_x", 0.3);
  node_->declare_parameter("place_y", 0.3);
  node_->declare_parameter("place_z", 0.1);
  
  // Get parameters
  arm_group_name_ = node_->get_parameter("arm_group").as_string();
  gripper_group_name_ = node_->get_parameter("gripper_group").as_string();
  ee_frame_ = node_->get_parameter("ee_frame").as_string();
  world_frame_ = node_->get_parameter("world_frame").as_string();
  object_radius_ = node_->get_parameter("object_radius").as_double();
  object_height_ = node_->get_parameter("object_height").as_double();
  place_x_ = node_->get_parameter("place_x").as_double();
  place_y_ = node_->get_parameter("place_y").as_double();
  place_z_ = node_->get_parameter("place_z").as_double();
  
  // Setup TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Subscribe to detected object pose
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/detected_object/pose", 10,
    std::bind(&MTCPickPlaceNode::objectPoseCallback, this, std::placeholders::_1));
  
  RCLCPP_INFO(LOGGER, "MTC Pick Place Node initialized");
  RCLCPP_INFO(LOGGER, "  Arm group: %s", arm_group_name_.c_str());
  RCLCPP_INFO(LOGGER, "  Gripper group: %s", gripper_group_name_.c_str());
  RCLCPP_INFO(LOGGER, "  EE frame: %s", ee_frame_.c_str());
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCPickPlaceNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCPickPlaceNode::objectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  
  // Transform to world frame if needed
  if (msg->header.frame_id != world_frame_)
  {
    try
    {
      geometry_msgs::msg::PoseStamped transformed_pose;
      transformed_pose = tf_buffer_->transform(*msg, world_frame_, tf2::durationFromSec(0.1));
      latest_object_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(transformed_pose);
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN(LOGGER, "Could not transform object pose: %s", ex.what());
      return;
    }
  }
  else
  {
    latest_object_pose_ = msg;
  }
  
  RCLCPP_INFO(LOGGER, "Received object pose: (%.3f, %.3f, %.3f)",
              latest_object_pose_->pose.position.x,
              latest_object_pose_->pose.position.y,
              latest_object_pose_->pose.position.z);
}

void MTCPickPlaceNode::setupPlanningScene()
{
  // Wait for object pose
  RCLCPP_INFO(LOGGER, "Waiting for detected object pose...");
  
  rclcpp::Rate rate(10);
  while (rclcpp::ok())
  {
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      if (latest_object_pose_)
      {
        break;
      }
    }
    rclcpp::spin_some(node_);
    rate.sleep();
  }
  
  geometry_msgs::msg::PoseStamped object_pose;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    object_pose = *latest_object_pose_;
  }
  
  RCLCPP_INFO(LOGGER, "Using object at position: (%.3f, %.3f, %.3f)",
              object_pose.pose.position.x,
              object_pose.pose.position.y,
              object_pose.pose.position.z);
  
  // Create collision object
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = world_frame_;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { object_height_, object_radius_ };
  
  object.pose = object_pose.pose;
  // Adjust Z to place cylinder on surface (cylinder origin is at center)
  object.pose.position.z += object_height_ / 2.0;
  
  // Add table surface
  moveit_msgs::msg::CollisionObject table;
  table.id = "table";
  table.header.frame_id = world_frame_;
  table.primitives.resize(1);
  table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  table.primitives[0].dimensions = { 1.0, 1.0, 0.01 };  // 1m x 1m table
  
  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = object_pose.pose.position.x;
  table_pose.position.y = object_pose.pose.position.y;
  table_pose.position.z = object_pose.pose.position.z - 0.005;  // Just below object
  table_pose.orientation.w = 1.0;
  table.pose = table_pose;
  
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
  psi.applyCollisionObject(table);
  
  RCLCPP_INFO(LOGGER, "Planning scene setup complete");
}

void MTCPickPlaceNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  
  task_.introspection().publishSolution(*task_.solutions().front());
  
  RCLCPP_INFO(LOGGER, "Task planning succeeded! Execute? (Press Enter to execute, Ctrl+C to cancel)");
  std::cin.get();

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with code: " << result.val);
    return;
  }

  RCLCPP_INFO(LOGGER, "Task execution completed successfully!");
}

mtc::Task MTCPickPlaceNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("pick and place task");
  task.loadRobotModel(node_);

  // Set task properties
  task.setProperty("group", arm_group_name_);
  task.setProperty("eef", gripper_group_name_);
  task.setProperty("ik_frame", ee_frame_);

  // Stage pointers for connections
  mtc::Stage* current_state_ptr = nullptr;
  mtc::Stage* attach_object_stage = nullptr;

  // ==================== Current State ====================
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current state");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  // ==================== Planners ====================
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(0.01);

  // ==================== Move to Ready ====================
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to ready", interpolation_planner);
    stage->setGroup(arm_group_name_);
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  // ==================== PICK STAGES ====================
  // Connect to pick position
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  // Pick serial container
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // Approach object
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", ee_frame_);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.15);

      // Set approach direction (Z down in world frame)
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame_;
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // Generate grasp pose
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("home");  // Use home as open gripper state
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);

      // Grasp frame transform - adjust based on your gripper
      Eigen::Isometry3d grasp_frame_transform;
      grasp_frame_transform.setIdentity();
      // Rotate to align gripper with object
      Eigen::Quaterniond q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.05;  // Offset from EE to grasp point

      // Compute IK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, ee_frame_);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    // Allow collision (gripper, object)
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(gripper_group_name_)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      grasp->insert(std::move(stage));
    }

    // Close gripper (move to grasp configuration)
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
      stage->setGroup(gripper_group_name_);
      stage->setGoal("home");  // You may need to define "closed" pose in SRDF
      grasp->insert(std::move(stage));
    }

    // Attach object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", ee_frame_);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    // Lift object
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.2);
      stage->setIKFrame(ee_frame_);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame_;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  // ==================== PLACE STAGES ====================
  // Connect to place position
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  // Place serial container
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // Generate place pose
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      // Set place position
      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = world_frame_;
      target_pose_msg.pose.position.x = place_x_;
      target_pose_msg.pose.position.y = place_y_;
      target_pose_msg.pose.position.z = place_z_ + object_height_ / 2.0;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);

      // Compute IK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(ee_frame_);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    // Open gripper
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
      stage->setGroup(gripper_group_name_);
      stage->setGoal("home");
      place->insert(std::move(stage));
    }

    // Forbid collision (gripper, object)
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (gripper,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(gripper_group_name_)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      place->insert(std::move(stage));
    }

    // Detach object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", ee_frame_);
      place->insert(std::move(stage));
    }

    // Retreat
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.2);
      stage->setIKFrame(ee_frame_);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction (up)
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame_;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  // ==================== Return Home ====================
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->setGroup(arm_group_name_);
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCPickPlaceNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
