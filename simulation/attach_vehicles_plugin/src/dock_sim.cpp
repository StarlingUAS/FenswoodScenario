#include "dock_sim.hpp"

namespace gazebo {

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(DockSim)

// Constructor
DockSim::DockSim()
  : world(nullptr),
	physics(nullptr),
	joint(nullptr),
	drone(nullptr),
	rover(nullptr),
	jointCounter(0)
	{
	}


// Destructor
DockSim::~DockSim() {}

void DockSim::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {

	// Create a GazeboRos node instead of a common ROS node.
	// Pass it SDF parameters so common options like namespace and remapping
	// can be handled.
	ros_node_ = gazebo_ros::Node::Get(_sdf);

	this->world = _world;
	this->physics = this->world->Physics();

	this->rover_model_name = "turtlebot_1";
	this->drone_model_name = "iris_1";

	// // Get parameters specified in the sdf file.
	if (_sdf->HasElement("drone_model")) {
		this->drone_model_name = _sdf->Get<std::string>("drone_model");
	} else {
		this->drone_model_name = "drone";
	}
	if (_sdf->HasElement("payload_model")) {
		this->rover_model_name = _sdf->Get<std::string>("payload_model");
	} else {
		this->rover_model_name = "sensor_payload";
	}
	if (_sdf->HasElement("allowable_offset_height")) {
		this->allowable_offset_height = _sdf->Get<double>("allowable_offset_height");
	}
	if (_sdf->HasElement("allowable_offset_horizontal")) {
		this->allowable_offset_height = _sdf->Get<double>("allowable_offset_horizontal");
	}

	// Create Publisher and Subscriber for ROS2 to interact with this
	this->dockStatusPub = this->ros_node_->create_publisher<std_msgs::msg::Bool>("status", 1);
	this->dockControlSub = this->ros_node_->create_subscription<std_msgs::msg::Bool>(
		"control",1,
		[this](const std_msgs::msg::Bool::SharedPtr dockControl) {
			if (dockControl->data) {
				auto result = this->attach();
				std_msgs::msg::Bool status_msg;
				status_msg.data = result;
				dockStatusPub->publish(status_msg);
			} else {
				auto result = this->detach();
				std_msgs::msg::Bool status_msg;
				status_msg.data = !result; // status of false means detached
				dockStatusPub->publish(status_msg);
			}
		}
	);

	RCLCPP_INFO(this->ros_node_->get_logger(), "Publisher initialised on [%s]", this->dockStatusPub->get_topic_name());
	RCLCPP_INFO(this->ros_node_->get_logger(), "Subscriber initialised on [%s]", this->dockControlSub->get_topic_name());

	bool initially_attach = true;
	if(initially_attach) {
		RCLCPP_INFO(this->ros_node_->get_logger(), "Attempting to initially attach");
		this->initial_attaching_timer = this->ros_node_->create_wall_timer(std::chrono::duration<double>(0.1), [this](){this->attach();});
	}
}

bool DockSim::attach() {
	if( !this->rover ) {
		this->rover = world->ModelByName(this->rover_model_name);
		if( !this->rover ) {
			RCLCPP_ERROR(this->ros_node_->get_logger(), "Could not find payload model [%s]", this->rover_model_name.c_str());
			return false;
			}
		}

	if( !this->drone ) {
		this->drone = world->ModelByName(this->drone_model_name);
		if( !this->drone ) {
			RCLCPP_ERROR(this->ros_node_->get_logger(), "Could not find drone model [%s]", this->drone_model_name.c_str());
			return false;
			}
		}

	// RCLCPP_INFO(this->ros_node_->get_logger(), "getting drone base link");
	auto child_link = this->drone->GetLink("iris::base_link");

	if (!child_link){
		RCLCPP_ERROR(this->ros_node_->get_logger(), "child link from drone is none, here is a list of links");
		auto links1 = this->drone->GetLinks();
		for(auto a: links1){
			RCLCPP_INFO(this->ros_node_->get_logger(), a->GetName());
		}
		return false;
	}

	// RCLCPP_INFO(this->ros_node_->get_logger(), "getting payload base link");
	auto parent_link = this->rover->GetLink("base_link");

	if (!parent_link){
		RCLCPP_INFO(this->ros_node_->get_logger(), "parent link from payload is none, here is a list of links");
		auto links = this->rover->GetLinks();
		for(auto a: links){
				RCLCPP_INFO(this->ros_node_->get_logger(), a->GetName());
		}
		return false;
	}

	if( this->joint ) {
		RCLCPP_INFO(this->ros_node_->get_logger(), "Already attached.");
		if (this->initial_attaching_timer) {
			RCLCPP_INFO(this->ros_node_->get_logger(), "Initial attach succesful, timer cancelling");
			this->initial_attaching_timer->cancel();
		}
		return true;
	}


	// Test if links are within docking tolerance
	RCLCPP_INFO(this->ros_node_->get_logger(), "Testing tolerance");
	auto poseOffset = parent_link->WorldPose() - child_link->WorldPose();
	bool inToleranceHeight = poseOffset.Pos().Z() < this->allowable_offset_height;
	poseOffset.Pos().Z(0.0);
	bool inToleranceHorizontal = poseOffset.Pos().SquaredLength() < this->allowable_offset_horizontal;
	bool inTolerance = inToleranceHeight && inToleranceHorizontal;

	if(!inTolerance) {
		RCLCPP_WARN(this->ros_node_->get_logger(), "Drone not within object tolerance %f", poseOffset.Pos().SquaredLength());
		return false;
	}

	RCLCPP_INFO(this->ros_node_->get_logger(), "Creating new joint.");
	std::stringstream jointName;
	jointName << "sim_dock_joint_" << jointCounter;
	joint = drone->CreateJoint(jointName.str(),"fixed",parent_link,child_link);
	if( !joint ) {
		RCLCPP_ERROR(this->ros_node_->get_logger(), "Could not create joint");
		return false;
	}
	jointCounter++;
	joint->Load(parent_link,child_link, ignition::math::Pose3d());
	joint->Init();

	RCLCPP_INFO(this->ros_node_->get_logger(), "Attaching joint");
	joint->Attach(parent_link,child_link);
	return true;
	}

bool DockSim::detach() {
	if(!this->joint) {
		// Joint doesn't exist so detached...
		return true;
		}
	RCLCPP_INFO(this->ros_node_->get_logger(), "Detaching joint");
	this->joint->Fini();
	this->joint->~Joint();
	this->joint = nullptr;
	return true;
	}

} // namespace gazebo
