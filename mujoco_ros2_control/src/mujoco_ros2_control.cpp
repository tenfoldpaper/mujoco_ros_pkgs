#include "mujoco_ros2_control/mujoco_ros2_control.hpp"
#include <set>

namespace mujoco_ros2_control {

std::string MujocoRos2ControlPluginPrivate::getURDF() const
{
	std::string urdf_string;

	using namespace std::chrono_literals;
	auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(node_, robot_description_node_);
	while (!parameters_client->wait_for_service(0.5s)) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for %s service. Exiting.",
			             robot_description_node_.c_str());
			return 0;
		}
		RCLCPP_ERROR(node_->get_logger(), "%s service not available, waiting again...", robot_description_node_.c_str());
	}

	RCLCPP_INFO(node_->get_logger(), "connected to service. %s asking for %s", robot_description_node_.c_str(),
	            this->robot_description_.c_str());

	// search and wait for robot_description on param server
	while (urdf_string.empty()) {
		RCLCPP_DEBUG(node_->get_logger(), "param_name %s", this->robot_description_.c_str());

		try {
			auto f = parameters_client->get_parameters({ this->robot_description_ });
			f.wait();
			std::vector<rclcpp::Parameter> values = f.get();
			urdf_string                           = values[0].as_string();
		} catch (const std::exception &e) {
			RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
		}

		if (!urdf_string.empty()) {
			break;
		} else {
			RCLCPP_ERROR(node_->get_logger(),
			             "mujoco_ros2_control plugin is waiting for model"
			             " URDF in parameter [%s] on the ROS param server.",
			             this->robot_description_.c_str());
		}
		std::this_thread::sleep_for(std::chrono::microseconds(100000));
	}
	RCLCPP_INFO(node_->get_logger(), "Received URDF from param server");

	return urdf_string;
}

// MujocoRos2ControlPlugin::MujocoRos2ControlPlugin()
// {

// }

MujocoRos2ControlPlugin::~MujocoRos2ControlPlugin()
{
	// Stop controller manager thread
	if (!this->dataPtr_->controller_manager_) {
		return;
	}
	this->dataPtr_->executor_->remove_node(this->dataPtr_->controller_manager_);
	this->dataPtr_->executor_->cancel();
	this->dataPtr_->thread_executor_spin_.join();
}

void MujocoRos2ControlPlugin::controlCallback(int model, int data)
{
	RCLCPP_INFO_STREAM(get_my_logger(), "controlCallback w/ " << model << " " << data);
};
void MujocoRos2ControlPlugin::passiveCallback(int model, int data)
{
	RCLCPP_INFO_STREAM(get_my_logger(), "passiveCallback w/ " << model << " " << data);
};
void MujocoRos2ControlPlugin::renderCallback(int model, int data, int scene)
{
	RCLCPP_INFO_STREAM(get_my_logger(), "renderCallback w/ " << model << " " << data << " " << scene);
};
void MujocoRos2ControlPlugin::lastStageCallback(int model, int data)
{
	RCLCPP_INFO_STREAM(get_my_logger(), "lastStageCallback w/ " << model << " " << data);
};
void MujocoRos2ControlPlugin::onGeomChanged(int model, int data, const int geom_id)
{
	RCLCPP_INFO_STREAM(get_my_logger(), "onGeomChanged w/ " << model << " " << data << " " << geom_id);
};

bool MujocoRos2ControlPlugin::load(const mjModel *model, mjData *data)
{
	dataPtr_ = std::make_unique<MujocoRos2ControlPluginPrivate>();
	
	std::set<std::string> yaml_keys{};
  	std::set<std::string>::iterator it;
	RCLCPP_INFO_STREAM(get_my_logger(), "loading with given model and data");
	// construct a set of yaml node's keys to make param processing more concise
	for (auto it = yaml_node_.begin(); it != yaml_node_.end(); ++it) {
		YAML::Node key   = it->first;
		yaml_keys.insert(key.as<std::string>());
	}
	
	// get the name of the robot_state_publisher node
	if(yaml_keys.find(std::string("robot_description_node")) != yaml_keys.end()){
		std::string robot_description_node = yaml_node_["robot_description_node"].as<std::string>();
		if (!robot_description_node.empty()) {
			this->dataPtr_->robot_description_node_ = robot_description_node;
		}
	}
	RCLCPP_INFO(
		get_my_logger(),
		"robot_description_node is %s", this->dataPtr_->robot_description_node_.c_str()
	);
	// get the name of the srv from the node above that holds the URDF string
	if(yaml_keys.find(std::string("robot_description_node")) != yaml_keys.end()){
		std::string robot_description = yaml_node_["robot_description"].as<std::string>();
		if (!robot_description.empty()) {
			this->dataPtr_->robot_description_ = robot_description;
		}
	}
	RCLCPP_INFO(
		get_my_logger(),
		"robot_description srv is %s", this->dataPtr_->robot_description_.c_str()
	);
	// todo: Make the logic for passing additional ros-args when initializing, like L292~ in ign_ros2_control_plugin.cpp

	// Construct the fully qualified robot_description_node's name.
	// The namespace will also be used for the nodes of this.
	std::string ns = "/";
	// Set namespace if tag is present
    if (yaml_keys.find(std::string("namespace")) != yaml_keys.end()) {
      	ns = yaml_node_["namespace"].as<std::string>();
		RCLCPP_INFO(get_my_logger(), "Namespace: %s", ns.c_str());
		// prevent exception: namespace must be absolute, it must lead with a '/'
		if (ns.empty() || ns[0] != '/') {
			ns = '/' + ns;
		}
		if (ns.length() > 1) {
			this->dataPtr_->robot_description_node_ = ns + "/" + this->dataPtr_->robot_description_node_;
		}
    }
	RCLCPP_INFO(get_my_logger(), "robot_description_node fully qualified name: %s", this->dataPtr_->robot_description_node_.c_str());


	// Create a default context, if not already
	if (!rclcpp::ok()) {
  		std::vector<const char *> _argv;
		rclcpp::init(static_cast<int>(_argv.size()), _argv.data()); // todo: make the logic for passing additional ros-args
	}
	std::string node_name = "mujoco_ros2_control";

	this->dataPtr_->node_ = rclcpp::Node::make_shared(node_name, ns);
	this->dataPtr_->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
	this->dataPtr_->executor_->add_node(this->dataPtr_->node_);
	auto spin = [this]()
		{
		this->dataPtr_->executor_->spin();
		};
	this->dataPtr_->thread_executor_spin_ = std::thread(spin);
	RCLCPP_INFO_STREAM(get_my_logger(), "Making node with : " << node_name.c_str());

	// Read urdf from ros parameter server then
	// setup actuators and mechanism control node.
	// This call will block if ROS is not properly initialized.
	std::string urdf_string;
	std::vector<hardware_interface::HardwareInfo> control_hardware_info;
	try {
		urdf_string = this->dataPtr_->getURDF();
		control_hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_string);
	} catch (const std::runtime_error & ex) {
		RCLCPP_ERROR_STREAM(
		this->dataPtr_->node_->get_logger(),
		"Error parsing URDF in mujoco_ros2_control plugin, plugin not active : " << ex.what());
		return false;
	}

	std::unique_ptr<hardware_interface::ResourceManager> resource_manager_ =
    	std::make_unique<hardware_interface::ResourceManager>();

	try {
		resource_manager_->load_urdf(urdf_string, false, false);
	} catch (...) {
		RCLCPP_ERROR(
		this->dataPtr_->node_->get_logger(), "Error initializing URDF to resource manager!");
	}
	try {
		this->dataPtr_->robot_hw_sim_loader_.reset(
		new pluginlib::ClassLoader<mujoco_ros2_control::MujocoRos2SystemInterface>(
			"mujoco_ros2_control",
			"mujoco_ros2_control::MujocoRos2SystemInterface"));
	} catch (pluginlib::LibraryLoadException & ex) {
		RCLCPP_ERROR(
		this->dataPtr_->node_->get_logger(), "Failed to create robot simulation interface loader: %s ",
		ex.what());
		return false;
	}
	for (unsigned int i = 0; i < control_hardware_info.size(); ++i) {
		std::string robot_hw_sim_type_str_ = control_hardware_info[i].hardware_class_type;
		std::unique_ptr<mujoco_ros2_control::MujocoRos2SystemInterface> mujocoRos2System;
		RCLCPP_DEBUG(
		this->dataPtr_->node_->get_logger(), "Load hardware interface %s ...",
		robot_hw_sim_type_str_.c_str());
		try {
			mujocoRos2System = std::unique_ptr<mujoco_ros2_control::MujocoRos2SystemInterface>(
				this->dataPtr_->robot_hw_sim_loader_->createUnmanagedInstance(robot_hw_sim_type_str_));
			RCLCPP_INFO(this->dataPtr_->node_->get_logger(), "createUnmanagedInstance");
		} catch (pluginlib::PluginlibException & ex) {
			RCLCPP_ERROR(
				this->dataPtr_->node_->get_logger(),
				"The plugin failed to load for some reason. Error: %s\n",
				ex.what());
			continue;
		}
		if (!mujocoRos2System->initSim(
			this->dataPtr_->node_,
			control_hardware_info[i],
			this->dataPtr_->update_rate))
		{
			RCLCPP_FATAL(
				this->dataPtr_->node_->get_logger(), "Could not initialize robot simulation interface");
				return false;	
		}
		RCLCPP_DEBUG(
			this->dataPtr_->node_->get_logger(), "Initialized robot simulation interface %s!",
			robot_hw_sim_type_str_.c_str());
		

		resource_manager_->import_component(std::move(mujocoRos2System), control_hardware_info[i]);
		RCLCPP_DEBUG(this->dataPtr_->node_->get_logger(), "Setting state to active");
		rclcpp_lifecycle::State state(
			lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
			hardware_interface::lifecycle_state_names::ACTIVE);
		resource_manager_->set_component_state(control_hardware_info[i].name, state);
	}
	
	return true;
}

void MujocoRos2ControlPlugin::reset()
{
	RCLCPP_INFO_STREAM(get_my_logger(), "reset");
}

} // namespace mujoco_ros2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control::MujocoRos2ControlPlugin, mujoco_ros2::MujocoPlugin)
