#include "mujoco_ros2_control/mujoco_ros2_control.hpp"
#include <set>

namespace mujoco_ros2_control {

std::string concatenateNamespace(const std::string ns1, const std::string ns2){
	if(ns1.back() == '/'){
		return ns1 + ns2;
	}
	else{
		return ns1 + '/' + ns2;
	}

}
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

void MujocoRos2ControlPlugin::ControlCallback(const mjModel* /* model */, mjData* data)
{
	
	rclcpp::Time sim_time_mj = rclcpp::Time(static_cast<int64_t>(data->time*1e9), RCL_STEADY_TIME);
	rclcpp::Duration sim_period = sim_time_mj - this->dataPtr_->last_update_sim_time_mj_;
	this->dataPtr_->controller_manager_->write(sim_time_mj, sim_period);
};
void MujocoRos2ControlPlugin::PassiveCallback(const mjModel* /* model */, mjData* data)
{
	rclcpp::Time sim_time_mj = rclcpp::Time(static_cast<int64_t>(data->time*1e9), RCL_STEADY_TIME);
	rclcpp::Duration sim_period = sim_time_mj - this->dataPtr_->last_update_sim_time_mj_;
	this->dataPtr_->last_update_sim_time_mj_ = sim_time_mj;
	this->dataPtr_->controller_manager_->read(sim_time_mj, sim_period);
	this->dataPtr_->controller_manager_->update(sim_time_mj, sim_period);
};
void MujocoRos2ControlPlugin::RenderCallback(const mjModel* /* model */, mjData* /* data */, mjvScene* /* scene */)
{
};
void MujocoRos2ControlPlugin::LastStageCallback(const mjModel* /* model */, mjData* /* data */)
{
};
void MujocoRos2ControlPlugin::OnGeomChanged(const mjModel* /* model */, mjData* /* data */, const int /* geom_id */)
{
};

bool MujocoRos2ControlPlugin::Load(const mjModel *model, mjData *data)
{
	dataPtr_ = std::make_unique<MujocoRos2ControlPluginPrivate>();
	
	RCLCPP_INFO_STREAM(get_my_logger(), "loading with given model and data");

	// get the name of the robot_state_publisher node
	std::string robot_description_node;
	if (env_ptr_->has_parameter(MUJOCO_PLUGIN_PARAM_NAME + "." + plugin_name + ".params.robot_description_node")) {
		robot_description_node = env_ptr_->get_parameter(MUJOCO_PLUGIN_PARAM_NAME + "." + plugin_name + ".params.robot_description_node").as_string();
	}

	if (robot_description_node.empty()) {
		RCLCPP_INFO_STREAM(get_my_logger(), "No string provided for robot_description_node." 
											"Proceeding with default value " << this->dataPtr_->robot_description_node_);
	}
	else{
		this->dataPtr_->robot_description_node_ = robot_description_node;
		RCLCPP_INFO_STREAM(get_my_logger(), "robot_description_node name is " << this->dataPtr_->robot_description_node_);
	}


	// get the name of the srv from the node above that holds the URDF string
	// get the name of the robot_state_publisher node
	std::string robot_description;
	if (env_ptr_->has_parameter(MUJOCO_PLUGIN_PARAM_NAME + "." + plugin_name + ".params.robot_description")) {
		robot_description = env_ptr_->get_parameter(MUJOCO_PLUGIN_PARAM_NAME + "." + plugin_name + ".params.robot_description").as_string();
	}

	if (robot_description.empty()) {
		RCLCPP_INFO_STREAM(get_my_logger(), "No string provided for robot_description service." 
											"Proceeding with default value " << this->dataPtr_->robot_description_);
	}
	else{
		this->dataPtr_->robot_description_ = robot_description;
		RCLCPP_INFO_STREAM(get_my_logger(), "robot_description service name is " << this->dataPtr_->robot_description_);
	}
	// todo: Make the logic for passing additional ros-args when initializing, like L292~ in ign_ros2_control_plugin.cpp

	// Construct the fully qualified robot_description_node's name.
	// The namespace will also be used for the nodes of this.
	std::string ns = "/";
	if (env_ptr_->has_parameter(MUJOCO_PLUGIN_PARAM_NAME + "." + plugin_name + ".params.namespace")) {
		ns = env_ptr_->get_parameter(MUJOCO_PLUGIN_PARAM_NAME + "." + plugin_name + ".params.namespace").as_string();

		// prevent exception: namespace must be absolute, it must lead with a '/'
		if (ns.empty() || ns[0] != '/') {
			ns = '/' + ns;
		}
		if (ns.length() > 1) {
			this->dataPtr_->robot_description_node_ = ns + "/" + this->dataPtr_->robot_description_node_;
		}
		else{
			this->dataPtr_->robot_description_node_ = ns + this->dataPtr_->robot_description_node_;
		}
	}
	RCLCPP_INFO(get_my_logger(), "robot_description_node fully qualified name: %s", this->dataPtr_->robot_description_node_.c_str());

	// Create a default context, if not already
	if (!rclcpp::ok()) {
  		std::vector<const char *> _argv;
		rclcpp::init(static_cast<int>(_argv.size()), _argv.data()); // todo: make the logic for passing additional ros-args
	}
	std::string node_name = "mujoco_ros2_control";

	this->dataPtr_->node_ = std::make_shared<rclcpp::Node>(node_name, ns);

	RCLCPP_INFO_STREAM(get_my_logger(), "Fully qualified mujoco_ros2_control node name: " << this->dataPtr_->node_->get_fully_qualified_name());

	// executor creation
	this->dataPtr_->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
	this->dataPtr_->executor_->add_node(this->dataPtr_->node_);
	auto spin = [this]()
		{
		this->dataPtr_->executor_->spin();
		};
	this->dataPtr_->thread_executor_spin_ = std::thread(spin);

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
			model,
			data,
			this->dataPtr_->update_rate))
		{
			RCLCPP_FATAL(
				this->dataPtr_->node_->get_logger(), "Could not initialize robot simulation interface");
				return false;	
		}
		RCLCPP_DEBUG(
			this->dataPtr_->node_->get_logger(), "Initialized robot simulation interface %s!",
			robot_hw_sim_type_str_.c_str());
		
		
		RCLCPP_DEBUG_STREAM(this->dataPtr_->node_->get_logger(), "resource-manager system comp size: " << resource_manager_->system_components_size());
		resource_manager_->import_component(std::move(mujocoRos2System), control_hardware_info[i]);
		
		RCLCPP_DEBUG_STREAM(this->dataPtr_->node_->get_logger(), "resource-manager system comp size: " << resource_manager_->system_components_size());
		RCLCPP_DEBUG(this->dataPtr_->node_->get_logger(), "Setting state of %s to active", control_hardware_info[i].name.c_str());
		rclcpp_lifecycle::State state(
			lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
			hardware_interface::lifecycle_state_names::ACTIVE);
		resource_manager_->set_component_state(control_hardware_info[i].name, state);
	}

	// Get controller manager node name
	std::string controllerManagerNodeName{"controller_manager"};

	// Create the controller manager
	RCLCPP_INFO(this->dataPtr_->node_->get_logger(), "Loading controller_manager");
		// This part sometimes breaks with error munmap_chunk(): invalid pointer
	this->dataPtr_->controller_manager_.reset(
		new controller_manager::ControllerManager(
		std::move(resource_manager_),
		this->dataPtr_->executor_,
		controllerManagerNodeName,
		ns));
	this->dataPtr_->executor_->add_node(this->dataPtr_->controller_manager_);

	if (!this->dataPtr_->controller_manager_->has_parameter("update_rate")) {
		RCLCPP_ERROR_STREAM(
		this->dataPtr_->node_->get_logger(),
		"controller manager doesn't have an update_rate parameter");
		return false;
	}

	this->dataPtr_->update_rate =
		this->dataPtr_->controller_manager_->get_parameter("update_rate").as_int();
	this->dataPtr_->control_period_ = rclcpp::Duration(
		std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::duration<double>(1.0 / static_cast<double>(this->dataPtr_->update_rate))));

	// Force setting of use_sim_time parameter
	this->dataPtr_->controller_manager_->set_parameter(
		rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));
		
	return true;
}

void MujocoRos2ControlPlugin::Reset()
{
	RCLCPP_INFO_STREAM(get_my_logger(), "reset");
}

} // namespace mujoco_ros2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control::MujocoRos2ControlPlugin, mujoco_ros::MujocoPlugin)
