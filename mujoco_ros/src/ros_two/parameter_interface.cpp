#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros/render_backend.hpp>
#include <mujoco_ros/logging.hpp>

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/ros_two/plugin_utils.hpp>

#include <mujoco/mujoco.h>
#include <mujoco_ros/array_safety.h>

#include <rosgraph_msgs/msg/clock.hpp>

namespace mju = ::mujoco::sample_util;

namespace mujoco_ros {

void MujocoEnv::FetchConfiguration()
{
	RCLCPP_DEBUG(this->get_logger(), "Fetching configuration");

	std::vector<rclcpp::Parameter> params;
	if (!this->has_parameter("eval_mode")) {
		params.emplace_back(rclcpp::Parameter("eval_mode", false));
	}
	if (!this->has_parameter("no_render")) {
		params.emplace_back(rclcpp::Parameter("no_render", false));
	}
	if (!this->has_parameter("render_offscreen")) {
		params.emplace_back(rclcpp::Parameter("render_offscreen", false));
	}
	if (!this->has_parameter("headless")) {
		params.emplace_back(rclcpp::Parameter("headless", false));
	}
	if (!this->has_parameter("no_x")) {
		params.emplace_back(rclcpp::Parameter("no_x", rclcpp::PARAMETER_BOOL));
	}
	if (!this->has_parameter("unpause")) {
		params.emplace_back(rclcpp::Parameter("unpause", true));
	}
	if (!this->has_parameter("num_steps")) {
		params.emplace_back(rclcpp::Parameter("num_steps", -1));
	}
	if (!this->has_parameter("modelfile")) {
		params.emplace_back(rclcpp::Parameter("modelfile", ""));
	}
	if (!this->has_parameter("realtime")) {
		params.emplace_back(rclcpp::Parameter("realtime", 1.0));
	}
	if (!this->has_parameter(plugin_utils::MUJOCO_PLUGIN_PARAM_NAME + ".names")) {
		params.emplace_back(
		    rclcpp::Parameter(plugin_utils::MUJOCO_PLUGIN_PARAM_NAME + ".names", std::vector<std::string>{}));
	}
	if (!this->has_parameter("wait_for_xml")) {
		params.emplace_back(rclcpp::Parameter("wait_for_xml", false));
	}
	if (!this->has_parameter("mujoco_xml")) {
		params.emplace_back(rclcpp::Parameter("mujoco_xml", rclcpp::PARAMETER_STRING));
	}

	rclcpp::Parameter eval_mode_param = this->get_parameter("eval_mode");
	settings_.eval_mode               = eval_mode_param.as_bool();

	rclcpp::Parameter use_sim_time_param;
	if (!this->get_parameter("use_sim_time", use_sim_time_param)) {
		RCLCPP_FATAL(this->get_logger(),
		             "/use_sim_time ROS param is unset. This node requires you to explicitly set it to true "
		             "or false. Also Make sure it is set before starting any node, "
		             "otherwise nodes might behave unexpectedly.");
		throw std::runtime_error("/use_sim_time ROS param is unset.");
	}

	rclcpp::Parameter render_offscreen_param = this->get_parameter("render_offscreen");
	rclcpp::Parameter headless_param         = this->get_parameter("headless");
	rclcpp::Parameter unpause_param          = this->get_parameter("unpause");
	rclcpp::Parameter num_steps_param        = this->get_parameter("num_steps");

	rclcpp::Parameter no_render_param = this->get_parameter("no_render");
	rclcpp::Parameter no_x_param;
	if (this->get_parameter("no_x", no_x_param)) {
		RCLCPP_WARN(this->get_logger(), "The 'no_x' parameter is deprecated. Use 'no_render' instead.");
		no_render_param = no_x_param;
	}

	if (no_render_param.as_bool()) {
		RCLCPP_INFO(this->get_logger(), "no_render is set. Disabling rendering and setting headless to true");
		this->set_parameters({ rclcpp::Parameter("headless", true), rclcpp::Parameter("render_offscreen", false) });
	}

	settings_.render_offscreen = render_offscreen_param.as_bool();
	settings_.headless         = headless_param.as_bool();
	settings_.run              = unpause_param.as_bool();
	num_steps_until_exit_      = num_steps_param.as_int();

	std::string filename = "";

	this->get_parameter("modelfile", filename);

	rclcpp::Parameter wait_for_xml_param = this->get_parameter("wait_for_xml");
	if (wait_for_xml_param.as_bool()) {
		RCLCPP_INFO(this->get_logger(), "Waiting for mujoco_xml content parameter...");
		bool wait_for_xml = true;
		std::string xml_content;
		rclcpp::Parameter xml_content_param;
		while (wait_for_xml) {
			if (this->get_parameter("mujoco_xml", xml_content_param)) {
				RCLCPP_INFO(this->get_logger(), "Got xml content from ros parameter");
				if (!xml_content_param.get_value<std::string>().empty()) {
					filename = "rosparam_content";
				} else {
					RCLCPP_WARN(this->get_logger(), "Empty xml content received from ros parameter");
				}
				wait_for_xml = false;
			}
		}
	}

	if (!filename.empty()) {
		RCLCPP_INFO_STREAM(this->get_logger(), "Using modelfile " << filename);
		mju::strcpy_arr(queued_filename_, filename.c_str());
		settings_.load_request = 2;
	} else {
		RCLCPP_WARN(this->get_logger(), "No modelfile was provided, launching empty simulation!");
	}

	// TODO: after python bindings merge
	// nh_->param<int>("num_mj_threads", settings_.num_threads, 4);
}

void MujocoEnv::InitTFBroadcasting()
{
	static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
	tf_buffer_          = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_buffer_->setUsingDedicatedThread(true);
	tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

template <typename T>
void get_maybe_undeclared_param(rclcpp::Node *node, const std::string &param_name, T &param, const T default_val)
{
	if (!node->has_parameter(param_name)) {
		node->declare_parameter(param_name, default_val);
		param = default_val;
		return;
	}
	// param = node->get_parameter(param_name).get_value();
}

void MujocoEnv::GetCameraConfiguration(const std::string &cam_name, rendering::StreamType &stream_type,
                                       float &pub_frequency, bool &use_segid, int &width, int &height,
                                       std::string &base_topic, std::string &rgb_topic, std::string &depth_topic,
                                       std::string &segment_topic)
{
	int stream_type_int;
	get_maybe_undeclared_param(this, cam_name + ".stream_type", stream_type_int,
	                           static_cast<int>(rendering::kDEFAULT_CAM_STREAM_TYPE));
	stream_type = rendering::StreamType(stream_type_int);
	get_maybe_undeclared_param(this, cam_name + ".frequency", pub_frequency, rendering::kDEFAULT_CAM_PUB_FREQ);
	get_maybe_undeclared_param(this, cam_name + ".use_segid", use_segid, rendering::kDEFAULT_CAM_USE_SEGID);
	get_maybe_undeclared_param(this, cam_name + ".width", width, rendering::kDEFAULT_CAM_WIDTH);
	get_maybe_undeclared_param(this, cam_name + ".height", height, rendering::kDEFAULT_CAM_HEIGHT);
	get_maybe_undeclared_param(this, cam_name + ".topic", base_topic, "cameras/" + cam_name);
	get_maybe_undeclared_param(this, cam_name + ".name_rgb", rgb_topic, std::string(rendering::kDEFAULT_CAM_RGB_TOPIC));
	get_maybe_undeclared_param(this, cam_name + ".name_depth", depth_topic,
	                           std::string(rendering::kDEFAULT_CAM_DEPTH_TOPIC));
	get_maybe_undeclared_param(this, cam_name + ".name_segment", segment_topic,
	                           std::string(rendering::kDEFAULT_CAM_SEGMENT_TOPIC));
}

void MujocoEnv::GetInitialJointPositions(std::map<std::string, std::vector<double>> & /*joint_pos_map*/)
{
	MJR_WARN("Initial joint positions NYI in ROS 2");
	// std::map<std::string, std::string> joint_map;
	// nh_->getParam("initial_joint_positions/joint_map", joint_map);

	// // This check only assures that there aren't single axis joint values that are non-strings.
	// // One ill-defined value among correct parameters can't be detected.
	// if (nh_->hasParam("initial_joint_positions/joint_map") && joint_map.empty()) {
	// 	MJR_WARN("Initial joint positions not recognized by rosparam server. Check your config, "
	// 	         "especially values for single axis joints should explicitly provided as string!");
	// 	return;
	// }

	// for (auto const &[name, str_values] : joint_map) {
	// 	MJR_DEBUG_STREAM("fetched jointpos values of joint " << name << ": " << str_values);

	// 	std::vector<double> axis_vals;
	// 	axis_vals.reserve(7);

	// 	std::stringstream stream_values(str_values);
	// 	std::string value;
	// 	while (std::getline(stream_values, value, ' ')) {
	// 		axis_vals.push_back(std::stod(value));
	// 	}
	// 	axis_vals.shrink_to_fit();
	// 	joint_pos_map[name] = axis_vals;
	// }
}

void MujocoEnv::GetInitialJointVelocities(std::map<std::string, std::vector<double>> & /*joint_vel_map*/)
{
	MJR_WARN("Initial joint velocities NYI in ROS 2");
	// std::map<std::string, std::string> joint_map;
	// nh_->getParam("initial_joint_velocities/joint_map", joint_map);

	// // This check only assures that there aren't single axis joint values that are non-strings.
	// // One ill-defined value among correct parameters can't be detected.
	// if (nh_->hasParam("initial_joint_velocities/joint_map") && joint_map.empty()) {
	// 	MJR_WARN("Initial joint velocities not recognized by rosparam server. Check your config, "
	// 	         "especially values for single axis joints should explicitly provided as string!");
	// 	return;
	// }

	// for (auto const &[name, str_values] : joint_map) {
	// 	MJR_DEBUG_STREAM("fetched jointvel values of joint " << name << ": " << str_values);

	// 	std::vector<double> axis_vals;
	// 	axis_vals.reserve(7);

	// 	std::stringstream stream_values(str_values);
	// 	std::string value;
	// 	while (std::getline(stream_values, value, ' ')) {
	// 		axis_vals.push_back(std::stod(value));
	// 	}
	// 	axis_vals.shrink_to_fit();
	// 	joint_vel_map[name] = axis_vals;
	// }
}

// TODO:
// setupServices
// callbacks

} // namespace mujoco_ros
