/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: David P. Leins*/

#include <mujoco_ros2_base/plugin_utils.h>

namespace mujoco_ros2::plugin_utils {

bool parsePlugins(const ros::NodeHandle *nh, XmlRpc::XmlRpcValue &plugin_config_rpc)
{
	std::string param_path;
	if (nh->searchParam(MUJOCO_PLUGIN_PARAM_NAME, param_path) ||
	    ros::param::search(MUJOCO_PLUGIN_PARAM_NAME, param_path)) {
		// RCLCPP_DEBUG_STREAM(rclcpp::get_logger("mujoco_ros2_plugin_loader",
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("mujoco_ros2_plugin_loader"),
		                    "Found MujocoPlugins param under " << param_path);
	} else {
		RCLCPP_INFO_STREAM(rclcpp::get_logger("mujoco_ros2_plugin_loader"),
		                   "No plugins to load listed in parameter server!");
		return false;
	}

	ROS_DEBUG_NAMED("mujoco_ros_plugin_loader", "Initializing plugin loader ... ");

	nh->getParam(param_path, plugin_config_rpc);

	if (plugin_config_rpc.getType() != XmlRpc::XmlRpcValue::TypeArray) {
		RCLCPP_ERROR_STREAM(rclcpp::get_logger("mujoco_ros2_plugin_loader"),
		                    "Error while parsing MujocoPlugins rosparam: wrong type.");
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("mujoco_ros2_plugin_loader"),
		                    "MujocoPlugins rosparam should be of type '"
		                        << XmlRpc::XmlRpcValue::TypeArray << "', but got type '" << plugin_config_rpc.getType()
		                        << "' (yaml array)!");
		return false;
	}
	return true;
}

void registerPlugins(const std::string &nh_namespace, const XmlRpc::XmlRpcValue &config_rpc,
                     std::vector<MujocoPluginPtr> &plugins, MujocoEnv *env)
{
	for (int8_t i = 0; i < config_rpc.size(); i++) {
		if (config_rpc[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("mujoco_ros2_plugin_loader"),
			                    "Error while parsing MujocoPlugins rosparam: wrong type.");
			RCLCPP_DEBUG_STREAM(rclcpp::get_logger("mujoco_ros2_plugin_loader"),
			                    "Children of 'MujocoPlugins' should be of type '"
			                        << XmlRpc::XmlRpcValue::TypeStruct << "', but got type '" << config_rpc.getType()
			                        << "'. Skipping " << config_rpc[i]);
			continue;
		}
		// TODO: handle failed registration somehow?
		registerPlugin(nh_namespace, config_rpc[i], plugins, env);
	}
}

bool registerPlugin(const std::string &nh_namespace, const XmlRpc::XmlRpcValue &config,
                    std::vector<MujocoPluginPtr> &plugins, MujocoEnv *env)
{
	assert(config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
	std::string type;

	if (!config.hasMember("type")) {
		RCLCPP_ERROR_STREAM(rclcpp::get_logger("mujoco_ros2_plugin_loader"),
		                    "Error while parsing MujocoPlugins rosparam: Every listed plugin "
		                    "should provide a 'type' member!");
		return false;
	}
	type = static_cast<std::string>(config["type"]);

	RCLCPP_DEBUG_STREAM(rclcpp::get_logger("mujoco_ros2_plugin_loader"), "Registering plugin of type " << type);

	try {
		MujocoPlugin *mjplugin_ptr = plugin_loader_ptr_->createUnmanagedInstance(type);
		mjplugin_ptr->init(config, nh_namespace, env);
		plugins.emplace_back(std::unique_ptr<MujocoPlugin>(mjplugin_ptr));
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("mujoco_ros2_plugin_loader"),
		                    "Added " << type << " to the list of loaded plugins in namespace '" << nh_namespace
		                             << "'. List now contains " << plugins.size() << " plugin(s)");
	} catch (const pluginlib::PluginlibException &ex) {
		RCLCPP_ERROR_STREAM(rclcpp::get_logger("mujoco_ros2_plugin_loader"),
		                    "The plugin failed to load (for namespace " << nh_namespace << " ): " << ex.what());
		return false;
	}

	return true;
}

void initPluginLoader()
{
	// NOLINTBEGIN(clang-analyzer-optin.cplusplus.VirtualCall)
	plugin_loader_ptr_ =
	    std::make_unique<pluginlib::ClassLoader<mujoco_ros2::MujocoPlugin>>("mujoco_ros2", "mujoco_ros2::MujocoPlugin");
	// NOLINTEND(clang-analyzer-optin.cplusplus.VirtualCall)
}

void unloadPluginloader()
{
	plugin_loader_ptr_.reset();
}

} // namespace mujoco_ros2::plugin_utils
