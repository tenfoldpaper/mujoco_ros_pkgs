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

/* Authors: David P. Leins */

#include <gtest/gtest.h>

// Ignore static variables unused in this compilation unit
// TODO(dleins): Remove this after object oriented refactoring
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#include <mujoco_ros/mujoco_sim.h>
#pragma GCC diagnostic pop

#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/common_types.h>
#include <mujoco_ros/util.h>

#include <mujoco_ros_msgs/SetPause.h>
#include <mujoco_ros_msgs/StepAction.h>
#include <mujoco_ros_msgs/StepGoal.h>
#include <mujoco_ros_msgs/SetBodyState.h>
#include <mujoco_ros_msgs/GetBodyState.h>
#include <mujoco_ros_msgs/SetGeomProperties.h>
#include <mujoco_ros_msgs/GeomType.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "mujoco_ros_test_node");
	return RUN_ALL_TESTS();
}

namespace unit_testing {
class MujocoRosCoreFixture : public ::testing::Test
{
protected:
	std::shared_ptr<ros::NodeHandle> nh;
	mujoco_ros::mjModelPtr getModel(mujoco_ros::MujocoEnvPtr env) { return env->model_; }
	mujoco_ros::mjDataPtr getData(mujoco_ros::MujocoEnvPtr env) { return env->data_; }

	virtual void SetUp()
	{
		nh.reset(new ros::NodeHandle("~"));
		nh->setParam("unpause", true);
		nh->setParam("no_x", true);
		nh->setParam("use_sim_time", true);
	}

	virtual void TearDown() {}
};

class MujocoRosBaseFixture : public ::testing::Test
{
protected:
	std::shared_ptr<ros::NodeHandle> nh;
	mujoco_ros::MujocoEnvPtr env;
	mujoco_ros::mjDataPtr d;
	mujoco_ros::mjModelPtr m;
	std::unique_ptr<std::thread> mj_thread;

	virtual void SetUp()
	{
		nh.reset(new ros::NodeHandle("~"));
		nh->setParam("unpause", false);
		nh->setParam("no_x", true);
		nh->setParam("use_sim_time", true);

		std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/pendulum_world.xml";

		mj_thread = std::unique_ptr<std::thread>(new std::thread(mujoco_ros::init, xml_path, ""));
		while (mujoco_ros::detail::settings_.loadrequest.load() == 0) { // wait for request to be made
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		while (mujoco_ros::detail::settings_.loadrequest.load() > 0) { // wait for model to be loaded
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		env = mujoco_ros::detail::main_env_;
		d   = env->data_;
		m   = env->model_;
	}

	virtual void TearDown() {}
};

void compare_qpos(mujoco_ros::mjDataPtr d, int qpos_adr_int, std::string joint_name, const std::vector<double> &values,
                  const std::vector<double> &tolerances = {})
{
	uint qpos_adr = static_cast<uint>(qpos_adr_int);
	if (tolerances.size() == 0) {
		for (size_t i = 0; i < values.size(); i++) {
			EXPECT_EQ(d->qpos[qpos_adr + i], values[i]) << "qpos of joint '" << joint_name << "' at index " << i << " is "
			                                            << d->qpos[qpos_adr + i] << " instead of " << values[i] << "!";
		}
	} else {
		for (size_t i = 0; i < values.size(); i++) {
			EXPECT_NEAR(d->qpos[qpos_adr + i], values[i], tolerances[i])
			    << "qpos of joint '" << joint_name << "' at index " << i << " is " << d->qpos[qpos_adr + i]
			    << " instead of " << values[i] << " (tolerance: " << tolerances[i] << ")!";
		}
	}
}

void compare_qvel(mujoco_ros::mjDataPtr d, int dof_adr_int, std::string joint_name, std::vector<double> values)
{
	uint dof_adr = static_cast<uint>(dof_adr_int);
	for (size_t i = 0; i < values.size(); i++) {
		EXPECT_EQ(d->qvel[dof_adr + i], values[i]) << "qvel of joint '" << joint_name << "' at index " << i << " is "
		                                           << d->qvel[dof_adr + i] << " instead of " << values[i] << "!";
	}
}

TEST_F(MujocoRosCoreFixture, init_with_model)
{
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	std::thread mjThread(mujoco_ros::init, xml_path, "");

	EXPECT_GE(ros::Time::now().toNSec(), 0.0) << "Time should be running!";
	std::this_thread::sleep_for(std::chrono::seconds(1));
	mujoco_ros::requestExternalShutdown();
	EXPECT_GE(ros::Time::now().toSec(), 0.001 * 100) << "Time should have kept running";
	mjThread.join();
}

TEST_F(MujocoRosCoreFixture, pause_unpause)
{
	nh->setParam("unpause", false);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	std::thread mjThread(mujoco_ros::init, xml_path, "");

	std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Give the sim thread enough time to reset ROS time to 0
	double time = ros::Time::now().toSec();
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	EXPECT_EQ(ros::Time::now().toSec(), time) << "Time should not be running!";

	mujoco_ros_msgs::SetPause srv;
	srv.request.paused = false;

	mujoco_ros::detail::setPauseCB(srv.request, srv.response);

	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	EXPECT_GT(ros::Time::now().toSec(), time) << "Time should have been moving forward!";

	srv.request.paused = true;
	mujoco_ros::detail::setPauseCB(srv.request, srv.response);

	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	time = ros::Time::now().toSec();
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	EXPECT_EQ(ros::Time::now().toSec(), time) << "Time should not have moved forward!";

	mujoco_ros::requestExternalShutdown();
	mjThread.join();
}

TEST_F(MujocoRosCoreFixture, num_steps)
{
	nh->setParam("num_steps", 100);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";

	std::thread mjThread(mujoco_ros::init, xml_path, "");
	mjThread.join();

	EXPECT_NEAR(ros::Time::now().toSec(), 0.001 * 100, 0.0001) << "Time should have stopped after 100 steps";
	nh->deleteParam("num_steps");
}

TEST_F(MujocoRosBaseFixture, default_initial_joint_states)
{
	nh->setParam("unpause", false);

	int id_balljoint, id1, id2, id_free;
	id_balljoint = mujoco_ros::util::jointName2id(m.get(), "balljoint");
	id1          = mujoco_ros::util::jointName2id(m.get(), "joint1");
	id2          = mujoco_ros::util::jointName2id(m.get(), "joint2");
	id_free      = mujoco_ros::util::jointName2id(m.get(), "ball_freejoint");

	EXPECT_NE(id_balljoint, -1) << "'balljoint' should be found as joint in model!";
	EXPECT_NE(id1, -1) << "'joint1' should be found as joint in model!";
	EXPECT_NE(id2, -1) << "'joint2' should be found as joint in model!";
	EXPECT_NE(id_free, -1) << "'ball_freejoint' should be found as joint in model!";

	compare_qpos(d, m->jnt_qposadr[id_balljoint], "balljoint", { 1.0, 0.0, 0.0, 0.0 });
	compare_qpos(d, m->jnt_qposadr[id1], "joint1", { 0.0 });
	compare_qpos(d, m->jnt_qposadr[id2], "joint2", { 0.0 });
	compare_qpos(d, m->jnt_qposadr[id_free], "ball_freejoint", { 1.0, 0.0, 0.06, 1.0, 0.0, 0.0, 0.0 });

	compare_qvel(d, m->jnt_dofadr[id_balljoint], "balljoint", { 0.0, 0.0, 0.0 });
	compare_qvel(d, m->jnt_dofadr[id1], "joint1", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[id2], "joint2", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[id_free], "ball_freejoint", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });

	mujoco_ros::requestExternalShutdown();
	mj_thread->join();
}

TEST_F(MujocoRosCoreFixture, custom_initial_joint_states_on_reset)
{
	nh->setParam("unpause", false);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/pendulum_world.xml";
	std::map<std::string, std::string> pos_map, vel_map;

	pos_map.insert({ "balljoint", "0 0.707 0 0.707" });
	pos_map.insert({ "joint1", "-1.57" });
	pos_map.insert({ "joint2", "-0.66" });
	pos_map.insert({ "ball_freejoint", "2.0 1.0 1.06 0.0 0.707 0.0 0.707" });

	vel_map.insert({ "balljoint", "5 5 10" });
	vel_map.insert({ "joint2", "1.05" });
	vel_map.insert({ "ball_freejoint", "1.0 2.0 3.0 10 20 30" });

	std::thread mjThread(mujoco_ros::init, xml_path, "");
	while (mujoco_ros::detail::settings_.loadrequest.load() == 0) { // wait for request to be made
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	while (mujoco_ros::detail::settings_.loadrequest.load() > 0) { // wait for model to be loaded
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	mujoco_ros::MujocoEnvPtr env = mujoco_ros::detail::main_env_;
	mujoco_ros::mjDataPtr d      = env->data_;
	mujoco_ros::mjModelPtr m     = env->model_;

	int id_balljoint, id1, id2, id_free;

	id_balljoint = mujoco_ros::jointName2id(m.get(), "balljoint");
	id1          = mujoco_ros::jointName2id(m.get(), "joint1");
	id2          = mujoco_ros::jointName2id(m.get(), "joint2");
	id_free      = mujoco_ros::jointName2id(m.get(), "ball_freejoint");

	EXPECT_NE(id_balljoint, -1) << "'balljoint' should be found as joint in model!";
	EXPECT_NE(id1, -1) << "'joint1' should be found as joint in model!";
	EXPECT_NE(id2, -1) << "'joint2' should be found as joint in model!";
	EXPECT_NE(id_free, -1) << "'ball_freejoint' should be found as joint in model!";

	compare_qpos(d, m->jnt_qposadr[id_balljoint], "balljoint", { 1.0, 0.0, 0.0, 0.0 });
	compare_qpos(d, m->jnt_qposadr[id1], "joint1", { 0.0 });
	compare_qpos(d, m->jnt_qposadr[id2], "joint2", { 0.0 });
	compare_qpos(d, m->jnt_qposadr[id_free], "ball_freejoint", { 1.0, 0.0, 0.06, 1.0, 0.0, 0.0, 0.0 });

	compare_qvel(d, m->jnt_dofadr[id_balljoint], "balljoint", { 0.0, 0.0, 0.0 });
	compare_qvel(d, m->jnt_dofadr[id1], "joint1", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[id2], "joint2", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[id_free], "ball_freejoint", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });

	nh->setParam("initial_joint_positions/joint_map", pos_map);
	nh->setParam("initial_joint_velocities/joint_map", vel_map);

	std_srvs::Empty srv;
	mujoco_ros::detail::resetCB(srv.request, srv.response);

	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	while (mujoco_ros::detail::settings_.resetrequest.load() > 0) { // wait for model to be loaded
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	compare_qpos(d, m->jnt_qposadr[id_balljoint], "balljoint", { 0.0, 0.707, 0.0, 0.707 }, { 0.0, 9e-4, 0.0, 9e-4 });
	compare_qpos(d, m->jnt_qposadr[id1], "joint1", { -1.57 });
	compare_qpos(d, m->jnt_qposadr[id2], "joint2", { -0.66 });
	compare_qpos(d, m->jnt_qposadr[id_free], "ball_freejoint", { 2.0, 1.0, 1.06, 0.0, 0.707, 0.0, 0.707 },
	             { 0.0, 0.0, 0.0, 0.0, 9e-4, 0.0, 9e-4 });

	compare_qvel(d, m->jnt_dofadr[id_balljoint], "balljoint", { 5.0, 5.0, 10.0 });
	compare_qvel(d, m->jnt_dofadr[id1], "joint1", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[id2], "joint2", { 1.05 });
	compare_qvel(d, m->jnt_dofadr[id_free], "ball_freejoint", { 1.0, 2.0, 3.0, 10.0, 20.0, 30.0 });

	mujoco_ros::requestExternalShutdown();
	mjThread.join();

	nh->deleteParam("initial_joint_positions/joint_map");
	nh->deleteParam("initial_joint_velocities/joint_map");
}

TEST_F(MujocoRosCoreFixture, custom_initial_joint_states)
{
	nh->setParam("unpause", false);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/pendulum_world.xml";
	std::map<std::string, std::string> pos_map, vel_map;

	pos_map.insert({ "balljoint", "0 0.707 0 0.707" });
	pos_map.insert({ "joint1", "-1.57" });
	pos_map.insert({ "joint2", "-0.66" });
	pos_map.insert({ "ball_freejoint", "2.0 1.0 1.06 0.0 0.707 0.0 0.707" });

	vel_map.insert({ "balljoint", "5 5 10" });
	vel_map.insert({ "joint2", "1.05" });
	vel_map.insert({ "ball_freejoint", "1.0 2.0 3.0 10 20 30" });

	nh->setParam("initial_joint_positions/joint_map", pos_map);
	nh->setParam("initial_joint_velocities/joint_map", vel_map);

	std::thread mjThread(mujoco_ros::init, xml_path, "");
	while (mujoco_ros::detail::settings_.loadrequest.load() == 0) { // wait for request to be made
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	while (mujoco_ros::detail::settings_.loadrequest.load() > 0) { // wait for model to be loaded
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	mujoco_ros::MujocoEnvPtr env = mujoco_ros::detail::main_env_;
	mujoco_ros::mjDataPtr d      = env->data_;
	mujoco_ros::mjModelPtr m     = env->model_;

	int id_balljoint, id1, id2, id_free;

	id_balljoint = mujoco_ros::jointName2id(m.get(), "balljoint");
	id1          = mujoco_ros::jointName2id(m.get(), "joint1");
	id2          = mujoco_ros::jointName2id(m.get(), "joint2");
	id_free      = mujoco_ros::jointName2id(m.get(), "ball_freejoint");

	EXPECT_NE(id_balljoint, -1) << "'balljoint' should be found as joint in model!";
	EXPECT_NE(id1, -1) << "'joint1' should be found as joint in model!";
	EXPECT_NE(id2, -1) << "'joint2' should be found as joint in model!";
	EXPECT_NE(id_free, -1) << "'ball_freejoint' should be found as joint in model!";

	compare_qpos(d, m->jnt_qposadr[id_balljoint], "balljoint", { 0.0, 0.707, 0.0, 0.707 }, { 0.0, 9e-4, 0.0, 9e-4 });
	compare_qpos(d, m->jnt_qposadr[id1], "joint1", { -1.57 });
	compare_qpos(d, m->jnt_qposadr[id2], "joint2", { -0.66 });
	compare_qpos(d, m->jnt_qposadr[id_free], "ball_freejoint", { 2.0, 1.0, 1.06, 0.0, 0.707, 0.0, 0.707 },
	             { 0.0, 0.0, 0.0, 0.0, 9e-4, 0.0, 9e-4 });

	compare_qvel(d, m->jnt_dofadr[id_balljoint], "balljoint", { 5.0, 5.0, 10.0 });
	compare_qvel(d, m->jnt_dofadr[id1], "joint1", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[id2], "joint2", { 1.05 });
	compare_qvel(d, m->jnt_dofadr[id_free], "ball_freejoint", { 1.0, 2.0, 3.0, 10.0, 20.0, 30.0 });

	mujoco_ros::requestExternalShutdown();
	mjThread.join();

	nh->deleteParam("initial_joint_positions/joint_map");
	nh->deleteParam("initial_joint_velocities/joint_map");
}

TEST_F(MujocoRosBaseFixture, set_body_state)
{
	nh->setParam("unpause", false);

	int id_free;

	id_free = mujoco_ros::jointName2id(m.get(), "ball_freejoint");

	mujoco_ros_msgs::SetBodyState srv;

	// Invalid body_name
	srv.request.state.env_id = 0;
	srv.request.state.name   = "unknown";
	mujoco_ros::detail::setBodyStateCB(srv.request, srv.response);
	EXPECT_FALSE(srv.response.success);

	// Resolve body
	srv.request.state.name = "middle_link";
	mujoco_ros::detail::setBodyStateCB(srv.request, srv.response);
	EXPECT_TRUE(srv.response.success);

	// Resolve body from child geom
	srv.request.state.name = "EE";
	mujoco_ros::detail::setBodyStateCB(srv.request, srv.response);
	EXPECT_TRUE(srv.response.success);

	// Position change errors
	srv.request.set_pose = true;

	//   Not a freejoint
	mujoco_ros::detail::setBodyStateCB(srv.request, srv.response);
	EXPECT_FALSE(srv.response.success);

	//   No joint
	srv.request.state.name = "immovable";
	mujoco_ros::detail::setBodyStateCB(srv.request, srv.response);
	EXPECT_FALSE(srv.response.success);

	//   unknown frame_id
	srv.request.state.name                 = "ball";
	srv.request.state.pose.header.frame_id = "unknown";
	mujoco_ros::detail::setBodyStateCB(srv.request, srv.response);
	EXPECT_FALSE(srv.response.success);

	// Twist change errors
	srv.request.set_pose  = false;
	srv.request.set_twist = true;

	//   other frame_id than world
	srv.request.state.twist.header.frame_id = "not-world";
	mujoco_ros::detail::setBodyStateCB(srv.request, srv.response);
	EXPECT_FALSE(srv.response.success);

	// New twist and pose
	srv.request.set_pose                      = true;
	srv.request.state.pose.header.frame_id    = "world";
	srv.request.state.pose.pose.position.x    = 2;
	srv.request.state.pose.pose.position.y    = 2;
	srv.request.state.pose.pose.position.z    = 2;
	srv.request.state.pose.pose.orientation.x = 0.707;
	srv.request.state.pose.pose.orientation.y = 0.0;
	srv.request.state.pose.pose.orientation.z = 0.707;
	srv.request.state.pose.pose.orientation.w = 0.0;

	srv.request.state.twist.header.frame_id = "world";
	srv.request.state.twist.twist.linear.x  = 0.1;
	srv.request.state.twist.twist.linear.y  = 0.1;
	srv.request.state.twist.twist.linear.z  = -0.1;
	srv.request.state.twist.twist.angular.x = 0.1;
	srv.request.state.twist.twist.angular.y = 0;
	srv.request.state.twist.twist.angular.z = 0;
	mujoco_ros::detail::setBodyStateCB(srv.request, srv.response);
	EXPECT_TRUE(srv.response.success);

	compare_qpos(d, m->jnt_qposadr[id_free], "ball_freejoint", { 2.0, 2.0, 2.0, 0.0, 0.707, 0.0, 0.707 },
	             { 0.0, 0.0, 0.0, 0.0, 9e-4, 0.0, 9e-4 });
	compare_qvel(d, m->jnt_dofadr[id_free], "ball_freejoint", { 0.1, 0.1, -0.1, 0.1, 0.0, 0.0 });

	// New mass
	srv.request.set_pose   = false;
	srv.request.set_twist  = false;
	srv.request.set_mass   = true;
	srv.request.state.mass = 0.299f;
	EXPECT_NE(m->body_mass[mj_name2id(m.get(), mjOBJ_BODY, "body_ball")], srv.request.state.mass)
	    << "Mass already has the requested value!"; // Check that mass is different beforehand
	mujoco_ros::detail::setBodyStateCB(srv.request, srv.response);
	EXPECT_TRUE(srv.response.success);

	EXPECT_EQ(m->body_mass[mj_name2id(m.get(), mjOBJ_BODY, "body_ball")], srv.request.state.mass)
	    << "Mass did not change to the requested value";
	// reset
	srv.request.set_mass   = false;
	srv.request.reset_qpos = true;
	mujoco_ros::detail::setBodyStateCB(srv.request, srv.response);
	EXPECT_TRUE(srv.response.success);

	compare_qpos(d, m->jnt_qposadr[id_free], "ball_freejoint", { 1.0, 0.0, 0.06, 1.0, 0.0, 0.0, 0.0 });
	compare_qvel(d, m->jnt_dofadr[id_free], "ball_freejoint", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });

	mujoco_ros::requestExternalShutdown();
	mj_thread->join();
}

TEST_F(MujocoRosBaseFixture, set_geom_properties)
{
	nh->setParam("unpause", false);

	int ball_geom_id = mj_name2id(m.get(), mjOBJ_GEOM, "ball");
	int ball_body_id = mj_name2id(m.get(), mjOBJ_BODY, "body_ball");

	EXPECT_NE(ball_geom_id, -1) << "'ball' should be found as geom in model!";
	EXPECT_NE(ball_body_id, -1) << "'body_ball' should be found as body in model!";

	mujoco_ros_msgs::SetGeomProperties srv;

	// Invalid geom_name
	srv.request.properties.env_id = 0;
	srv.request.properties.name   = "unknown";
	mujoco_ros::detail::setGeomPropertiesCB(srv.request, srv.response);
	EXPECT_FALSE(srv.response.success);

	// Resolve geom
	srv.request.properties.name = "ball";
	mujoco_ros::detail::setGeomPropertiesCB(srv.request, srv.response);
	EXPECT_TRUE(srv.response.success);

	// set mass
	srv.request.set_mass             = true;
	srv.request.properties.body_mass = 0.299f;
	EXPECT_NE(m->body_mass[ball_body_id], srv.request.properties.body_mass) << "Mass already has requested value!";
	mujoco_ros::detail::setGeomPropertiesCB(srv.request, srv.response);
	EXPECT_TRUE(srv.response.success);
	EXPECT_EQ(m->body_mass[ball_body_id], srv.request.properties.body_mass)
	    << "Mass did not change to the requested value";

	// set friction
	srv.request.set_mass                  = false;
	srv.request.set_friction              = true;
	srv.request.properties.friction_slide = 0;
	srv.request.properties.friction_spin  = 0;
	srv.request.properties.friction_roll  = 0;
	EXPECT_TRUE(m->geom_friction[ball_geom_id * 3] != 0 && m->geom_friction[ball_geom_id * 3 + 1] != 0 &&
	            m->geom_friction[ball_geom_id * 3 + 2] != 0)
	    << "Some friction values already at 0!";
	mujoco_ros::detail::setGeomPropertiesCB(srv.request, srv.response);
	EXPECT_TRUE(srv.response.success);
	EXPECT_TRUE(m->geom_friction[ball_geom_id * 3] == 0) << "Slide friction unchanged!";
	EXPECT_TRUE(m->geom_friction[ball_geom_id * 3 + 1] == 0) << "Spin friction unchanged!";
	EXPECT_TRUE(m->geom_friction[ball_geom_id * 3 + 2] == 0) << "Roll friction uncahnged!";

	// set type (not checking PLANE, HFIELD, MESH, and rendering types)
	srv.request.set_friction = false;
	srv.request.set_type     = true;
	//   BOX
	srv.request.properties.type.value = mujoco_ros_msgs::GeomType::BOX;
	EXPECT_NE(env->model_->geom_type[ball_geom_id], mjGEOM_BOX) << "Geom already is of type BOX";
	mujoco_ros::detail::setGeomPropertiesCB(srv.request, srv.response);
	EXPECT_TRUE(srv.response.success);
	EXPECT_EQ(env->model_->geom_type[ball_geom_id], mjGEOM_BOX) << "Geom unchanged";
	//   CYLINDER
	srv.request.properties.type.value = mujoco_ros_msgs::GeomType::CYLINDER;
	mujoco_ros::detail::setGeomPropertiesCB(srv.request, srv.response);
	EXPECT_TRUE(srv.response.success);
	EXPECT_EQ(env->model_->geom_type[ball_geom_id], mjGEOM_CYLINDER) << "Geom unchanged";
	//  ELLIPSOID
	srv.request.properties.type.value = mujoco_ros_msgs::GeomType::ELLIPSOID;
	mujoco_ros::detail::setGeomPropertiesCB(srv.request, srv.response);
	EXPECT_TRUE(srv.response.success);
	EXPECT_EQ(env->model_->geom_type[ball_geom_id], mjGEOM_ELLIPSOID) << "Geom unchanged";
	//  CAPSULE
	srv.request.properties.type.value = mujoco_ros_msgs::GeomType::CAPSULE;
	mujoco_ros::detail::setGeomPropertiesCB(srv.request, srv.response);
	EXPECT_TRUE(srv.response.success);
	EXPECT_EQ(env->model_->geom_type[ball_geom_id], mjGEOM_CAPSULE) << "Geom unchanged";
	//  SPHERE
	srv.request.properties.type.value = mujoco_ros_msgs::GeomType::SPHERE;
	mujoco_ros::detail::setGeomPropertiesCB(srv.request, srv.response);
	EXPECT_TRUE(srv.response.success);
	EXPECT_EQ(env->model_->geom_type[ball_geom_id], mjGEOM_SPHERE) << "Geom unchanged";

	// set size
	srv.request.set_type          = false;
	srv.request.set_size          = true;
	srv.request.properties.size_0 = 0.01f;
	srv.request.properties.size_1 = 0.01f;
	srv.request.properties.size_2 = 0.01f;
	EXPECT_TRUE(env->model_->geom_size[ball_geom_id * 3] != 0.01 &&
	            env->model_->geom_size[ball_geom_id * 3 + 1] != 0.01 &&
	            env->model_->geom_size[ball_geom_id * 3 + 2] != 0.01)
	    << "Geom size is already 0.01 0.01 0.01!";
	mujoco_ros::detail::setGeomPropertiesCB(srv.request, srv.response);
	EXPECT_NEAR(env->model_->geom_size[ball_geom_id * 3], 0.01, 9e-4) << "Size 0 unchanged";
	EXPECT_NEAR(env->model_->geom_size[ball_geom_id * 3 + 1], 0.01, 9e-4) << "Size 1 unchanged";
	EXPECT_NEAR(env->model_->geom_size[ball_geom_id * 3 + 2], 0.01, 9e-4) << "Size 2 unchanged";

	mujoco_ros::requestExternalShutdown();
	mj_thread->join();
}

TEST_F(MujocoRosBaseFixture, get_body_state)
{
	nh->setParam("unpause", false);

	mujoco_ros_msgs::SetBodyState srv;
	srv.request.set_pose                      = true;
	srv.request.set_twist                     = true;
	srv.request.set_mass                      = true;
	srv.request.set_pose                      = true;
	srv.request.state.env_id                  = 0;
	srv.request.state.name                    = "body_ball";
	srv.request.state.mass                    = 0.299f;
	srv.request.state.pose.pose.position.x    = 1.0;
	srv.request.state.pose.pose.position.y    = 1.0;
	srv.request.state.pose.pose.position.z    = 1.0;
	srv.request.state.pose.pose.orientation.w = 1.0;
	srv.request.state.pose.pose.orientation.x = 0.0;
	srv.request.state.pose.pose.orientation.y = 0.0;
	srv.request.state.pose.pose.orientation.z = 0.0;
	srv.request.state.twist.twist.linear.x    = 0.1;
	srv.request.state.twist.twist.linear.y    = 0.1;
	srv.request.state.twist.twist.linear.z    = -0.1;
	srv.request.state.twist.twist.angular.x   = 0.1;
	srv.request.state.twist.twist.angular.y   = 0.1;
	srv.request.state.twist.twist.angular.z   = -0.1;
	mujoco_ros::detail::setBodyStateCB(srv.request, srv.response);
	EXPECT_TRUE(srv.response.success);

	mujoco_ros_msgs::GetBodyState g_srv;
	// wrong body name
	g_srv.request.env_id = 0;
	g_srv.request.name   = "unknown";

	// correct request
	g_srv.request.env_id = 0;
	g_srv.request.name   = "body_ball";
	mujoco_ros::detail::getBodyStateCB(g_srv.request, g_srv.response);
	EXPECT_TRUE(g_srv.response.success);
	EXPECT_EQ(g_srv.response.state.mass, srv.request.state.mass);
	EXPECT_EQ(g_srv.response.state.name, srv.request.state.name);
	EXPECT_EQ(g_srv.response.state.pose.pose, srv.request.state.pose.pose);
	EXPECT_EQ(g_srv.response.state.twist.twist, srv.request.state.twist.twist);

	// TODO: tests for bodies with a non-freejoint, no joint, and multiple joints (cannot set position but read!)

	mujoco_ros::requestExternalShutdown();
	mj_thread->join();
}

TEST_F(MujocoRosBaseFixture, get_geom_properties)
{
	nh->setParam("unpause", false);

	mujoco_ros_msgs::SetGeomProperties srv;
	srv.request.set_type                  = true;
	srv.request.set_mass                  = true;
	srv.request.set_size                  = true;
	srv.request.set_friction              = true;
	srv.request.properties.env_id         = 0;
	srv.request.properties.name           = "ball";
	srv.request.properties.type.value     = mujoco_ros_msgs::GeomType::BOX;
	srv.request.properties.body_mass      = 0.299f;
	srv.request.properties.size_0         = 0.01f;
	srv.request.properties.size_1         = 0.01f;
	srv.request.properties.size_2         = 0.01f;
	srv.request.properties.friction_slide = 1.;
	srv.request.properties.friction_spin  = 1.;
	srv.request.properties.friction_roll  = 1.;
	mujoco_ros::detail::setGeomPropertiesCB(srv.request, srv.response);
	EXPECT_TRUE(srv.response.success);

	mujoco_ros_msgs::GetGeomProperties g_srv;
	// wrong geom name
	g_srv.request.env_id    = 0;
	g_srv.request.geom_name = "unknown";
	mujoco_ros::detail::getGeomPropertiesCB(g_srv.request, g_srv.response);
	EXPECT_FALSE(g_srv.response.success);

	g_srv.request.geom_name = "ball";
	mujoco_ros::detail::getGeomPropertiesCB(g_srv.request, g_srv.response);
	EXPECT_TRUE(g_srv.response.success);

	EXPECT_EQ(srv.request.properties.env_id, g_srv.response.properties.env_id);
	EXPECT_EQ(srv.request.properties.name, g_srv.response.properties.name);
	EXPECT_EQ(srv.request.properties.type.value, g_srv.response.properties.type.value);
	EXPECT_EQ(srv.request.properties.body_mass, g_srv.response.properties.body_mass);
	EXPECT_EQ(srv.request.properties.size_0, g_srv.response.properties.size_0);
	EXPECT_EQ(srv.request.properties.size_1, g_srv.response.properties.size_1);
	EXPECT_EQ(srv.request.properties.size_2, g_srv.response.properties.size_2);
	EXPECT_EQ(srv.request.properties.friction_slide, g_srv.response.properties.friction_slide);
	EXPECT_EQ(srv.request.properties.friction_spin, g_srv.response.properties.friction_spin);
	EXPECT_EQ(srv.request.properties.friction_roll, g_srv.response.properties.friction_roll);

	mujoco_ros::requestExternalShutdown();
	mj_thread->join();
}

} // namespace unit_testing
