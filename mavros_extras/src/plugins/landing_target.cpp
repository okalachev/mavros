/**
 * @brief LandingTarget plugin
 * @file langing_target.cpp
 * @author Oleg Kalachev <okalachev@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2018 Oleg Kalachev.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>


namespace mavros {
namespace extra_plugins{
using mavlink::common::MAV_FRAME;
/**
 * @brief LandingTarget plugin
 *
 * Sends and receives landing target pose from FCU.
 */
class LandingTargetPlugin : public plugin::PluginBase
{
public:
	LandingTargetPlugin() : PluginBase(),
		lt_nh("~landing_target")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		lt_pose_sub = lt_nh.subscribe("send", 1, &LandingTargetPlugin::lt_pose_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle lt_nh;

	ros::Subscriber lt_pose_sub;

	/* -*- low-level send -*- */
	void lt_pose_send
		(uint64_t usec,
			Eigen::Quaterniond &q,
			Eigen::Vector3d &v)
	{
		mavlink::common::msg::LANDING_TARGET msg;

		msg.time_usec = usec;
		msg.frame = utils::enum_value(MAV_FRAME::LOCAL_NED);
		msg.x = v.x();
		msg.y = v.y();
		msg.z = v.z();
		ftf::quaternion_to_mavlink(q, msg.q);
		msg.position_valid = true;

		UAS_FCU(m_uas)->send_message_ignore_drop(msg);
	}

	/* -*- mid-level helpers -*- */
	void lt_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose)
	{
		Eigen::Quaterniond q_enu;

		tf::quaternionMsgToEigen(pose->pose.orientation, q_enu);
		auto q = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(q_enu));

		auto position = ftf::transform_frame_enu_ned(
				Eigen::Vector3d(
					pose->pose.position.x,
					pose->pose.position.y,
					pose->pose.position.z));

		lt_pose_send(pose->header.stamp.toNSec() / 1000,
				q,
				position);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::LandingTargetPlugin, mavros::plugin::PluginBase)
