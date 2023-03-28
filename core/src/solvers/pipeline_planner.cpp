/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
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

/* Authors: Robert Haschke
   Desc:    plan using MoveIt's PlanningPipeline
*/

#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/task.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit/kinematic_constraints/utils.h>

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace moveit {
namespace task_constructor {
namespace solvers {

struct PlannerCache
{
	using PlannerID = std::tuple<std::string, std::string>;
	using PlannerMap = std::map<PlannerID, std::weak_ptr<planning_pipeline::PlanningPipeline> >;
	using ModelList = std::list<std::pair<std::weak_ptr<const moveit::core::RobotModel>, PlannerMap> >;
	ModelList cache_;

	PlannerMap::mapped_type& retrieve(const moveit::core::RobotModelConstPtr& model, const PlannerID& planner_id) {
		// find model in cache_ and remove expired entries while doing so
		ModelList::iterator model_it = cache_.begin();
		while (model_it != cache_.end()) {
			if (model_it->first.expired()) {
				model_it = cache_.erase(model_it);
				continue;
			}
			if (model_it->first.lock() == model)
				break;
			++model_it;
		}
		if (model_it == cache_.end())  // if not found, create a new PlannerMap for this model
			model_it = cache_.insert(cache_.begin(), std::make_pair(model, PlannerMap()));

		return model_it->second.insert(std::make_pair(planner_id, PlannerMap::mapped_type())).first->second;
	}
};

planning_pipeline::PlanningPipelinePtr PipelinePlanner::create(const rclcpp::Node::SharedPtr& node,
                                                               const PipelinePlanner::Specification& specification) {
	static PlannerCache cache;

	static constexpr char const* PLUGIN_PARAMETER_NAME = "planning_plugin";

	std::string pipeline_ns = specification.ns;
	const std::string parameter_name = pipeline_ns + "." + PLUGIN_PARAMETER_NAME;
	// fallback to old structure for pipeline parameters in MoveIt
	if (!node->has_parameter(parameter_name)) {
		node->declare_parameter(parameter_name, rclcpp::ParameterType::PARAMETER_STRING);
	}
	if (std::string parameter; !node->get_parameter(parameter_name, parameter)) {
		RCLCPP_WARN(node->get_logger(), "Failed to find '%s.%s'. %s", pipeline_ns.c_str(), PLUGIN_PARAMETER_NAME,
		            "Attempting to load pipeline from old parameter structure. Please update your MoveIt config.");
		pipeline_ns = "move_group";
	}

	PlannerCache::PlannerID id(pipeline_ns, specification.adapter_param);

	std::weak_ptr<planning_pipeline::PlanningPipeline>& entry = cache.retrieve(specification.model, id);
	planning_pipeline::PlanningPipelinePtr planner = entry.lock();
	if (!planner) {
		// create new entry
		planner = std::make_shared<planning_pipeline::PlanningPipeline>(
		    specification.model, node, pipeline_ns, PLUGIN_PARAMETER_NAME, specification.adapter_param);
		// store in cache
		entry = planner;
	}
	return planner;
}

PipelinePlanner::PipelinePlanner(const rclcpp::Node::SharedPtr& node, const std::string& pipeline_name)
  : pipeline_name_{ pipeline_name }, node_(node) {
	auto& planner_properties = properties();
	planner_properties.declare<std::string>("planner", "", "planner id");

	planner_properties.declare<uint>("num_planning_attempts", 1u, "number of planning attempts");
	planner_properties.declare<moveit_msgs::msg::WorkspaceParameters>(
	    "workspace_parameters", moveit_msgs::msg::WorkspaceParameters(), "allowed workspace of mobile base?");

	planner_properties.declare<double>("goal_joint_tolerance", 1e-4, "tolerance for reaching joint goals");
	planner_properties.declare<double>("goal_position_tolerance", 1e-4, "tolerance for reaching position goals");
	planner_properties.declare<double>("goal_orientation_tolerance", 1e-4, "tolerance for reaching orientation goals");

	planner_properties.declare<bool>("display_motion_plans", false,
	                                 "publish generated solutions on topic " +
	                                     planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC);
	planner_properties.declare<bool>("publish_planning_requests", false,
	                                 "publish motion planning requests on topic " +
	                                     planning_pipeline::PlanningPipeline::MOTION_PLAN_REQUEST_TOPIC);
}

PipelinePlanner::PipelinePlanner(const planning_pipeline::PlanningPipelinePtr& planning_pipeline)
  : PipelinePlanner(rclcpp::Node::SharedPtr()) {
	planner_ = planning_pipeline;
}

void PipelinePlanner::init(const core::RobotModelConstPtr& robot_model) {
	if (!planner_) {
		Specification specification;
		specification.model = robot_model;
		specification.pipeline = pipeline_name_;
		specification.ns = pipeline_name_;
		planner_ = create(node_, specification);
	} else if (robot_model != planner_->getRobotModel()) {
		throw std::runtime_error(
		    "The robot model of the planning pipeline isn't the same as the task's robot model -- "
		    "use Task::setRobotModel for setting the robot model when using custom planning pipeline");
	}
	planner_->displayComputedMotionPlans(properties().get<bool>("display_motion_plans"));
	planner_->publishReceivedRequests(properties().get<bool>("publish_planning_requests"));
}

void initMotionPlanRequest(moveit_msgs::msg::MotionPlanRequest& request, const PropertyMap& p,
                           const moveit::core::JointModelGroup* joint_model_group, double timeout) {
	request.group_name = joint_model_group->getName();
	request.planner_id = p.get<std::string>("planner");
	request.allowed_planning_time = timeout;
	request.start_state.is_diff = true;  // we don't specify an extra start state

	request.num_planning_attempts = p.get<uint>("num_planning_attempts");
	request.max_velocity_scaling_factor = p.get<double>("max_velocity_scaling_factor");
	request.max_acceleration_scaling_factor = p.get<double>("max_acceleration_scaling_factor");
	request.workspace_parameters = p.get<moveit_msgs::msg::WorkspaceParameters>("workspace_parameters");
}

bool PipelinePlanner::plan(const planning_scene::PlanningSceneConstPtr& from,
                           const planning_scene::PlanningSceneConstPtr& to,
                           const moveit::core::JointModelGroup* joint_model_group, double timeout,
                           robot_trajectory::RobotTrajectoryPtr& result,
                           const moveit_msgs::msg::Constraints& path_constraints) {
	const auto& planner_properties = properties();
	moveit_msgs::msg::MotionPlanRequest request;
	initMotionPlanRequest(request, planner_properties, joint_model_group, timeout);

	request.goal_constraints.resize(1);
	request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(
	    to->getCurrentState(), joint_model_group, planner_properties.get<double>("goal_joint_tolerance"));
	request.path_constraints = path_constraints;

	::planning_interface::MotionPlanResponse response;
	bool success = planner_->generatePlan(from, request, response);
	result = response.trajectory;
	return success;
}

bool PipelinePlanner::plan(const planning_scene::PlanningSceneConstPtr& from, const moveit::core::LinkModel& link,
                           const Eigen::Isometry3d& offset, const Eigen::Isometry3d& target_eigen,
                           const moveit::core::JointModelGroup* joint_model_group, double timeout,
                           robot_trajectory::RobotTrajectoryPtr& result,
                           const moveit_msgs::msg::Constraints& path_constraints) {
	const auto& planner_properties = properties();
	moveit_msgs::msg::MotionPlanRequest request;
	initMotionPlanRequest(request, planner_properties, joint_model_group, timeout);

	geometry_msgs::msg::PoseStamped target;
	target.header.frame_id = from->getPlanningFrame();
	target.pose = tf2::toMsg(target_eigen * offset.inverse());

	request.goal_constraints.resize(1);
	request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(
	    link.getName(), target, planner_properties.get<double>("goal_position_tolerance"),
	    planner_properties.get<double>("goal_orientation_tolerance"));
	request.path_constraints = path_constraints;

	::planning_interface::MotionPlanResponse response;
	bool success = planner_->generatePlan(from, request, response);
	result = response.trajectory;
	return success;
}
}  // namespace solvers
}  // namespace task_constructor
}  // namespace moveit
