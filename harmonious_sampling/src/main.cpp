/* Author: Mincheul Kang */

#include <ros/ros.h>
#include <boost/scope_exit.hpp>
#include <boost/date_time.hpp>
#include <cstdlib>

// ompl
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/goals/GoalStates.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// kinematics
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_conversions/kdl_msg.h>

// reachability map
#include <harmonious_msgs/PossibleConf.h>

// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

// Harmonious_sampling
#include <harmonious_sampling/StateSpaces.h>
#include <harmonious_sampling/Scene.h>
#include <harmonious_sampling/HarmoniousSampler.h>
#include <harmonious_sampling/ManipulationRegion.h>
#include <harmonious_sampling/HarmoniousParameters.h>
#include <harmonious_sampling/HarmoniousLazyPRMstarMulti.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "harmonious");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    //// Please set your group in moveit!.
    const std::string PLANNING_GROUP = "base_with_right_arm";
    const std::string BASE_GROUP = "base";
    const std::string MANI_COLL_CHECK_GROUP = "without_right_arm";
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    //// Please set your start configuration and predefined configuration.
    std::vector<double> s_conf, pre_conf;
    joint_model_group->getVariableDefaultPositions(s_conf);
    s_conf[0] = -1.8;
    s_conf[1] = -1.0;
    joint_model_group->getVariableDefaultPositions(pre_conf);
    pre_conf.erase(pre_conf.begin(), pre_conf.begin()+NUM_BASE_DOF);

    // moveit planning scene
    ScenePtr scene(new Scene(planning_scene, "world"));
    scene->addCollisionObjects();
    scene->updateCollisionScene();

    // state space
    ompl::base::StateSpacePtr state_space;
    std::string collisionCheckGroup;
    robot_model::JointBoundsVector joint_bounds;
    joint_bounds = joint_model_group->getActiveJointModelsBounds();
    unsigned int num_dof = (unsigned int)joint_bounds.size();
    std::vector<std::string> indices = joint_model_group->getActiveJointModelNames();
    collisionCheckGroup = PLANNING_GROUP;

    ompl::base::RealVectorBounds bounds(num_dof);

    for (std::size_t i = 0; i < joint_bounds.size(); ++i) {
        const robot_model::JointModel::Bounds& b = *joint_bounds[i];
        bounds.setLow(i, b[0].min_position_);
        bounds.setHigh(i, b[0].max_position_);
    }

    //// Please set your boundaries for base space.
    bounds.setLow(0, -2.5);
    bounds.setHigh(0, 2.5);
    bounds.setLow(1, -2.0);
    bounds.setHigh(1, 2.0);
    bounds.setLow(2, -M_PI);
    bounds.setHigh(2, M_PI);

    HarmoniousParameters hp(bounds);

    // inverse reachability map
    ros::ServiceClient client = node_handle.serviceClient<harmonious_msgs::PossibleConf>("/possible_conf_generator");
    harmonious_msgs::PossibleConf srv;

    //// Please set the number of possible base and joint configurations for grasp pose.
    srv.request.base_num = 2000;
    srv.request.joint_num = 200;

    //// Please set the target grasp pose.
    geometry_msgs::Pose grasp_pose;
    grasp_pose.position.x = 2.0;
    grasp_pose.position.y = 1.6;
    grasp_pose.position.z = 0.95;
    grasp_pose.orientation.w = 0.5;
    grasp_pose.orientation.x = -0.5;
    grasp_pose.orientation.y = 0.5;
    grasp_pose.orientation.z = 0.5;

    srv.request.grasp_poses.push_back(grasp_pose);
    client.call(srv);

    if(srv.response.base_confs.size() == 0){
        ROS_ERROR("No possible base and joint configurations.");
        return 0;
    }
    ros::Duration(0.1).sleep();

    // construct a manipulation region
    MainpulationRegion mr(planning_scene, BASE_GROUP, MANI_COLL_CHECK_GROUP, hp, srv.response.base_confs, srv.response.joint_confs);
    mr.generate_regions(bounds);

    // set harmonious sampler
    ompl::base::HarmoniousSampler h(srv.request.grasp_poses, srv.response.base_confs, srv.response.joint_confs,
                                    hp, mr, pre_conf);

    //// ompl
    state_space.reset(new ompl::base::RealVectorStateSpace(num_dof));
    state_space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    state_space->setup();

    // simple setup
    ompl::geometric::SimpleSetupPtr simple_setup;
    simple_setup.reset(new ompl::geometric::SimpleSetup(state_space));

    // state validity checker
    StateValidityCheckerPtr validity_checker;
    validity_checker.reset(new StateValidityChecker(simple_setup->getSpaceInformation(), planning_scene, PLANNING_GROUP, collisionCheckGroup));
    simple_setup->setStateValidityChecker(std::static_pointer_cast<ompl::base::StateValidityChecker>(validity_checker));

    typedef ompl::base::ScopedState<ompl::base::StateSpace> ScopedState;

    // set initial state
    ScopedState q_start(state_space);
    for (size_t i = 0; i < num_dof; i++) {
        q_start[i] = s_conf[i];
    }
    simple_setup->addStartState(q_start);

    // set goal state
    ScopedState q_goal(state_space);
    for (size_t i = 0; i < num_dof; i++) {
        q_goal[i] = bounds.low[i]; // trash_value
    }
    simple_setup->setGoalState(q_goal);
    simple_setup->setStartAndGoalStates(q_start, q_goal);

    // set planner
    simple_setup->setPlanner(ompl::base::PlannerPtr(new ompl::geometric::HarmoniousLazyPRMstarMulti(simple_setup->getSpaceInformation(), h)));
    simple_setup->solve(ompl::base::timedPlannerTerminationCondition(10.0));

    if (simple_setup->haveSolutionPath()) {
//        simple_setup->simplifySolution();
        ompl::geometric::PathGeometric &p = simple_setup->getSolutionPath();
//        simple_setup->getPathSimplifier()->simplifyMax(p);
//        simple_setup->getPathSimplifier()->smoothBSpline(p);
        ros::Publisher display_pub = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
        moveit_msgs::DisplayTrajectory display_trajectory;
        moveit_msgs::RobotTrajectory robot_traj;
        const moveit::core::JointModelGroup* model_group = planning_scene->getRobotModel()->getJointModelGroup(PLANNING_GROUP);
        const std::vector<std::string>& active_joint_names = model_group->getActiveJointModelNames();
        robot_traj.joint_trajectory.joint_names = active_joint_names;
        robot_traj.joint_trajectory.points.resize(p.getStateCount());

        for(uint i = 0; i < p.getStateCount(); i++){
            ompl::base::RealVectorStateSpace::StateType *rstate = static_cast<ompl::base::RealVectorStateSpace::StateType*>(p.getState(i));
            robot_traj.joint_trajectory.points[i].positions.resize(num_dof);
            for (uint j = 0; j < num_dof; j++){
                robot_traj.joint_trajectory.points[i].positions[j] = rstate->values[j];
            }
            robot_traj.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
        }
        display_trajectory.trajectory.push_back(robot_traj);
        display_pub.publish(display_trajectory);
        ros::Duration(1.0).sleep();
    }

    return 0;
}
