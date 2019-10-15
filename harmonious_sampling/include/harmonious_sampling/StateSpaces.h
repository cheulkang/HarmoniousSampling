/* Author: Mincheul Kang */

#ifndef HARMONIOUS_SAMPLING_STATESPACES_H_
#define HARMONIOUS_SAMPLING_STATESPACES_H_

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/move_group_interface/move_group_interface.h>

class StateValidityChecker: public ompl::base::StateValidityChecker {
public:
    StateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                         planning_scene::PlanningScenePtr& planning_scene,
                         std::string planning_group,
                         std::string collision_check_group);
    virtual bool isValid(const ompl::base::State *state) const;
protected:
    ompl::base::StateSpace * state_space_;
    collision_detection::CollisionRequest collision_request_;
    planning_scene::PlanningScenePtr planning_scene_;
    std::string planning_group_;
};

typedef std::shared_ptr<StateValidityChecker> StateValidityCheckerPtr;

#endif