/* Author: Mincheul Kang */

#include <boost/chrono.hpp>
#include <boost/foreach.hpp>
#include <ompl/base/SpaceInformation.h>

#include <harmonious_sampling/StateSpaces.h>

StateValidityChecker::StateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                                                   planning_scene::PlanningScenePtr& planning_scene,
                                                   std::string planning_group,
                                                   std::string collision_check_group):
        ompl::base::StateValidityChecker(si),
        state_space_(si->getStateSpace().get())
{
    planning_scene_ = planning_scene;
    planning_group_ = planning_group;

    collision_request_.group_name = collision_check_group;
    collision_request_.contacts = true;
    collision_request_.max_contacts = 100;
    collision_request_.max_contacts_per_pair = 1;
    collision_request_.verbose = false;
}

bool StateValidityChecker::isValid(const ompl::base::State *state) const {
    std::map<std::string, double> configuration;
    collision_detection::CollisionResult collision_result;
    std::vector<double> values;
    state_space_->copyToReals(values, state);

    collision_result.clear();

    robot_state::RobotState robot_state = planning_scene_->getCurrentStateNonConst();
    robot_state.setJointGroupPositions(planning_group_, values);
    planning_scene_->checkCollision(collision_request_, collision_result, robot_state);

    return !collision_result.collision;
}