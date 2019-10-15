/* Author: Mincheul Kang */

#ifndef HARMONIOUS_SAMPLING_MANIPULATIONREGION_H
#define HARMONIOUS_SAMPLING_MANIPULATIONREGION_H

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <harmonious_msgs/BaseConf.h>
#include <harmonious_sampling/HarmoniousParameters.h>
#include <harmonious_msgs/JointConf.h>

class MainpulationRegion
{
public:
    MainpulationRegion(planning_scene::PlanningScenePtr& planning_scene,
                       std::string planning_group,
                       std::string collision_check_group,
                       HarmoniousParameters &params,
                       std::vector<harmonious_msgs::BaseConf> &base_confs,
                       std::vector<harmonious_msgs::JointConf> &joint_confs);
    ~MainpulationRegion();

    bool isValid(harmonious_msgs::BaseConf &bp);
    void generate_regions(ompl::base::RealVectorBounds &bounds);

    bool***                                     regions_;
private:
    collision_detection::CollisionRequest       collision_request_;
    planning_scene::PlanningScenePtr            planning_scene_;
    std::string                                 planning_group_;
    std::vector<harmonious_msgs::BaseConf>      &base_confs_;
    std::vector<harmonious_msgs::JointConf>     &joint_confs_;
    HarmoniousParameters                        &params_;
};

#endif //HARMONIOUS_SAMPLING_MANIPULATIONREGION_H
