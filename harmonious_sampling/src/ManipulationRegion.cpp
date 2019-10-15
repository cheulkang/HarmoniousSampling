/* Author: Mincheul Kang */

#include <boost/chrono.hpp>
#include <boost/foreach.hpp>
#include <ompl/base/SpaceInformation.h>

#include <harmonious_sampling/ManipulationRegion.h>

MainpulationRegion::MainpulationRegion(planning_scene::PlanningScenePtr& planning_scene,
                                       std::string planning_group,
                                       std::string collision_check_group,
                                       HarmoniousParameters &params,
                                       std::vector<harmonious_msgs::BaseConf> &base_confs,
                                       std::vector<harmonious_msgs::JointConf> &joint_confs):
        base_confs_(base_confs),
        joint_confs_(joint_confs),
        params_(params)
{
    planning_scene_ = planning_scene;
    planning_group_ = planning_group; // base_group

    collision_request_.group_name = collision_check_group;
    collision_request_.contacts = true;
    collision_request_.max_contacts = 100;
    collision_request_.max_contacts_per_pair = 1;
    collision_request_.verbose = false;

    regions_ = new bool**[params_.reg_index_[0]];
    for (uint i = 0; i < params_.reg_index_[0]; i++){
        regions_[i] = new bool*[params_.reg_index_[1]];
        for (uint j = 0; j < params_.reg_index_[1]; j++){
            regions_[i][j] = new bool[params_.reg_index_[2]];
        }
    }

    for(uint i = 0; i < params_.reg_index_[0]; i++){
        for(uint j = 0; j < params_.reg_index_[1]; j++) {
            for (uint k = 0; k < params_.reg_index_[2]; k++) {
                regions_[i][j][k] = false;
            }
        }
    }
}

MainpulationRegion::~MainpulationRegion(){
    for(uint i=0; i<params_.reg_index_[0]; i++) {
        for(uint j=0; j<params_.reg_index_[1]; j++)
            delete [] regions_[i][j];
        delete [] regions_[i];
    }
    delete [] regions_;
}

bool MainpulationRegion::isValid(harmonious_msgs::BaseConf &bp) {
    std::map<std::string, double> configuration;
    collision_detection::CollisionResult collision_result;
    std::vector<double> values;

    values.push_back(bp.b_c[0]);
    values.push_back(bp.b_c[1]);
    values.push_back(bp.b_c[2]);

    collision_result.clear();

    robot_state::RobotState robot_state = planning_scene_->getCurrentStateNonConst();
    robot_state.setJointGroupPositions(planning_group_, values);
    planning_scene_->checkCollision(collision_request_, collision_result, robot_state);

    return !collision_result.collision;
}

void MainpulationRegion::generate_regions(ompl::base::RealVectorBounds &bounds){
    for (uint i = 0; i < base_confs_.size(); i++){
        if(base_confs_[i].b_c[0] < bounds.low[0] || base_confs_[i].b_c[0] > bounds.high[0]){
            joint_confs_[i].j_c.clear();
            continue;
        }
        if(base_confs_[i].b_c[1] < bounds.low[1] || base_confs_[i].b_c[1] > bounds.high[1]){
            joint_confs_[i].j_c.clear();
            continue;
        }

        if(isValid(base_confs_[i])){
            regions_[int(((base_confs_[i]).b_c[0]-params_.bounds_.low[0]) * params_.resolution_)]
            [int(((base_confs_[i]).b_c[1]-params_.bounds_.low[1]) * params_.resolution_)]
            [int(((base_confs_[i]).b_c[2]-params_.bounds_.low[2]) * params_.res_yaw_)] = true;
        }
    }
}