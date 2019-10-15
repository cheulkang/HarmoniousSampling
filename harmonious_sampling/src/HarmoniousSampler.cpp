/* Author: Mincheul Kang */

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <fstream>
#include <queue>
#include <stack>
#include <utility>
#include <numeric>
#include <stdio.h>
#include <iostream>
#include <string>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/lookup_edge.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/minmax_element.hpp>
#include <boost/math/constants/constants.hpp>

#include <harmonious_sampling/HarmoniousSampler.h>

ompl::base::HarmoniousSampler::HarmoniousSampler(std::vector<geometry_msgs::Pose> &grasp_poses,
                                                 std::vector<harmonious_msgs::BaseConf> &base_confs,
                                                 std::vector<harmonious_msgs::JointConf> &joint_confs,
                                                 HarmoniousParameters &params,
                                                 MainpulationRegion &mr,
                                                 std::vector<double> &defined_joint_conf):
        grasp_poses_(grasp_poses),
        base_confs_(base_confs),
        joint_confs_(joint_confs),
        params_(params),
        defined_joint_conf_(defined_joint_conf),
        mr_(mr)
{
    size_of_gp = (uint)grasp_poses_.size();
    num_possible_confs_ = 0;
    num_dof_ = defined_joint_conf.size() + NUM_BASE_DOF;

    prob_dist_ = new double**[params_.reg_index_[0]];
    cumul_dist_ = new double**[params_.reg_index_[0]];
    for (uint i = 0; i < params_.reg_index_[0]; i++){
        prob_dist_[i] = new double*[params_.reg_index_[1]];
        cumul_dist_[i] = new double*[params_.reg_index_[1]];
        for (uint j = 0; j < params_.reg_index_[1]; j++){
            prob_dist_[i][j] = new double[params_.reg_index_[2]];
            cumul_dist_[i][j] = new double[params_.reg_index_[2]];
        }
    }

    double pre = 0.0;
    for(uint i = 0; i < params_.reg_index_[0]; i++){
        ddd.push_back(0);
        for(uint j = 0; j < params_.reg_index_[1]; j++) {
            for (uint k = 0; k < params_.reg_index_[2]; k++) {
                if(mr.regions_[i][j][k]){
                    prob_dist_[i][j][k] = params_.v_manip_;
                }
                else{
                    prob_dist_[i][j][k] = params_.v_base_;
                }
                pre += prob_dist_[i][j][k];
                cumul_dist_[i][j][k] = pre;
            }
        }
    }
}

ompl::base::HarmoniousSampler::~HarmoniousSampler(){
    for(uint i=0; i<params_.reg_index_[0]; i++) {
        for(uint j=0; j<params_.reg_index_[1]; j++){
            delete [] prob_dist_[i][j];
            delete [] cumul_dist_[i][j];
        }
        delete [] prob_dist_[i];
        delete [] cumul_dist_[i];
    }
    delete [] prob_dist_;
    delete [] cumul_dist_;
}

bool ompl::base::HarmoniousSampler::makeGoal(const SpaceInformationPtr si_, ompl::base::State *workState){
    base::RealVectorStateSpace::StateType *rstate = static_cast<base::RealVectorStateSpace::StateType*>(workState);

    for(; num_possible_confs_ < base_confs_.size(); num_possible_confs_++){
        if(joint_confs_[num_possible_confs_].j_c.size() != 0){
            for (int i = 0; i < num_dof_; i++){
                if(i < NUM_BASE_DOF){
                    rstate->values[i] = base_confs_[num_possible_confs_].b_c[i];
                }
                else{
                    rstate->values[i] = joint_confs_[num_possible_confs_].j_c[i-NUM_BASE_DOF];
                }
            }
            num_possible_confs_++;
            if (!si_->isValid(workState)) {
                return false;
            }
            else{
                return true;
            }
        }
    }

    return false;
}

bool ompl::base::HarmoniousSampler::getReachable(ompl::base::State *workState){
    base::RealVectorStateSpace::StateType *rstate = static_cast<base::RealVectorStateSpace::StateType*>(workState);

    return mr_.regions_[int((rstate->values[0]-params_.bounds_.low[0]) * params_.resolution_)]
                       [int((rstate->values[1]-params_.bounds_.low[1]) * params_.resolution_)]
                       [int((rstate->values[2]-params_.bounds_.low[2]) * params_.res_yaw_)];
}

void ompl::base::HarmoniousSampler::jointSampling(ompl::base::State *workState) {
    RealVectorStateSpace::StateType *rstate = static_cast<RealVectorStateSpace::StateType*>(workState);

    for(int i = NUM_BASE_DOF; i < num_dof_; i++){
        rstate->values[i] = rng_.uniformReal(params_.bounds_.low[i], params_.bounds_.high[i]);
    }
}

void ompl::base::HarmoniousSampler::uniformSampling(ompl::base::State *workState) {
    RealVectorStateSpace::StateType *rstate = static_cast<RealVectorStateSpace::StateType*>(workState);

    for(int i = 0; i < num_dof_; i++){
        rstate->values[i] = rng_.uniformReal(params_.bounds_.low[i], params_.bounds_.high[i]);
    }
}

void ompl::base::HarmoniousSampler::biasedSampling(ompl::base::State *workState) {
    base::RealVectorStateSpace::StateType *rstate = static_cast<base::RealVectorStateSpace::StateType*>(workState);

    double f = (double)rand() / RAND_MAX;
    double cdf_rnd = f * (cumul_dist_[params_.reg_index_[0]-1][params_.reg_index_[1]-1][params_.reg_index_[2]-1]);

    std::vector<int> index = findIndex(cdf_rnd);

    ddd[index[0]]++;

    double x = (index[0] + 0.5) / params_.resolution_ + params_.bounds_.low[0];
    rstate->values[0] = rng_.uniformReal(x - params_.interval_[0], x + params_.interval_[0]);
    double y = (index[1] + 0.5) / params_.resolution_ + params_.bounds_.low[1];
    rstate->values[1] = rng_.uniformReal(y - params_.interval_[1], y + params_.interval_[1]);
    double theta = (index[2] + 0.5) / params_.res_yaw_ + params_.bounds_.low[2];
    rstate->values[2] = rng_.uniformReal(theta - params_.interval_[2], theta + params_.interval_[2]);
}

void ompl::base::HarmoniousSampler::setBasicPose(ompl::base::State *workState){
    base::RealVectorStateSpace::StateType *rstate = static_cast<base::RealVectorStateSpace::StateType*>(workState);

    for (int i = NUM_BASE_DOF; i < num_dof_; i++)
        rstate->values[i] = defined_joint_conf_[i-NUM_BASE_DOF];
}

std::vector<int> ompl::base::HarmoniousSampler::findIndex(const double value) {
    std::vector<int> index(NUM_BASE_DOF, 0);

    if(value < cumul_dist_[0][0][0]){
        return index;
    }

    index[0] = bSearch(value, 0, params_.reg_index_[0]-1);
    index[1] = bSearch(value, 0, params_.reg_index_[1]-1, index[0]);
    index[2] = bSearch(value, 0, params_.reg_index_[2]-1, index[0], index[1]);

    return index;
}

int ompl::base::HarmoniousSampler::bSearch(double val, int first, int last) {
    int mid;

    while(first <= last) {
        mid = (first + last) / 2;
        if (cumul_dist_[mid][0][0] < val && cumul_dist_[mid][params_.reg_index_[1]-1][params_.reg_index_[2]-1] >= val){
            return mid;
        }

        if (cumul_dist_[mid][0][0] < val) {
            first = mid + 1;
        }
        else if (cumul_dist_[mid][0][0] > val) {
            last = mid - 1;
        }
    }

    return mid;
}

int ompl::base::HarmoniousSampler::bSearch(double val, int first, int last, int x) {
    int mid;

    while(first <= last) {
        mid = (first + last) / 2;
        if (cumul_dist_[x][mid][0] < val && cumul_dist_[x][mid][params_.reg_index_[2]-1] >= val){
            return mid;
        }

        if (cumul_dist_[x][mid][0] < val) {
            first = mid + 1;
        }
        else if (cumul_dist_[x][mid][0] > val) {
            last = mid - 1;
        }
    }

    return mid;
}

int ompl::base::HarmoniousSampler::bSearch(double val, int first, int last, int x, int y) {
    int mid = 0;
    while(first <= last)
    {
        mid = (first + last) / 2;
        if (mid == 0) {
            return mid;
        }
        if (cumul_dist_[x][y][mid] < val && cumul_dist_[x][y][mid-1] >= val){
            return mid;
        }
        if (cumul_dist_[x][y][mid] < val) {
            first = mid + 1;
        }
        else if (cumul_dist_[x][y][mid] > val) {
            last = mid - 1;
        }
    }

    return mid;
}

