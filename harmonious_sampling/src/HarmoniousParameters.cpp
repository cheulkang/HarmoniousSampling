/* Author: Mincheul Kang */

#include <harmonious_sampling/HarmoniousParameters.h>

HarmoniousParameters::HarmoniousParameters(ompl::base::RealVectorBounds &bounds):
    bounds_(bounds)
{
    resolution_ = 10;
    prob_goal_ = 1;
    prob_uniform_ = 20;
    max_num_goal_ = 50;

    weights_.push_back(1.0);
    weights_.push_back(1.0);
    weights_.push_back(0.421);
    weights_.push_back(0.921);
    weights_.push_back(0.921);
    weights_.push_back(0.921);
    weights_.push_back(0.521);
    weights_.push_back(0.521);
    weights_.push_back(0.2);
    weights_.push_back(0.2);

    reg_index_[0] = uint((bounds_.high[0]-bounds_.low[0])*resolution_); // x
    reg_index_[1] = uint((bounds_.high[1]-bounds_.low[1])*resolution_); // y
    reg_index_[2] = 8; // yaw

    res_yaw_ = reg_index_[2]/M_PI/2;

    factor_base_ = 0.01;
    factor_joint_ = 0.005;

    interval_[0] = 1/resolution_/2;
    interval_[1] = 1/resolution_/2;
    interval_[2] = 1/res_yaw_/2;

    v_base_ = 1.0;
//    for(uint i = 0; i < 3; i++){
//        v_base_ *= weights_[i]*(bounds_.high[i]-bounds_.low[i]);
//    }
    v_manip_ = v_base_;
    for(uint i = NUM_BASE_DOF; i < weights_.size(); i++){
        v_manip_ *= weights_[i]*(bounds_.high[i]-bounds_.low[i]);
    }
}

HarmoniousParameters::~HarmoniousParameters(){

}
