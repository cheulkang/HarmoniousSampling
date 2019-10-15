/* Author: Mincheul Kang */

#include <harmonious_sampling/Scene.h>
#include <tf/LinearMath/Quaternion.h>

Scene::Scene(planning_scene::PlanningScenePtr& planning_scene, std::string frameID){
    planning_scene_ = planning_scene;
    frameID_ = frameID;
}

void Scene::addCollisionObjects(){
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    collision_objects.push_back(addBox("table", 0.0, 0.0, 0.7, 0.6, 1.2, 0.05, 0.0));
    collision_objects.push_back(addBox("s0", 0.0 + 2.0,   0.0+1.6,  1.33,   0.8, 0.28, 0.02, 0.0));
    collision_objects.push_back(addBox("s1", 0.0 + 2.0,   0.0+1.6,  0.95,   0.8, 0.28, 0.02, 0.0));
    collision_objects.push_back(addBox("s2", 0.0 + 2.0,   0.0+1.6,  0.62,  0.8, 0.28, 0.02, 0.0));
    collision_objects.push_back(addBox("s3", 0.0 + 2.0,   0.0+1.6,  0.31,  0.8, 0.28, 0.02, 0.0));
    collision_objects.push_back(addBox("s4", 0.0 + 2.0,   0.0+1.6,  0.01,  0.8, 0.28, 0.02, 0.0));
    collision_objects.push_back(addBox("s5", 0.39+ 2.0,   0.0+1.6,  0.67,  0.02,0.28,  1.34, 0.0));
    collision_objects.push_back(addBox("s6",-0.39+ 2.0,   0.0+1.6,  0.67, 0.02,0.28,  1.34, 0.0));
    collision_objects.push_back(addBox("s7", 0.0 + 2.0,  0.13+1.6, 0.67, 0.8, 0.02, 1.34, 0.0));
//    collision_objects.push_back(addBox("target", 2.0, 1.5, 1.0, 0.03, 0.03, 0.1, 0.0));

    planning_scene_interface_.applyCollisionObjects(collision_objects);
}

moveit_msgs::CollisionObject Scene::addBox(std::string name,
                                                 double x, double y, double z,
                                                 double d_x, double d_y, double d_z,
                                                 double yaw){
    moveit_msgs::CollisionObject box;

    // Add the first table where the cube will originally be kept.
    box.id = name;
    box.header.frame_id = frameID_;

    /* Define the primitive and its dimensions. */
    box.primitives.resize(1);
    box.primitives[0].type = box.primitives[0].BOX;
    box.primitives[0].dimensions.resize(3);
    box.primitives[0].dimensions[0] = d_x;
    box.primitives[0].dimensions[1] = d_y;
    box.primitives[0].dimensions[2] = d_z;

    /* Define the pose of the table. */
    box.primitive_poses.resize(1);
    box.primitive_poses[0].position.x = x;
    box.primitive_poses[0].position.y = y;
    box.primitive_poses[0].position.z = z;

    tf::Quaternion q(tf::Vector3(0, 0, 1), yaw);

    box.primitive_poses[0].orientation.w = q.w();
    box.primitive_poses[0].orientation.x = q.x();
    box.primitive_poses[0].orientation.y = q.y();
    box.primitive_poses[0].orientation.z = q.z();

    // END_SUB_TUTORIAL
    box.operation = box.ADD;

    return box;
}

void Scene::updateCollisionScene(){
    std::map<std::string, moveit_msgs::CollisionObject> collision_objects_map = planning_scene_interface_.getObjects();

    for(auto& kv : collision_objects_map){
        planning_scene_->processCollisionObjectMsg(kv.second);
    }
}