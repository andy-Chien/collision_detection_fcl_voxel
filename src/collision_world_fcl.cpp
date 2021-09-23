/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include "collision_detection_fcl_voxel/collision_world_fcl.h"
#include "collision_detection_fcl_voxel/collision_detector_allocator_fcl.h"
#include "collision_detection_fcl_voxel/fcl_compat.h"

#if (MOVEIT_FCL_VERSION >= FCL_VERSION_CHECK(0, 6, 0))
#include <fcl/geometry/geometric_shape_to_BVH_model.h>
#include <fcl/narrowphase/detail/traversal/collision/bvh_collision_traversal_node.h>
#include <fcl/narrowphase/detail/traversal/collision_node.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#else
#include <fcl/shape/geometric_shape_to_BVH_model.h>
#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/collision_node.h>
#endif

#include <boost/bind.hpp>
#include <typeinfo>
#include <XmlRpcException.h>


namespace collision_detection
{
const std::string CollisionDetectorAllocatorFCLVoxel::NAME("FCL_VOXEL");
std::unique_ptr<GvlManager> CollisionWorldFCLVoxel::gvl_manager_ = std::make_unique<GvlManager>();
int CollisionWorldFCLVoxel::check_time = 0;
CollisionWorldFCLVoxel::CollisionWorldFCLVoxel() : CollisionWorld()
{
  auto m = new fcl::DynamicAABBTreeCollisionManagerd();
  // m->tree_init_level = 2;
  manager_.reset(m);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldFCLVoxel::notifyObjectChange, this, _1, _2));
}

CollisionWorldFCLVoxel::CollisionWorldFCLVoxel(const WorldPtr& world) : CollisionWorld(world)
{
  auto m = new fcl::DynamicAABBTreeCollisionManagerd();
  // m->tree_init_level = 2;
  manager_.reset(m);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldFCLVoxel::notifyObjectChange, this, _1, _2));
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

CollisionWorldFCLVoxel::CollisionWorldFCLVoxel(const CollisionWorldFCLVoxel& other, const WorldPtr& world)
  : CollisionWorld(other, world)
{
  auto m = new fcl::DynamicAABBTreeCollisionManagerd();
  // m->tree_init_level = 2;
  manager_.reset(m);

  fcl_objs_ = other.fcl_objs_;
  for (auto& fcl_obj : fcl_objs_)
    fcl_obj.second.registerTo(manager_.get());
  // manager_->update();

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldFCLVoxel::notifyObjectChange, this, _1, _2));
}

CollisionWorldFCLVoxel::~CollisionWorldFCLVoxel()
{
  getWorld()->removeObserver(observer_handle_);
}

void CollisionWorldFCLVoxel::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionRobot& robot, const robot_state::RobotState& state) const
{
  checkRobotCollisionHelper(req, res, robot, state, nullptr);
}

void CollisionWorldFCLVoxel::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionRobot& robot, const robot_state::RobotState& state,
                                            const AllowedCollisionMatrix& acm) const
{
  checkRobotCollisionHelper(req, res, robot, state, &acm);
}

void CollisionWorldFCLVoxel::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionRobot& robot, const robot_state::RobotState& state1,
                                            const robot_state::RobotState& state2) const
{
  ROS_ERROR_NAMED("collision_detection.fcl", "FCL continuous collision checking not yet implemented");
}

void CollisionWorldFCLVoxel::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionRobot& robot, const robot_state::RobotState& state1,
                                            const robot_state::RobotState& state2,
                                            const AllowedCollisionMatrix& acm) const
{
  ROS_ERROR_NAMED("collision_detection.fcl", "FCL continuous collision checking not yet implemented");
}

void CollisionWorldFCLVoxel::checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                  const CollisionRobot& robot, const robot_state::RobotState& state,
                                                  const AllowedCollisionMatrix* acm) const
{
  check_time += 1;
  ros::Time begin = ros::Time::now();
  const CollisionRobotFCLVoxel& robot_fcl = dynamic_cast<const CollisionRobotFCLVoxel&>(robot);
  FCLObject fcl_obj;
  robot_fcl.constructFCLObject(state, fcl_obj);

  CollisionData cd(&req, &res, acm);
  cd.enableGroup(robot.getRobotModel());
  for (std::size_t i = 0; !cd.done_ && i < fcl_obj.collision_objects_.size(); ++i)
    manager_->collide(fcl_obj.collision_objects_[i].get(), &cd, &collisionCallback);

  if (req.distance)
  {
    DistanceRequest dreq;
    DistanceResult dres;

    dreq.group_name = req.group_name;
    dreq.acm = acm;
    dreq.enableGroup(robot.getRobotModel());
    distanceRobot(dreq, dres, robot, state);
    res.distance = dres.minimum_distance.distance;
  }
  ros::Time end = ros::Time::now();
  std::cout << "times = " << check_time << ", check collisions spend " << (end - begin).toSec() * 1000 << "ms" <<std::endl;
}

void CollisionWorldFCLVoxel::checkWorldCollision(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionWorld& other_world) const
{
  checkWorldCollisionHelper(req, res, other_world, nullptr);
}

void CollisionWorldFCLVoxel::checkWorldCollision(const CollisionRequest& req, CollisionResult& res,
                                            const CollisionWorld& other_world, const AllowedCollisionMatrix& acm) const
{
  checkWorldCollisionHelper(req, res, other_world, &acm);
}

void CollisionWorldFCLVoxel::checkWorldCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                  const CollisionWorld& other_world,
                                                  const AllowedCollisionMatrix* acm) const
{
  const CollisionWorldFCLVoxel& other_fcl_world = dynamic_cast<const CollisionWorldFCLVoxel&>(other_world);
  CollisionData cd(&req, &res, acm);
  manager_->collide(other_fcl_world.manager_.get(), &cd, &collisionCallback);

  if (req.distance)
  {
    DistanceRequest dreq;
    DistanceResult dres;

    dreq.group_name = req.group_name;
    dreq.acm = acm;
    distanceWorld(dreq, dres, other_world);
    res.distance = dres.minimum_distance.distance;
  }
}

void CollisionWorldFCLVoxel::constructFCLObject(const World::Object* obj, FCLObject& fcl_obj) const
{
  for (std::size_t i = 0; i < obj->shapes_.size(); ++i)
  {
    FCLGeometryConstPtr g = createCollisionGeometry(obj->shapes_[i], obj);
    if (g)
    {
      auto co = new fcl::CollisionObjectd(g->collision_geometry_, transform2fcl(obj->shape_poses_[i]));
      fcl_obj.collision_objects_.push_back(FCLCollisionObjectPtr(co));
      fcl_obj.collision_geometry_.push_back(g);
    }
  }
}

void CollisionWorldFCLVoxel::updateFCLObject(const std::string& id)
{
  // remove FCL objects that correspond to this object
  auto jt = fcl_objs_.find(id);
  if (jt != fcl_objs_.end())
  {
    jt->second.unregisterFrom(manager_.get());
    jt->second.clear();
  }

  // check to see if we have this object
  auto it = getWorld()->find(id);
  if (it != getWorld()->end())
  {
    // construct FCL objects that correspond to this object
    if (jt != fcl_objs_.end())
    {
      constructFCLObject(it->second.get(), jt->second);
      jt->second.registerTo(manager_.get());
    }
    else
    {
      constructFCLObject(it->second.get(), fcl_objs_[id]);
      fcl_objs_[id].registerTo(manager_.get());
    }
  }
  else
  {
    if (jt != fcl_objs_.end())
      fcl_objs_.erase(jt);
  }

  // manager_->update();
}

void CollisionWorldFCLVoxel::setWorld(const WorldPtr& world)
{
  if (world == getWorld())
    return;

  // turn off notifications about old world
  getWorld()->removeObserver(observer_handle_);

  // clear out objects from old world
  manager_->clear();
  fcl_objs_.clear();
  cleanCollisionGeometryCache();

  CollisionWorld::setWorld(world);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldFCLVoxel::notifyObjectChange, this, _1, _2));

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

void CollisionWorldFCLVoxel::notifyObjectChange(const ObjectConstPtr& obj, World::Action action)
{
  if (action == World::DESTROY)
  {
    auto it = fcl_objs_.find(obj->id_);
    if (it != fcl_objs_.end())
    {
      it->second.unregisterFrom(manager_.get());
      it->second.clear();
      fcl_objs_.erase(it);
    }
    cleanCollisionGeometryCache();
  }
  else
  {
    updateFCLObject(obj->id_);
    if (action & (World::DESTROY | World::REMOVE_SHAPE))
      cleanCollisionGeometryCache();
  }
}

void CollisionWorldFCLVoxel::distanceRobot(const DistanceRequest& req, DistanceResult& res, const CollisionRobot& robot,
                                      const robot_state::RobotState& state) const
{
  const CollisionRobotFCLVoxel& robot_fcl = dynamic_cast<const CollisionRobotFCLVoxel&>(robot);
  FCLObject fcl_obj;
  robot_fcl.constructFCLObject(state, fcl_obj);

  DistanceData drd(&req, &res);
  for (std::size_t i = 0; !drd.done && i < fcl_obj.collision_objects_.size(); ++i)
    manager_->distance(fcl_obj.collision_objects_[i].get(), &drd, &distanceCallback);
}

void CollisionWorldFCLVoxel::distanceWorld(const DistanceRequest& req, DistanceResult& res, const CollisionWorld& world) const
{
  const CollisionWorldFCLVoxel& other_fcl_world = dynamic_cast<const CollisionWorldFCLVoxel&>(world);
  DistanceData drd(&req, &res);
  manager_->distance(other_fcl_world.manager_.get(), &drd, &distanceCallback);
}

GvlManager::GvlManager()
{
  this->voxelInit();
}
GvlManager::~GvlManager()
{

}
void GvlManager::voxelInit()
{
  Vector3ui map_dim(256, 256, 256);
  map_dimensions =  map_dim;
  voxel_side_length = 0.01f; // 1 cm voxel size

  icl_core::logging::initialize();
  /*
   * First, we generate an API class, which defines the
   * volume of our space and the resolution.
   * Be careful here! The size is limited by the memory
   * of your GPU. Even if an empty Octree is small, a
   * Voxelmap will always require the full memory.
   */
  icl_core::logging::initialize();

  voxel_side_length = icl_core::config::paramOptDefault<float>("voxel_side_length", 0.01f);

  const Vector3f camera_offsets(-0.3, 0.7, 0.5);
  float roll = icl_core::config::paramOptDefault<float>("roll", 0.0f) * 3.141592f / 180.0f;
  float pitch = icl_core::config::paramOptDefault<float>("pitch", 90.0f) * 3.141592f / 180.0f;
  float yaw = icl_core::config::paramOptDefault<float>("yaw", 0.0f) * 3.141592f / 180.0f;
  tf = Matrix4f::createFromRotationAndTranslation(Matrix3f::createFromRPY(roll, pitch, yaw), camera_offsets);

  // std::string point_cloud_topic = icl_core::config::paramOptDefault<std::string>("points-topic", "/camera/depth/color/points");
  // LOGGING_INFO(Gpu_voxels, "DistanceROSDemo start. Point-cloud topic: " << point_cloud_topic << endl);

  gvl = GpuVoxels::getInstance();
  gvl->initialize(map_dimensions.x, map_dimensions.y, map_dimensions.z, voxel_side_length); // ==> 200 Voxels, each one is 1 cm in size so the map represents 20x20x20 centimeter

  //Vis Helper
  gvl->addPrimitives(primitive_array::ePRIM_SPHERE, "measurementPoints");

  // Add a map:
  gvl->addMap(MT_PROBAB_VOXELMAP, "myObjectVoxelmap");
  gvl->addMap(MT_PROBAB_VOXELMAP, "myHandVoxellist");
  gvl->addMap(MT_COUNTING_VOXELLIST, "countingVoxelList");
  countingVoxelList = boost::dynamic_pointer_cast<gpu_voxels::voxellist::CountingVoxelList>(gvl->getMap("countingVoxelList"));

  // And a robot, generated from a ROS URDF file:
  ros_hn.getParam("/robot_description_voxels/urdf_path", urdf_path);
  ros_hn.getParam("/move_group/sensors", sensors_param);
  if (sensors_param[0].hasMember("point_cloud_topic"))
  {
    std::string point_cloud_topic = static_cast<const std::string&>(sensors_param[0]["point_cloud_topic"]);
    point_sub = ros_hn.subscribe<pcl::PointCloud<pcl::PointXYZ> >(point_cloud_topic, 1, &GvlManager::pointCloudCallback, this);
    LOGGING_INFO(Gpu_voxels, "Gpu_voxels checker start to subcribe Point-cloud topic: " << point_cloud_topic << endl);
  }
  gvl->addRobot("myUrdfRobot", urdf_path, false);
  // update the robot joints:
  gvl->setRobotConfiguration("myUrdfRobot", myRobotJointValues);
  // insert the robot into the map:
  gvl->insertRobotIntoMap("myUrdfRobot", "myHandVoxellist", eBVM_OCCUPIED);
}

void GvlManager::pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
  std::vector<Vector3f> point_data;
  point_data.resize(msg->points.size());

  for (uint32_t i = 0; i < msg->points.size(); i++)
  {
    point_data[i].x = msg->points[i].x;
    point_data[i].y = msg->points[i].y;
    point_data[i].z = msg->points[i].z;
  }

  //my_point_cloud.add(point_data);
  my_point_cloud.update(point_data);

  // transform new pointcloud to world coordinates
  my_point_cloud.transformSelf(&tf);
  countingVoxelList->clearMap();
  countingVoxelList->insertPointCloud(my_point_cloud, eBVM_OCCUPIED);
  
  // LOGGING_INFO(Gpu_voxels, "DistanceROSDemo camera callback. PointCloud size: " << msg->points.size() << endl);
}

void GvlManager::jointStateCallback(const robot_state::RobotState& state)
{
  // 0.09 ~ 1.2 ms  avg 0.1 ms
  gvl->clearMap("myHandVoxellist");
  for (int i=0; i<state.getVariableCount(); i++)
  {
    myRobotJointValues[state.getVariableNames()[i]] = state.getVariablePositions()[i];
  }
  // update the robot joints:  spend 0.15 ~ 2 ms avg 0.16 ms
  gvl->setRobotConfiguration("myUrdfRobot", myRobotJointValues);
  // insert the robot into the map: spend 0.02 ~ 0.04 ms
  gvl->insertRobotIntoMap("myUrdfRobot", "myHandVoxellist", eBVM_OCCUPIED);
}

}  // end of namespace collision_detection
