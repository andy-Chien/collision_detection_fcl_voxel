#include <pluginlib/class_list_macros.h>
#include "collision_detection_fcl_voxel/collision_detector_fcl_plugin_loader.h"

namespace collision_detection
{
bool CollisionDetectorFCLVoxelPluginLoader::initialize(const planning_scene::PlanningScenePtr& scene, bool exclusive) const
{
  scene->setActiveCollisionDetector(CollisionDetectorAllocatorFCLVoxel::create(), exclusive);
  return true;
}
}  // namespace collision_detection

PLUGINLIB_EXPORT_CLASS(collision_detection::CollisionDetectorFCLVoxelPluginLoader, collision_detection::CollisionPlugin)
