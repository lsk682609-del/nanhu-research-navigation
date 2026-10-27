#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <plan_env/raycast.h>

using namespace std;
// constant parameters

struct MappingParameters {

  /* map properties */
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;  // map range in pos
  Eigen::Vector3i map_voxel_num_;                        // map range in index
  Eigen::Vector3i map_bound_min_idx_, map_bound_max_idx_;
  Eigen::Vector3i map_origin_idx_;
  double resolution_, resolution_inv_;
  string frame_id_;

  /* depth camera intrinsics */
  double cx_, cy_, fx_, fy_;

  /* depth image projection filtering */
  double depth_filter_maxdist_, depth_filter_mindist_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  /* raycasting */
  double max_ray_length_;

  /* visualization */
  double vis_height_, ground_height_;

  /* single-frame perception mode */
  string perception_mode_;
  bool depth_enabled_, semantic_enabled_, fused_enabled_;

  /* mapping sensor input */
  bool need_extrinsic_;
  Eigen::Matrix4d depth_extrinsic_;

  /* semantic drivable-boundary extraction */
  bool boundary_use_ground_plane_fallback_;
  bool boundary_publish_debug_cloud_;
  double semantic_ground_height_, semantic_ground_tolerance_;
  double boundary_max_forward_distance_, boundary_near_ignore_distance_;
  double boundary_center_ignore_half_width_, boundary_max_neighbor_jump_;
  double boundary_seed_roi_width_ratio_, boundary_seed_roi_height_ratio_;
  double projection_obstacle_min_height_, projection_obstacle_max_height_;
  int boundary_pixel_step_, boundary_min_component_area_, boundary_depth_window_size_;

};

// intermediate mapping data for fusion

struct MappingData {
  // main map data, occupancy of each voxel and Euclidean distance

  // Binary single-frame voxels: -1 unknown, 0 free, 100 occupied.
  std::vector<int8_t> occupancy_buffer_;

  // raycast origin and sensor pose data

  Eigen::Vector3d ray_pos_;
  Eigen::Quaterniond ray_q_;
  Eigen::Vector3d base_pos_;
  Eigen::Quaterniond base_q_;

  // depth image data

  cv::Mat depth_image_;
  cv::Mat semantic_mask_;
  int image_cnt_;
  // flags of map state

  bool has_ray_pose_;
  bool has_camera_info_;
  uint32_t camera_info_width_, camera_info_height_;

  // depth image projected point cloud

  vector<Eigen::Vector3d> proj_points_;
  vector<char> proj_points_have_surface_endpoint_;
  int proj_points_cnt;

  // Latest-frame raw depth projection in ROS OccupancyGrid row-major order.
  // This is deliberately independent of semantic ground filtering.
  vector<int8_t> depth_costmap_data_;
  // Sparse semantic boundary, directly rasterized in 2-D without a 3-D wall.
  vector<int8_t> boundary_costmap_data_;
  vector<Eigen::Vector3d> boundary_points_;

  // Per-frame flags for avoiding repeated ray traversal.
  vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_;

  // range of updating grid

  Eigen::Vector3i local_bound_min_, local_bound_max_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap {
public:
  GridMap() {}
  ~GridMap() {}

  enum { INVALID_IDX = -10000 };

  // occupancy map management
  inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  inline int toAddress(const Eigen::Vector3i& id);
  inline int toAddress(int& x, int& y, int& z);
  inline bool isInMap(const Eigen::Vector3d& pos);
  inline bool isInMap(const Eigen::Vector3i& idx);

  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  inline void setOccupied(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3i id);

  inline void boundIndex(Eigen::Vector3i& id);
  inline bool isUnknown(const Eigen::Vector3i& id);
  inline bool isUnknown(const Eigen::Vector3d& pos);
  inline bool isKnownFree(const Eigen::Vector3i& id);
  inline bool isKnownOccupied(const Eigen::Vector3i& id);

  void initMap(rclcpp::Node* node);

  void publishMap();

  void publishDepthCloud();
  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
  inline double getResolution();
  Eigen::Vector3d getOrigin();
  int getVoxelNum();

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;

  // get depth image and sensor pose
  void depthSemanticCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth,
                             const sensor_msgs::msg::Image::ConstSharedPtr& mask);
  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth);
  void processFrame(const sensor_msgs::msg::Image::ConstSharedPtr& depth,
                    const sensor_msgs::msg::Image::ConstSharedPtr& mask);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);

  // update occupancy by raycasting
  void visCallback();

  // main update process
  void projectDepthImage();
  void raycastSingleFrame();
  void updateBoundaryLayer();
  bool projectBoundaryPixel(int u, int v, int inward_du, int inward_dv,
                            Eigen::Vector3d& point_world) const;
  void addBoundaryPolyline(const vector<Eigen::Vector3d>& points);
  void rasterizeBoundarySegment(const Eigen::Vector3d& start,
                                const Eigen::Vector3d& end);
  void markBoundaryCostmapCell(int x, int y);
  void publishBoundaryCloud();
  void publishBoundaryCostmap();
  void publishDepthCostmap();
  void publishFusedCostmap();
  void rebuildCostmapCaches();
  void initializeCostmapMessage(nav_msgs::msg::OccupancyGrid& message) const;
  void resetDepthCostmapData();
  void resetBoundaryData();
  void markDepthCostmapCell(const Eigen::Vector3i& index, int8_t value);
  inline int toAddress2D(int x, int y) const;

  inline int getLocalIndex(int id, int dim) const;
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& ray_pos);
  void updateMapBoundaryFromIndex();
  void resetAllMapData();

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
      sensor_msgs::msg::Image> SyncPolicyDepthSemantic;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyDepthSemantic>>
      SynchronizerDepthSemantic;

  rclcpp::Node* node_{nullptr};
  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> semantic_sub_;
  SynchronizerDepthSemantic sync_depth_semantic_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr boundary_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr boundary_costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr depth_costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr fused_costmap_pub_;
  nav_msgs::msg::OccupancyGrid boundary_costmap_cache_, depth_costmap_cache_, fused_costmap_cache_;
  sensor_msgs::msg::PointCloud2 boundary_cloud_cache_;
  bool boundary_costmap_cache_valid_{false}, boundary_cloud_cache_valid_{false};
  bool depth_costmap_cache_valid_{false}, fused_costmap_cache_valid_{false};
  rclcpp::TimerBase::SharedPtr vis_timer_;

};

/* ============================== definition of inline function
 * ============================== */

inline int GridMap::toAddress(const Eigen::Vector3i& id) {
  return getLocalIndex(id(0), 0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         getLocalIndex(id(1), 1) * mp_.map_voxel_num_(2) + getLocalIndex(id(2), 2);
}

inline int GridMap::toAddress(int& x, int& y, int& z) {
  return getLocalIndex(x, 0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         getLocalIndex(y, 1) * mp_.map_voxel_num_(2) + getLocalIndex(z, 2);
}

inline int GridMap::getLocalIndex(int id, int dim) const {
  int local_id = id % mp_.map_voxel_num_(dim);
  if (local_id < 0) local_id += mp_.map_voxel_num_(dim);
  return local_id;
}

inline int GridMap::toAddress2D(int x, int y) const {
  return getLocalIndex(x, 0) * mp_.map_voxel_num_(1) + getLocalIndex(y, 1);
}

inline void GridMap::boundIndex(Eigen::Vector3i& id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_bound_max_idx_(0)), mp_.map_bound_min_idx_(0));
  id1(1) = max(min(id(1), mp_.map_bound_max_idx_(1)), mp_.map_bound_min_idx_(1));
  id1(2) = max(min(id(2), mp_.map_bound_max_idx_(2)), mp_.map_bound_min_idx_(2));
  id = id1;
}

inline bool GridMap::isUnknown(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < 0;
}

inline bool GridMap::isUnknown(const Eigen::Vector3d& pos) {
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

inline bool GridMap::isKnownFree(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_[adr] == 0;
}

inline bool GridMap::isKnownOccupied(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_[adr] == 100;
}

inline void GridMap::setOccupied(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_[toAddress(id)] = 100;
}

inline void GridMap::setOccupancy(Eigen::Vector3d pos, double occ) {
  if (occ != 1 && occ != 0) {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_[toAddress(id)] = occ > 0.5 ? 100 : 0;
}

inline int GridMap::getOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_[toAddress(id)] == 100 ? 1 : 0;
}

inline int GridMap::getOccupancy(Eigen::Vector3i id) {
  if (!isInMap(id))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] == 100 ? 1 : 0;
}

inline bool GridMap::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}

inline bool GridMap::isInMap(const Eigen::Vector3i& idx) {
  if (idx(0) < mp_.map_bound_min_idx_(0) || idx(1) < mp_.map_bound_min_idx_(1) ||
      idx(2) < mp_.map_bound_min_idx_(2)) {
    return false;
  }
  if (idx(0) > mp_.map_bound_max_idx_(0) || idx(1) > mp_.map_bound_max_idx_(1) ||
      idx(2) > mp_.map_bound_max_idx_(2)) {
    return false;
  }
  return true;
}

inline void GridMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor(pos(i) * mp_.resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_;
}

inline double GridMap::getResolution() { return mp_.resolution_; }

#endif
