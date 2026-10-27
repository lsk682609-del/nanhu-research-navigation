#include "plan_env/grid_map.h"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
template <typename T>
void load_parameter(rclcpp::Node *node, const std::string &name, T &value, const T &default_value)
{
  if (!node->has_parameter(name))
    node->declare_parameter<T>(name, default_value);
  node->get_parameter(name, value);
}
}  // namespace

void GridMap::initMap(rclcpp::Node *node)
{
  node_ = node;

  /* get parameter */
  double x_size, y_size, z_size;
  std::string depth_topic;
  std::string camera_info_topic;
  std::string mask_topic;
  std::string depth_costmap_topic;
  std::string boundary_costmap_topic;
  std::string boundary_cloud_topic;
  std::string fused_costmap_topic;
  load_parameter(node_, "perception.mode", mp_.perception_mode_, string("fused"));
  if (mp_.perception_mode_ != "semantic_only" &&
      mp_.perception_mode_ != "depth_only" && mp_.perception_mode_ != "fused")
    throw std::invalid_argument(
        "perception.mode must be 'semantic_only', 'depth_only', or 'fused'");
  mp_.depth_enabled_ = mp_.perception_mode_ != "semantic_only";
  mp_.semantic_enabled_ = mp_.perception_mode_ != "depth_only";
  mp_.fused_enabled_ = mp_.perception_mode_ == "fused";

  load_parameter(node_, "topics.depth_image", depth_topic, string("/camera/depth/image_raw"));
  load_parameter(node_, "topics.camera_info", camera_info_topic, string("/camera/color/camera_info"));
  load_parameter(node_, "topics.drivable_mask", mask_topic,
                 string("/twinlitenet/drivable_mask"));
  load_parameter(node_, "topics.depth_costmap", depth_costmap_topic,
                 string("/grid_map/depth_costmap"));
  load_parameter(node_, "topics.boundary_costmap", boundary_costmap_topic,
                 string("/grid_map/drivable_boundary_costmap"));
  load_parameter(node_, "topics.boundary_cloud", boundary_cloud_topic,
                 string("/grid_map/drivable_boundary_cloud"));
  load_parameter(node_, "topics.fused_costmap", fused_costmap_topic,
                 string("/grid_map/fused_costmap"));
  if (depth_topic.empty() || camera_info_topic.empty() ||
      (mp_.semantic_enabled_ && mask_topic.empty()) ||
      (mp_.depth_enabled_ && depth_costmap_topic.empty()) ||
      (mp_.semantic_enabled_ &&
       (boundary_costmap_topic.empty() || boundary_cloud_topic.empty())) ||
      (mp_.fused_enabled_ && fused_costmap_topic.empty()))
    throw std::invalid_argument("input topic names must not be empty");

  load_parameter(node_, "grid_map.resolution", mp_.resolution_, -1.0);
  load_parameter(node_, "grid_map.size_x", x_size, -1.0);
  load_parameter(node_, "grid_map.size_y", y_size, -1.0);
  load_parameter(node_, "grid_map.size_z", z_size, -1.0);

  load_parameter(node_, "grid_map.fx", mp_.fx_, -1.0);
  load_parameter(node_, "grid_map.fy", mp_.fy_, -1.0);
  load_parameter(node_, "grid_map.cx", mp_.cx_, -1.0);
  load_parameter(node_, "grid_map.cy", mp_.cy_, -1.0);

  load_parameter(node_, "grid_map.depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  load_parameter(node_, "grid_map.depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  load_parameter(node_, "grid_map.depth_filter_margin", mp_.depth_filter_margin_, -1);
  load_parameter(node_, "grid_map.k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  load_parameter(node_, "grid_map.skip_pixel", mp_.skip_pixel_, -1);

  load_parameter(node_, "grid_map.max_ray_length", mp_.max_ray_length_, -0.1);

  load_parameter(node_, "grid_map.vis_height", mp_.vis_height_, 0.3);

  load_parameter(node_, "mapping.frame_id", mp_.frame_id_, string("base_link"));
  load_parameter(node_, "grid_map.ground_height", mp_.ground_height_, 0.0);
  if (mp_.frame_id_.empty())
    throw std::invalid_argument("mapping.frame_id must not be empty");

  load_parameter(node_, "grid_map.need_extrinsic", mp_.need_extrinsic_, true);

  load_parameter(node_, "semantic.ground_height", mp_.semantic_ground_height_, 0.0);
  load_parameter(node_, "semantic.ground_tolerance", mp_.semantic_ground_tolerance_, 0.25);
  load_parameter(node_, "boundary.max_forward_distance",
                 mp_.boundary_max_forward_distance_, 6.0);
  load_parameter(node_, "boundary.near_ignore_distance",
                 mp_.boundary_near_ignore_distance_, 0.5);
  load_parameter(node_, "boundary.center_ignore_half_width",
                 mp_.boundary_center_ignore_half_width_, 0.4);
  load_parameter(node_, "boundary.pixel_step", mp_.boundary_pixel_step_, 4);
  load_parameter(node_, "boundary.min_component_area",
                 mp_.boundary_min_component_area_, 1000);
  load_parameter(node_, "boundary.depth_window_size",
                 mp_.boundary_depth_window_size_, 5);
  load_parameter(node_, "boundary.max_neighbor_jump",
                 mp_.boundary_max_neighbor_jump_, 0.75);
  load_parameter(node_, "boundary.seed_roi_width_ratio",
                 mp_.boundary_seed_roi_width_ratio_, 0.25);
  load_parameter(node_, "boundary.seed_roi_height_ratio",
                 mp_.boundary_seed_roi_height_ratio_, 0.15);
  load_parameter(node_, "boundary.use_ground_plane_fallback",
                 mp_.boundary_use_ground_plane_fallback_, true);
  load_parameter(node_, "boundary.publish_debug_cloud",
                 mp_.boundary_publish_debug_cloud_, true);
  load_parameter(node_, "projection.obstacle_min_height", mp_.projection_obstacle_min_height_, 0.10);
  load_parameter(node_, "projection.obstacle_max_height", mp_.projection_obstacle_max_height_, 1.50);

  std::vector<double> depth_extrinsic{
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0};
  load_parameter(node_, "grid_map.depth_extrinsic", depth_extrinsic, depth_extrinsic);
  if (depth_extrinsic.size() != 16)
    throw std::invalid_argument("grid_map.depth_extrinsic must contain 16 row-major values");
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col)
      mp_.depth_extrinsic_(row, col) = depth_extrinsic[row * 4 + col];
      
  if (mp_.resolution_ <= 0.0 || x_size <= 0.0 || y_size <= 0.0 || z_size <= 0.0 ||
      mp_.fx_ <= 0.0 || mp_.fy_ <= 0.0 || mp_.skip_pixel_ <= 0 ||
      mp_.k_depth_scaling_factor_ <= 0.0)
    throw std::invalid_argument("invalid grid_map size, resolution, camera intrinsics, or depth scaling parameter");
  if (mp_.semantic_ground_tolerance_ < 0.0 ||
      mp_.boundary_max_forward_distance_ <= 0.0 ||
      mp_.boundary_near_ignore_distance_ < 0.0 ||
      mp_.boundary_center_ignore_half_width_ < 0.0 ||
      mp_.boundary_pixel_step_ <= 0 || mp_.boundary_min_component_area_ <= 0 ||
      mp_.boundary_depth_window_size_ <= 0 ||
      mp_.boundary_depth_window_size_ % 2 == 0 ||
      mp_.boundary_max_neighbor_jump_ <= 0.0 ||
      mp_.boundary_seed_roi_width_ratio_ <= 0.0 ||
      mp_.boundary_seed_roi_width_ratio_ > 1.0 ||
      mp_.boundary_seed_roi_height_ratio_ <= 0.0 ||
      mp_.boundary_seed_roi_height_ratio_ > 1.0)
    throw std::invalid_argument("invalid semantic ground or boundary extraction parameter");

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;
  posToIndex(mp_.map_origin_, mp_.map_bound_min_idx_);
  mp_.map_bound_max_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  mp_.map_origin_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ / 2;
  updateMapBoundaryFromIndex();

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  if (mp_.depth_enabled_)
  {
    md_.occupancy_buffer_ = vector<int8_t>(buffer_size, -1);
    md_.flag_rayend_ = vector<char>(buffer_size, -1);
    md_.flag_traverse_ = vector<char>(buffer_size, -1);
  }

  const int costmap_buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1);
  if (mp_.depth_enabled_)
    md_.depth_costmap_data_ = vector<int8_t>(costmap_buffer_size, -1);
  if (mp_.semantic_enabled_)
    md_.boundary_costmap_data_ = vector<int8_t>(costmap_buffer_size, -1);

  md_.raycast_num_ = 0;

  if (mp_.depth_enabled_)
  {
    md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
    md_.proj_points_have_surface_endpoint_.resize(md_.proj_points_.size(), 0);
  }
  md_.proj_points_cnt = 0;

  /* init callback */
  depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>();
  depth_sub_->subscribe(node_, depth_topic, rmw_qos_profile_sensor_data);
  if (mp_.semantic_enabled_)
  {
    semantic_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>();
    semantic_sub_->subscribe(node_, mask_topic, rmw_qos_profile_sensor_data);
    sync_depth_semantic_.reset(new message_filters::Synchronizer<SyncPolicyDepthSemantic>(
        SyncPolicyDepthSemantic(50), *depth_sub_, *semantic_sub_));
    sync_depth_semantic_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.08));
    sync_depth_semantic_->registerCallback(std::bind(&GridMap::depthSemanticCallback,
        this, std::placeholders::_1, std::placeholders::_2));
  }
  else
  {
    depth_sub_->registerCallback(
        std::bind(&GridMap::depthCallback, this, std::placeholders::_1));
  }
  camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, rclcpp::SensorDataQoS(),
      std::bind(&GridMap::cameraInfoCallback, this, std::placeholders::_1));

  vis_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                        std::bind(&GridMap::visCallback, this));

  if (mp_.depth_enabled_)
  {
    map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "grid_map/occupancy", rclcpp::SensorDataQoS());
    depth_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "grid_map/depth_cloud", rclcpp::SensorDataQoS());
    depth_costmap_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        depth_costmap_topic, rclcpp::QoS(1).transient_local().reliable());
  }
  if (mp_.semantic_enabled_)
  {
    boundary_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        boundary_cloud_topic, rclcpp::SensorDataQoS());
    boundary_costmap_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        boundary_costmap_topic, rclcpp::QoS(1).transient_local().reliable());
  }
  if (mp_.fused_enabled_)
    fused_costmap_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        fused_costmap_topic, rclcpp::QoS(1).transient_local().reliable());

  md_.has_ray_pose_ = false;
  md_.has_camera_info_ = false;
  md_.camera_info_width_ = 0;
  md_.camera_info_height_ = 0;
  md_.image_cnt_ = 0;
  md_.ray_pos_.setZero();
  md_.base_pos_.setZero();
  md_.ray_q_ = Eigen::Quaterniond::Identity();
  md_.base_q_ = Eigen::Quaterniond::Identity();

  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;

  RCLCPP_INFO(node_->get_logger(), "perception mode: %s, single-frame output: %s",
              mp_.perception_mode_.c_str(), mp_.frame_id_.c_str());
  RCLCPP_INFO(node_->get_logger(),
              "input topics: depth=%s camera_info=%s drivable_mask=%s",
              depth_topic.c_str(), camera_info_topic.c_str(),
              mp_.semantic_enabled_ ? mask_topic.c_str() : "disabled");
  RCLCPP_INFO(node_->get_logger(),
              "costmaps: semantic=%s depth=%s fused=%s",
              mp_.semantic_enabled_ ? boundary_costmap_topic.c_str() : "disabled",
              mp_.depth_enabled_ ? depth_costmap_topic.c_str() : "disabled",
              mp_.fused_enabled_ ? fused_costmap_topic.c_str() : "disabled");

}

void GridMap::updateMapBoundaryFromIndex()
{
  mp_.map_bound_min_idx_ = mp_.map_origin_idx_ - mp_.map_voxel_num_ / 2;
  mp_.map_bound_max_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  mp_.map_min_boundary_ = mp_.map_bound_min_idx_.cast<double>() * mp_.resolution_;
  mp_.map_max_boundary_ = (mp_.map_bound_max_idx_.cast<double>() + Eigen::Vector3d::Ones()) * mp_.resolution_;
  mp_.map_origin_ = mp_.map_min_boundary_;
}

void GridMap::resetAllMapData()
{
  std::fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(), -1);
  std::fill(md_.flag_rayend_.begin(), md_.flag_rayend_.end(), -1);
  std::fill(md_.flag_traverse_.begin(), md_.flag_traverse_.end(), -1);
  resetDepthCostmapData();
  resetBoundaryData();
  boundary_costmap_cache_valid_ = false;
  boundary_cloud_cache_valid_ = false;
  depth_costmap_cache_valid_ = false;
  fused_costmap_cache_valid_ = false;
}

void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;

  const int projected_capacity =
      ((rows + mp_.skip_pixel_ - 1) / mp_.skip_pixel_) *
      ((cols + mp_.skip_pixel_ - 1) / mp_.skip_pixel_);
  if (static_cast<int>(md_.proj_points_.size()) < projected_capacity)
  {
    md_.proj_points_.resize(projected_capacity);
    md_.proj_points_have_surface_endpoint_.resize(projected_capacity, 0);
  }

  double depth;

  Eigen::Matrix3d sensor_r = md_.ray_q_.toRotationMatrix();

  // cout << "rotate: " << md_.ray_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << md_.ray_pos_ << std::endl;

  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

  for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
  {
    row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

    for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_; u += mp_.skip_pixel_)
    {
      const uint16_t raw_depth = *row_ptr;
      depth = raw_depth * inv_factor;
      row_ptr = row_ptr + mp_.skip_pixel_;
      bool has_surface_endpoint = raw_depth != 0;

      if (raw_depth == 0)
      {
        depth = mp_.max_ray_length_ + 0.1;
      }
      else if (depth < mp_.depth_filter_mindist_)
      {
        continue;
      }
      else if (depth > mp_.depth_filter_maxdist_)
      {
        has_surface_endpoint = false;
        depth = mp_.max_ray_length_ + 0.1;
      }

      // project to world frame
      pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
      pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
      pt_cur(2) = depth;

      pt_world = sensor_r * pt_cur + md_.ray_pos_;
      // if (!isInMap(pt_world)) {
      //   pt_world = closetPointInMap(pt_world, md_.ray_pos_);
      // }

      const int point_index = md_.proj_points_cnt++;
      md_.proj_points_[point_index] = pt_world;
      md_.proj_points_have_surface_endpoint_[point_index] = has_surface_endpoint ? 1 : 0;
    }
  }
}

void GridMap::resetDepthCostmapData()
{
  std::fill(md_.depth_costmap_data_.begin(), md_.depth_costmap_data_.end(), -1);
}

void GridMap::resetBoundaryData()
{
  std::fill(md_.boundary_costmap_data_.begin(), md_.boundary_costmap_data_.end(), -1);
  md_.boundary_points_.clear();
  boundary_cloud_cache_valid_ = false;
}

void GridMap::markDepthCostmapCell(const Eigen::Vector3i& index, int8_t value)
{
  if (!isInMap(index))
    return;
  const int min_z = std::max(mp_.map_bound_min_idx_(2), static_cast<int>(std::floor(
      (mp_.semantic_ground_height_ + mp_.projection_obstacle_min_height_) * mp_.resolution_inv_)));
  const int max_z = std::min(mp_.map_bound_max_idx_(2), static_cast<int>(std::floor(
      (mp_.semantic_ground_height_ + mp_.projection_obstacle_max_height_) * mp_.resolution_inv_)));
  if (index.z() < min_z || index.z() > max_z)
    return;

  const size_t mx = index.x() - mp_.map_bound_min_idx_(0);
  const size_t my = index.y() - mp_.map_bound_min_idx_(1);
  const size_t address = my * mp_.map_voxel_num_(0) + mx;
  if (value == 100 || md_.depth_costmap_data_[address] != 100)
    md_.depth_costmap_data_[address] = value;
}

void GridMap::raycastSingleFrame()
{
  resetDepthCostmapData();
  ++md_.raycast_num_;

  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);
  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  const Eigen::Vector3d half(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    Eigen::Vector3d endpoint = md_.proj_points_[i];
    bool has_occupied_endpoint = md_.proj_points_have_surface_endpoint_[i] != 0;

    if (!isInMap(endpoint))
    {
      endpoint = closetPointInMap(endpoint, md_.ray_pos_);
      has_occupied_endpoint = false;
    }
    double length = (endpoint - md_.ray_pos_).norm();
    if (length > mp_.max_ray_length_)
    {
      endpoint = (endpoint - md_.ray_pos_) / length * mp_.max_ray_length_ + md_.ray_pos_;
      has_occupied_endpoint = false;
    }

    Eigen::Vector3i endpoint_index;
    posToIndex(endpoint, endpoint_index);
    int endpoint_address = INVALID_IDX;
    if (isInMap(endpoint_index))
    {
      endpoint_address = toAddress(endpoint_index);
      markDepthCostmapCell(endpoint_index, has_occupied_endpoint ? 100 : 0);
      if (has_occupied_endpoint)
        md_.occupancy_buffer_[endpoint_address] = 100;
      else if (md_.occupancy_buffer_[endpoint_address] != 100)
        md_.occupancy_buffer_[endpoint_address] = 0;
    }

    max_x = std::max(max_x, endpoint.x());
    max_y = std::max(max_y, endpoint.y());
    max_z = std::max(max_z, endpoint.z());
    min_x = std::min(min_x, endpoint.x());
    min_y = std::min(min_y, endpoint.y());
    min_z = std::min(min_z, endpoint.z());

    if (endpoint_address != INVALID_IDX)
    {
      if (md_.flag_rayend_[endpoint_address] == md_.raycast_num_)
        continue;
      md_.flag_rayend_[endpoint_address] = md_.raycast_num_;
    }

    raycaster.setInput(endpoint / mp_.resolution_, md_.ray_pos_ / mp_.resolution_);
    while (raycaster.step(ray_pt))
    {
      const Eigen::Vector3d position = (ray_pt + half) * mp_.resolution_;
      Eigen::Vector3i index;
      posToIndex(position, index);
      if (!isInMap(index))
        continue;

      const int address = toAddress(index);
      markDepthCostmapCell(index, 0);
      // Deterministic single-frame rule: free rays never overwrite an occupied endpoint.
      if (md_.occupancy_buffer_[address] != 100)
        md_.occupancy_buffer_[address] = 0;

      if (md_.flag_traverse_[address] == md_.raycast_num_)
        break;
      md_.flag_traverse_[address] = md_.raycast_num_;
    }
  }

  min_x = std::min(min_x, md_.ray_pos_.x());
  min_y = std::min(min_y, md_.ray_pos_.y());
  min_z = std::min(min_z, md_.ray_pos_.z());
  max_x = std::max(max_x, md_.ray_pos_.x());
  max_y = std::max(max_y, md_.ray_pos_.y());
  max_z = std::max({max_z, md_.ray_pos_.z(), mp_.ground_height_});
  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &ray_pos)
{
  Eigen::Vector3d diff = pt - ray_pos;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - ray_pos;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - ray_pos;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return ray_pos + (min_t - 1e-3) * diff;
}

void GridMap::visCallback()
{
  publishMap();
  publishDepthCloud();
  publishDepthCostmap();
  publishBoundaryCostmap();
  publishBoundaryCloud();
  publishFusedCostmap();
}

void GridMap::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info)
{
  if (info->k[0] <= 0.0 || info->k[4] <= 0.0)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "received invalid aligned-depth CameraInfo intrinsics");
    return;
  }
  mp_.fx_ = info->k[0];
  mp_.fy_ = info->k[4];
  mp_.cx_ = info->k[2];
  mp_.cy_ = info->k[5];
  md_.camera_info_width_ = info->width;
  md_.camera_info_height_ = info->height;
  md_.has_camera_info_ = true;
}

void GridMap::depthSemanticCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &depth,
    const sensor_msgs::msg::Image::ConstSharedPtr &mask)
{
  processFrame(depth, mask);
}

void GridMap::depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &depth)
{
  processFrame(depth, nullptr);
}

void GridMap::processFrame(
    const sensor_msgs::msg::Image::ConstSharedPtr &img,
    const sensor_msgs::msg::Image::ConstSharedPtr &mask)
{
  if (!md_.has_camera_info_)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "waiting for aligned color CameraInfo");
    return;
  }
  /* get depth image */
  if (img->encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
      img->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "unsupported depth encoding '%s'; expected 16UC1 or 32FC1",
                         img->encoding.c_str());
    return;
  }
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  if (md_.has_camera_info_ &&
      (md_.camera_info_width_ != img->width || md_.camera_info_height_ != img->height))
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "aligned depth (%ux%u) does not match color CameraInfo (%ux%u)",
                         img->width, img->height, md_.camera_info_width_, md_.camera_info_height_);
    return;
  }

  if (mp_.semantic_enabled_)
  {
    if (!mask)
    {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                            "semantic boundary is enabled but no drivable mask was supplied");
      return;
    }
    if (mask->encoding != sensor_msgs::image_encodings::MONO8)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "unsupported semantic encoding '%s'; expected mono8",
                           mask->encoding.c_str());
      return;
    }
    cv_bridge::CvImageConstPtr semantic_ptr = cv_bridge::toCvShare(
        mask, sensor_msgs::image_encodings::MONO8);
    if (semantic_ptr->image.size() != md_.depth_image_.size())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "aligned depth (%dx%d) and semantic mask (%dx%d) sizes differ",
                           md_.depth_image_.cols, md_.depth_image_.rows,
                           semantic_ptr->image.cols, semantic_ptr->image.rows);
      return;
    }
    semantic_ptr->image.copyTo(md_.semantic_mask_);
  }
  else
  {
    md_.semantic_mask_.release();
  }

  resetAllMapData();
  md_.raycast_num_ = 0;
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;

  const Eigen::Quaterniond base_q = Eigen::Quaterniond::Identity();
  const Eigen::Vector3d base_pos = Eigen::Vector3d::Zero();
  Eigen::Quaterniond ray_q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d ray_pos = Eigen::Vector3d::Zero();
  if (mp_.need_extrinsic_)
  {
    ray_pos = mp_.depth_extrinsic_.block<3, 1>(0, 3);
    ray_q = Eigen::Quaterniond(mp_.depth_extrinsic_.block<3, 3>(0, 0));
    ray_q.normalize();
  }

  md_.ray_pos_ = ray_pos;
  md_.ray_q_ = ray_q;
  md_.base_pos_ = base_pos;
  md_.base_q_ = base_q;
  if (isInMap(md_.ray_pos_))
  {
    md_.has_ray_pose_ = true;
    if (mp_.depth_enabled_)
    {
      projectDepthImage();
      if (md_.proj_points_cnt > 0)
        raycastSingleFrame();
    }
    if (mp_.semantic_enabled_)
      updateBoundaryLayer();
    rebuildCostmapCaches();
  }
  else
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                          "configured camera origin lies outside the local map");
  }
}

bool GridMap::projectBoundaryPixel(int u, int v, int inward_du, int inward_dv,
                                   Eigen::Vector3d& point_world) const
{
  const int radius = mp_.boundary_depth_window_size_ / 2;
  const int center_u = u + inward_du * radius;
  const int center_v = v + inward_dv * radius;
  std::vector<uint16_t> samples;
  samples.reserve(mp_.boundary_depth_window_size_ * mp_.boundary_depth_window_size_);
  for (int dv = -radius; dv <= radius; ++dv)
    for (int du = -radius; du <= radius; ++du)
    {
      const int sample_u = center_u + du;
      const int sample_v = center_v + dv;
      if (sample_u < 0 || sample_u >= md_.depth_image_.cols ||
          sample_v < 0 || sample_v >= md_.depth_image_.rows ||
          md_.semantic_mask_.at<uint8_t>(sample_v, sample_u) == 0)
        continue;
      const uint16_t raw_depth = md_.depth_image_.at<uint16_t>(sample_v, sample_u);
      const double depth = raw_depth / mp_.k_depth_scaling_factor_;
      if (raw_depth != 0 && depth >= mp_.depth_filter_mindist_ &&
          depth <= mp_.depth_filter_maxdist_)
        samples.push_back(raw_depth);
    }

  bool have_point = false;
  if (!samples.empty())
  {
    const auto middle = samples.begin() + samples.size() / 2;
    std::nth_element(samples.begin(), middle, samples.end());
    const double depth = *middle / mp_.k_depth_scaling_factor_;
    const Eigen::Vector3d point_camera(
        (u - mp_.cx_) * depth / mp_.fx_,
        (v - mp_.cy_) * depth / mp_.fy_, depth);
    point_world = md_.ray_q_ * point_camera + md_.ray_pos_;
    have_point = std::abs(point_world.z() - mp_.semantic_ground_height_) <=
                 mp_.semantic_ground_tolerance_;
  }

  if (!have_point && mp_.boundary_use_ground_plane_fallback_)
  {
    const Eigen::Vector3d ray_camera(
        (u - mp_.cx_) / mp_.fx_, (v - mp_.cy_) / mp_.fy_, 1.0);
    const Eigen::Vector3d ray_world = md_.ray_q_ * ray_camera;
    if (std::abs(ray_world.z()) > 1e-6)
    {
      const double distance =
          (mp_.semantic_ground_height_ - md_.ray_pos_.z()) / ray_world.z();
      if (distance > 0.0)
      {
        point_world = md_.ray_pos_ + distance * ray_world;
        have_point = true;
      }
    }
  }
  if (!have_point)
    return false;

  Eigen::Vector3d point_base = md_.base_q_.conjugate() * (point_world - md_.base_pos_);
  if (point_base.x() <= 0.0)
    return false;
  if (point_base.x() < mp_.boundary_near_ignore_distance_ &&
      std::abs(point_base.y()) < mp_.boundary_center_ignore_half_width_)
    return false;
  if (point_base.x() > mp_.boundary_max_forward_distance_)
  {
    const double scale = mp_.boundary_max_forward_distance_ / point_base.x();
    point_base.x() = mp_.boundary_max_forward_distance_;
    point_base.y() *= scale;
    point_world = md_.base_q_ * point_base + md_.base_pos_;
  }
  point_world.z() = mp_.semantic_ground_height_;
  return point_world.x() >= mp_.map_min_boundary_.x() &&
         point_world.x() < mp_.map_max_boundary_.x() &&
         point_world.y() >= mp_.map_min_boundary_.y() &&
         point_world.y() < mp_.map_max_boundary_.y();
}

void GridMap::markBoundaryCostmapCell(int x, int y)
{
  if (x < mp_.map_bound_min_idx_.x() || x > mp_.map_bound_max_idx_.x() ||
      y < mp_.map_bound_min_idx_.y() || y > mp_.map_bound_max_idx_.y())
    return;
  const size_t mx = x - mp_.map_bound_min_idx_.x();
  const size_t my = y - mp_.map_bound_min_idx_.y();
  md_.boundary_costmap_data_[my * mp_.map_voxel_num_.x() + mx] = 100;
}

void GridMap::rasterizeBoundarySegment(const Eigen::Vector3d& start,
                                       const Eigen::Vector3d& end)
{
  Eigen::Vector3i start_index, end_index;
  posToIndex(start, start_index);
  posToIndex(end, end_index);
  int x0 = start_index.x();
  int y0 = start_index.y();
  const int x1 = end_index.x();
  const int y1 = end_index.y();
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;
  while (true)
  {
    markBoundaryCostmapCell(x0, y0);
    if (x0 == x1 && y0 == y1)
      break;
    const int doubled_error = 2 * error;
    if (doubled_error >= dy)
    {
      error += dy;
      x0 += sx;
    }
    if (doubled_error <= dx)
    {
      error += dx;
      y0 += sy;
    }
  }
}

void GridMap::addBoundaryPolyline(const vector<Eigen::Vector3d>& points)
{
  if (points.empty())
    return;
  md_.boundary_points_.push_back(points.front());
  Eigen::Vector3i first_index;
  posToIndex(points.front(), first_index);
  markBoundaryCostmapCell(first_index.x(), first_index.y());
  for (size_t i = 1; i < points.size(); ++i)
  {
    md_.boundary_points_.push_back(points[i]);
    Eigen::Vector3i index;
    posToIndex(points[i], index);
    markBoundaryCostmapCell(index.x(), index.y());
    if ((points[i] - points[i - 1]).head<2>().norm() <=
        mp_.boundary_max_neighbor_jump_)
      rasterizeBoundarySegment(points[i - 1], points[i]);
  }
}

void GridMap::updateBoundaryLayer()
{
  resetBoundaryData();
  if (md_.depth_image_.empty() || md_.semantic_mask_.empty())
    return;

  cv::Mat binary;
  cv::compare(md_.semantic_mask_, 0, binary, cv::CMP_GT);
  cv::Mat labels, stats, centroids;
  const int component_count = cv::connectedComponentsWithStats(
      binary, labels, stats, centroids, 8, CV_32S);
  if (component_count <= 1)
    return;

  const int rows = labels.rows;
  const int cols = labels.cols;
  const int roi_width = std::max(1, static_cast<int>(cols * mp_.boundary_seed_roi_width_ratio_));
  const int roi_height = std::max(1, static_cast<int>(rows * mp_.boundary_seed_roi_height_ratio_));
  const int roi_left = std::max(0, (cols - roi_width) / 2);
  const int roi_right = std::min(cols, roi_left + roi_width);
  const int roi_top = std::max(0, rows - roi_height);
  std::vector<int> seed_votes(component_count, 0);
  for (int v = roi_top; v < rows; ++v)
    for (int u = roi_left; u < roi_right; ++u)
    {
      const int label = labels.at<int>(v, u);
      if (label > 0)
        ++seed_votes[label];
    }

  int selected_label = -1;
  int best_votes = 0;
  for (int label = 1; label < component_count; ++label)
  {
    if (stats.at<int>(label, cv::CC_STAT_AREA) < mp_.boundary_min_component_area_)
      continue;
    if (seed_votes[label] > best_votes)
    {
      best_votes = seed_votes[label];
      selected_label = label;
    }
  }
  if (selected_label < 0)
  {
    int largest_area = 0;
    for (int label = 1; label < component_count; ++label)
    {
      const int area = stats.at<int>(label, cv::CC_STAT_AREA);
      if (area >= mp_.boundary_min_component_area_ && area > largest_area)
      {
        largest_area = area;
        selected_label = label;
      }
    }
  }
  if (selected_label < 0)
    return;

  std::vector<int> row_left(rows, cols), row_right(rows, -1);
  std::vector<int> column_top(cols, rows);
  for (int v = 0; v < rows; ++v)
    for (int u = 0; u < cols; ++u)
    {
      if (labels.at<int>(v, u) != selected_label)
        continue;
      row_left[v] = std::min(row_left[v], u);
      row_right[v] = std::max(row_right[v], u);
      column_top[u] = std::min(column_top[u], v);
    }

  vector<Eigen::Vector3d> left_boundary, right_boundary, front_boundary;
  for (int v = rows - 1; v >= 0; v -= mp_.boundary_pixel_step_)
  {
    if (row_right[v] < 0)
      continue;
    Eigen::Vector3d point;
    if (projectBoundaryPixel(row_left[v], v, 1, 0, point))
      left_boundary.push_back(point);
    if (projectBoundaryPixel(row_right[v], v, -1, 0, point))
      right_boundary.push_back(point);
  }

  const int component_left = stats.at<int>(selected_label, cv::CC_STAT_LEFT);
  const int component_width = stats.at<int>(selected_label, cv::CC_STAT_WIDTH);
  for (int u = component_left; u < component_left + component_width;
       u += mp_.boundary_pixel_step_)
  {
    if (column_top[u] >= rows)
      continue;
    Eigen::Vector3d point;
    if (projectBoundaryPixel(u, column_top[u], 0, 1, point))
      front_boundary.push_back(point);
  }

  // If the road leaves the top of the image, or its top-edge depth is unusable,
  // close the corridor with one virtual 2-D segment at the configured range.
  const int component_top = stats.at<int>(selected_label, cv::CC_STAT_TOP);
  const bool touches_image_top =
      component_top <= mp_.depth_filter_margin_ + mp_.boundary_pixel_step_;
  if ((touches_image_top || front_boundary.size() < 2) &&
      !left_boundary.empty() && !right_boundary.empty())
  {
    auto farthest_point = [&](const vector<Eigen::Vector3d>& points) {
      const Eigen::Vector3d* farthest = &points.front();
      double farthest_x = -std::numeric_limits<double>::infinity();
      for (const Eigen::Vector3d& point : points)
      {
        const Eigen::Vector3d point_base =
            md_.base_q_.conjugate() * (point - md_.base_pos_);
        if (point_base.x() > farthest_x)
        {
          farthest_x = point_base.x();
          farthest = &point;
        }
      }
      return *farthest;
    };
    const Eigen::Vector3d left_far = farthest_point(left_boundary);
    const Eigen::Vector3d right_far = farthest_point(right_boundary);
    auto extend_to_limit = [&](const Eigen::Vector3d& point) {
      Eigen::Vector3d point_base =
          md_.base_q_.conjugate() * (point - md_.base_pos_);
      const double scale = mp_.boundary_max_forward_distance_ /
                           std::max(point_base.x(), mp_.resolution_);
      point_base.x() = mp_.boundary_max_forward_distance_;
      point_base.y() *= scale;
      Eigen::Vector3d result = md_.base_q_ * point_base + md_.base_pos_;
      result.z() = mp_.semantic_ground_height_;
      return result;
    };
    const Eigen::Vector3d virtual_left = extend_to_limit(left_far);
    const Eigen::Vector3d virtual_right = extend_to_limit(right_far);
    front_boundary.clear();
    const double width = (virtual_right - virtual_left).head<2>().norm();
    const int sample_count = std::max(
        1, static_cast<int>(std::ceil(width /
            std::min(mp_.boundary_max_neighbor_jump_, 0.5))));
    for (int sample = 0; sample <= sample_count; ++sample)
    {
      const double ratio = static_cast<double>(sample) / sample_count;
      front_boundary.push_back((1.0 - ratio) * virtual_left + ratio * virtual_right);
    }
    rasterizeBoundarySegment(left_far, virtual_left);
    rasterizeBoundarySegment(right_far, virtual_right);
  }

  addBoundaryPolyline(left_boundary);
  addBoundaryPolyline(right_boundary);
  addBoundaryPolyline(front_boundary);

  if (mp_.boundary_publish_debug_cloud_)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(md_.boundary_points_.size());
    for (const Eigen::Vector3d& point : md_.boundary_points_)
      cloud.emplace_back(point.x(), point.y(), point.z() + 0.05);
    cloud.width = cloud.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;
    pcl::toROSMsg(cloud, boundary_cloud_cache_);
    boundary_cloud_cache_.header.stamp = node_->now();
    boundary_cloud_cache_valid_ = true;
  }
}

void GridMap::publishBoundaryCloud()
{
  if (!boundary_cloud_pub_ || !boundary_cloud_cache_valid_ ||
      boundary_cloud_pub_->get_subscription_count() == 0)
    return;
  boundary_cloud_pub_->publish(boundary_cloud_cache_);
}

void GridMap::initializeCostmapMessage(nav_msgs::msg::OccupancyGrid& message) const
{
  message.header.stamp = node_->now();
  message.header.frame_id = mp_.frame_id_;
  message.info.map_load_time = message.header.stamp;
  message.info.resolution = mp_.resolution_;
  message.info.width = mp_.map_voxel_num_(0);
  message.info.height = mp_.map_voxel_num_(1);
  message.info.origin.position.x = mp_.map_min_boundary_.x();
  message.info.origin.position.y = mp_.map_min_boundary_.y();
  message.info.origin.position.z = mp_.semantic_ground_height_;
  message.info.origin.orientation.w = 1.0;
  message.data.assign(message.info.width * message.info.height, -1);
}

void GridMap::rebuildCostmapCaches()
{
  depth_costmap_cache_valid_ = false;
  boundary_costmap_cache_valid_ = false;
  fused_costmap_cache_valid_ = false;

  if (mp_.depth_enabled_)
  {
    initializeCostmapMessage(depth_costmap_cache_);
    depth_costmap_cache_.data = md_.depth_costmap_data_;
    depth_costmap_cache_valid_ = true;
  }

  if (!mp_.semantic_enabled_)
    return;

  initializeCostmapMessage(boundary_costmap_cache_);
  boundary_costmap_cache_.data = md_.boundary_costmap_data_;
  boundary_costmap_cache_valid_ = true;

  if (mp_.fused_enabled_)
  {
    initializeCostmapMessage(fused_costmap_cache_);
    fused_costmap_cache_.data = md_.depth_costmap_data_;
    for (size_t address = 0; address < md_.boundary_costmap_data_.size(); ++address)
      if (md_.boundary_costmap_data_[address] == 100)
        fused_costmap_cache_.data[address] = 100;
    fused_costmap_cache_valid_ = true;
  }
}

void GridMap::publishDepthCostmap()
{
  if (!depth_costmap_pub_ || !depth_costmap_cache_valid_ ||
      depth_costmap_pub_->get_subscription_count() == 0)
    return;
  depth_costmap_pub_->publish(depth_costmap_cache_);
}

void GridMap::publishBoundaryCostmap()
{
  if (!boundary_costmap_pub_ || !boundary_costmap_cache_valid_ ||
      boundary_costmap_pub_->get_subscription_count() == 0)
    return;
  boundary_costmap_pub_->publish(boundary_costmap_cache_);
}

void GridMap::publishFusedCostmap()
{
  if (!fused_costmap_pub_ || !fused_costmap_cache_valid_ ||
      fused_costmap_pub_->get_subscription_count() == 0)
    return;
  fused_costmap_pub_->publish(fused_costmap_cache_);
}

void GridMap::publishMap()
{
  if (!map_pub_ || map_pub_->get_subscription_count() == 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = mp_.map_bound_min_idx_;
  Eigen::Vector3i max_cut = mp_.map_bound_max_idx_;

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] != 100)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
          continue;
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  map_pub_->publish(cloud_msg);
}

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

// GridMap

void GridMap::publishDepthCloud()
{
  if (!depth_cloud_pub_ || depth_cloud_pub_->get_subscription_count() == 0)
    return;

  if (md_.proj_points_cnt == 0)
    return;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ pt;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt.x = md_.proj_points_[i](0);
    pt.y = md_.proj_points_[i](1);
    pt.z = md_.proj_points_[i](2);
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  depth_cloud_pub_->publish(cloud_msg);
}
