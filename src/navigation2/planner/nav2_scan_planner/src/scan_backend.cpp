#include "nav2_scan_planner/scan_backend.hpp"
#include <algorithm>
#include <cmath>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace nav2_scan_planner {
namespace {
double distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return std::hypot(a.x-b.x, a.y-b.y);
}
geometry_msgs::msg::Quaternion orientation(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw/2); q.w = std::cos(yaw/2);
  return q;
}
}

ScanBackend::ScanBackend(rclcpp_lifecycle::LifecycleNode &parent,
  const std::string &prefix, std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string &frame, const std::string &base_frame)
: tf_(std::move(tf)), frame_(frame), base_frame_(base_frame) {
  map_ = std::make_shared<nav2_scan_core::GridMap>();
  map_->initMap(parent, prefix);
  resolution_ = map_->getResolution();
  horizon_ = parameter(parent, prefix + ".planning_horizon", 3.0);
  body_z_ = parameter(parent, prefix + ".body_height", 0.35);
  timeout_ = parameter(parent, prefix + ".sensor_timeout", 0.5);
  budget_ms_ = parameter(parent, prefix + ".planning_budget_ms", 60.0);
  max_vel_ = parameter(parent, prefix + ".optimization.max_vel", 0.75);
  max_acc_ = parameter(parent, prefix + ".optimization.max_acc", 0.5);
  spacing_ = parameter(parent, prefix + ".path_sample_spacing", 0.05);
  min_range_ = parameter(parent, prefix + ".min_sensor_range", 0.15);
  int max_points = parameter(parent, prefix + ".max_cloud_points", 100000);
  if (!(horizon_ > 0.5 && horizon_ <= 10 && body_z_ > 0 && timeout_ > 0 &&
      budget_ms_ > 0 && budget_ms_ <= 1000 && max_vel_ > 0 && max_acc_ > 0 &&
      spacing_ >= 0.01 && spacing_ <= resolution_ && min_range_ >= 0 && max_points > 0))
    throw std::invalid_argument("Invalid SCAN planning or sensor parameters");
  max_points_ = static_cast<size_t>(max_points);
  radius_ = parameter(parent, prefix + ".grid_map.double_cylinder_radius", 0.32) +
    parameter(parent, prefix + ".grid_map.double_cylinder_offset", 0.15);
  optimizer_.setParam(parent, prefix);
  // An internal ROS2 node permits perception updates while controller execution is busy.
  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  options.parameter_overrides({rclcpp::Parameter("use_sim_time",
    parent.get_parameter("use_sim_time").as_bool())});
  node_ = std::make_shared<rclcpp::Node>(std::string(parent.get_name()) + "_scan_gridmap",
    parent.get_namespace(), options);
  const auto topic = parameter(parent, prefix + ".cloud_topic", std::string("cur_lid"));
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(topic,
    rclcpp::SensorDataQoS().keep_last(1),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) { cloudCallback(cloud); });
  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("scan/grid_map/occupancy_inflate", 1);
  vis_timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), [this] { publishMap(); });
  executor_.add_node(node_);
  spin_thread_ = std::thread([this] { executor_.spin(); });
}

ScanBackend::~ScanBackend() {
  active_ = false;
  executor_.cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
}
void ScanBackend::activate() {
  std::lock_guard<std::mutex> lock(mutex_);
  map_->reset(); received_ = false; active_ = true;
}
void ScanBackend::deactivate() {
  active_ = false;
  std::lock_guard<std::mutex> lock(mutex_);
  received_ = false;
}
bool ScanBackend::readyLocked() const {
  if (!active_ || !received_) return false;
  const double age = (node_->now() - last_stamp_).seconds();
  const double receipt_age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_received_).count();
  return age >= -0.05 && age <= timeout_ && receipt_age <= timeout_;
}
bool ScanBackend::ready() const {
  std::lock_guard<std::mutex> lock(mutex_); return readyLocked();
}
bool ScanBackend::inside(double x, double y) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return readyLocked() && map_->isInMap(Eigen::Vector3d(x,y,body_z_));
}
bool ScanBackend::collision(double x, double y, double yaw) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return !readyLocked() || map_->getInflateOccupancy(Eigen::Vector3d(x,y,body_z_),yaw) != 0;
}
bool ScanBackend::segmentFree(double x0, double y0, double x1, double y1) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!readyLocked()) return false;
  double yaw = std::atan2(y1-y0,x1-x0);
  int n = std::max(1, static_cast<int>(std::ceil(std::hypot(x1-x0,y1-y0)/(resolution_*0.5))));
  for (int i=0;i<=n;++i) {
    double f = static_cast<double>(i)/n;
    if (map_->getInflateOccupancy(Eigen::Vector3d(x0+f*(x1-x0),y0+f*(y1-y0),body_z_),yaw)) return false;
  }
  return true;
}

void ScanBackend::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
  if (!active_ || cloud->header.frame_id.empty() || cloud->width*cloud->height == 0) return;
  try {
    const rclcpp::Time stamp(cloud->header.stamp);
    const double age = (node_->now()-stamp).seconds();
    if (age < -0.05 || age > timeout_ || stamp.nanoseconds() == 0) return;
    // Source cloud is in its sensor frame. TF also supplies the true ray origin.
    const auto transform = tf_->lookupTransform(frame_, cloud->header.frame_id, stamp,
      rclcpp::Duration::from_seconds(0.03));
    // Require a contemporary robot pose too; never fuse with unrelated stale odometry.
    tf_->lookupTransform(frame_, base_frame_, stamp, rclcpp::Duration::from_seconds(0.03));
    tf2::Transform pose; tf2::fromMsg(transform.transform,pose);
    Eigen::Vector3d sensor(pose.getOrigin().x(),pose.getOrigin().y(),pose.getOrigin().z());
    sensor_msgs::PointCloud2ConstIterator<float> x(*cloud,"x"), y(*cloud,"y"), z(*cloud,"z");
    std::vector<Eigen::Vector3d> points;
    const size_t total = static_cast<size_t>(cloud->width)*cloud->height;
    const size_t stride = std::max<size_t>(1,(total+max_points_-1)/max_points_);
    points.reserve(std::min(total,max_points_));
    for (size_t i=0; x!=x.end(); ++x,++y,++z,++i) {
      if (i%stride || !std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
      if (std::sqrt(*x**x+*y**y+*z**z) < min_range_) continue;
      auto p = pose*tf2::Vector3(*x,*y,*z);
      points.emplace_back(p.x(),p.y(),p.z());
    }
    if (points.empty()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) return;
    if (received_ && stamp <= last_stamp_) return;
    map_->integrate(points,sensor);
    last_stamp_ = stamp; last_received_ = std::chrono::steady_clock::now(); received_ = true;
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "SCAN cloud rejected: %s",e.what());
  }
}
void ScanBackend::publishMap() {
  if (!active_ || map_pub_->get_subscription_count() == 0) return;
  std::vector<Eigen::Vector3d> points;
  { std::lock_guard<std::mutex> lock(mutex_); if (!readyLocked()) return; points=map_->occupiedPoints(true); }
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id=frame_; cloud.header.stamp=node_->now();
  sensor_msgs::PointCloud2Modifier modifier(cloud); modifier.setPointCloud2FieldsByString(1,"xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"), y(cloud,"y"), z(cloud,"z");
  for (const auto &p:points) { *x=p.x(); *y=p.y(); *z=p.z(); ++x; ++y; ++z; }
  map_pub_->publish(cloud);
}

nav_msgs::msg::Path ScanBackend::plan(const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::Twist &speed, const nav_msgs::msg::Path &reference,
  const geometry_msgs::msg::PoseStamped &goal) {
  nav2_scan_core::GridMap::Ptr snapshot;
  { std::lock_guard<std::mutex> lock(mutex_);
    if (!readyLocked()) throw std::runtime_error("SCAN GridMap is not current");
    snapshot=std::make_shared<nav2_scan_core::GridMap>(*map_);
  }
  auto path=optimize(snapshot,optimizer_,start,speed,reference,goal,body_z_,max_vel_,max_acc_,spacing_,budget_ms_);
  // The live map may have advanced during optimization. Validate every output segment again.
  for (size_t i=1;i<path.poses.size();++i) {
    const auto &a=path.poses[i-1].pose.position, &b=path.poses[i].pose.position;
    if (!segmentFree(a.x,a.y,b.x,b.y)) throw std::runtime_error("SCAN result invalidated by new perception");
  }
  return path;
}

nav_msgs::msg::Path ScanBackend::optimize(nav2_scan_core::GridMap::Ptr map,
  nav2_scan_core::BsplineOptimizer &optimizer, const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::Twist &speed, const nav_msgs::msg::Path &reference,
  const geometry_msgs::msg::PoseStamped &goal, double z, double max_vel,
  double max_acc, double spacing, double budget_ms) {
  using nav2_scan_core::UniformBspline;
  map->deadline=std::chrono::steady_clock::now()+std::chrono::microseconds(static_cast<int64_t>(budget_ms*1000));
  optimizer.setEnvironment(map);
  optimizer.a_star_=std::make_shared<nav2_scan_core::AStar>();
  Eigen::Vector3d origin,size; map->getRegion(origin,size);
  // Native search uses XY neighbours and an interpolated Z plane: three Z slots suffice here.
  optimizer.a_star_->initGridMap(map,Eigen::Vector3i(
    static_cast<int>(size.x()/map->getResolution())+4,
    static_cast<int>(size.y()/map->getResolution())+4,3));
  map->checkDeadline();
  std::vector<Eigen::Vector3d> vertices;
  vertices.emplace_back(start.pose.position.x,start.pose.position.y,z);
  for (const auto &p:reference.poses) {
    Eigen::Vector3d v(p.pose.position.x,p.pose.position.y,z);
    if ((v-vertices.back()).norm()>0.01) vertices.push_back(v);
  }
  Eigen::Vector3d end(goal.pose.position.x,goal.pose.position.y,z);
  if ((end-vertices.back()).norm()>0.01) vertices.push_back(end);
  std::vector<double> lengths{0};
  for (size_t i=1;i<vertices.size();++i) lengths.push_back(lengths.back()+(vertices[i]-vertices[i-1]).norm());
  if (lengths.back()<0.2) throw std::runtime_error("SCAN segment too short");
  int count=std::max(8,static_cast<int>(std::ceil(lengths.back()/0.2))+1);
  if (count>200) throw std::runtime_error("SCAN reference exceeds local planning bound");
  std::vector<Eigen::Vector3d> samples;
  size_t j=1;
  for(int i=0;i<count;++i) {
    const double s=lengths.back()*i/(count-1);
    while(j+1<lengths.size() && lengths[j]<s) ++j;
    const double f=(s-lengths[j-1])/(lengths[j]-lengths[j-1]);
    samples.push_back(vertices[j-1]+f*(vertices[j]-vertices[j-1]));
  }
  const double ts=lengths.back()/(count-1)/max_vel*1.5;
  const double yaw=tf2::getYaw(start.pose.orientation);
  Eigen::Vector3d velocity(std::cos(yaw)*speed.linear.x-std::sin(yaw)*speed.linear.y,
    std::sin(yaw)*speed.linear.x+std::cos(yaw)*speed.linear.y,0);
  if(velocity.norm()>max_vel) velocity*=max_vel/velocity.norm();
  Eigen::Vector3d end_velocity=vertices.back()-vertices[vertices.size()-2];
  end_velocity=end_velocity.normalized()*max_vel*0.5;
  std::vector<Eigen::Vector3d> derivatives{velocity,end_velocity,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()};
  Eigen::MatrixXd controls;
  UniformBspline::parameterizeToBspline(ts,samples,derivatives,controls);
  controls.row(2).setConstant(z);
  optimizer.initControlPoints(controls,true);
  if(!optimizer.BsplineOptimizeTrajRebound(controls,ts) || !controls.allFinite())
    throw std::runtime_error("SCAN rebound optimization failed");
  UniformBspline trajectory(controls,3,ts);
  trajectory.setPhysicalLimits(max_vel,max_acc,0.05);
  double ratio=1;
  if(!trajectory.checkFeasibility(ratio,false)) {
    // Uniform time scaling preserves the collision-checked geometry. RPP owns execution timing.
    if(!std::isfinite(ratio) || ratio<=0) throw std::runtime_error("Invalid SCAN feasibility ratio");
    trajectory=UniformBspline(controls,3,ts*std::max(1.0,ratio)*1.05);
  }
  auto velocity_traj=trajectory.getDerivative();
  double max_speed=0;
  const auto velocity_controls=velocity_traj.getControlPoint();
  for(int i=0;i<velocity_controls.cols();++i) max_speed=std::max(max_speed,velocity_controls.col(i).norm());
  const double dt=std::min(0.05,spacing/std::max(0.01,max_speed));
  const double duration=trajectory.getTimeSum();
  nav_msgs::msg::Path path; path.header=start.header;
  for(double t=0;;t=std::min(t+dt,duration)) {
    map->checkDeadline();
    Eigen::Vector3d p=trajectory.evaluateDeBoorT(t);
    if(!p.allFinite()) throw std::runtime_error("Non-finite SCAN output");
    geometry_msgs::msg::PoseStamped pose; pose.header=path.header;
    pose.pose.position.x=p.x(); pose.pose.position.y=p.y(); pose.pose.position.z=start.pose.position.z;
    if(path.poses.empty() || distance(path.poses.back().pose.position,pose.pose.position)>1e-5)
      path.poses.push_back(pose);
    if(t>=duration) break;
  }
  if(path.poses.size()<2 || distance(path.poses.front().pose.position,start.pose.position)>0.2 ||
      distance(path.poses.back().pose.position,goal.pose.position)>0.2)
    throw std::runtime_error("SCAN endpoints do not connect to route");
  path.poses.insert(path.poses.begin(),start); path.poses.push_back(goal);
  for(size_t i=1;i<path.poses.size();++i) {
    const auto &a=path.poses[i-1].pose.position, &b=path.poses[i].pose.position;
    const double heading=std::atan2(b.y-a.y,b.x-a.x);
    path.poses[i-1].pose.orientation=orientation(heading);
    const int n=std::max(1,static_cast<int>(std::ceil(distance(a,b)/(map->getResolution()*0.5))));
    for(int k=0;k<=n;++k) {
      const double f=static_cast<double>(k)/n;
      if(map->getInflateOccupancy(Eigen::Vector3d(a.x+f*(b.x-a.x),a.y+f*(b.y-a.y),z),heading))
        throw std::runtime_error("SCAN output or connection collides");
    }
  }
  path.poses.back().pose.orientation=goal.pose.orientation;
  map->checkDeadline();
  return path;
}
} // namespace nav2_scan_planner
