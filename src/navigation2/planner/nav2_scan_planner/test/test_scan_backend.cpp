#include <gtest/gtest.h>
#include "nav2_scan_planner/scan_backend.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

class ScanTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0,nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("scan_test");
    map=std::make_shared<nav2_scan_core::GridMap>();
    map->initMap(*node,"scan");
    optimizer.setParam(*node,"scan");
  }
  geometry_msgs::msg::PoseStamped pose(double x,double y) {
    geometry_msgs::msg::PoseStamped p; p.header.frame_id="map";
    p.pose.position.x=x; p.pose.position.y=y; p.pose.orientation.w=1; return p;
  }
  nav_msgs::msg::Path straight() {
    nav_msgs::msg::Path p; p.header.frame_id="map";
    for(int i=0;i<=30;++i) p.poses.push_back(pose(i*0.1,0));
    return p;
  }
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  nav2_scan_core::GridMap::Ptr map;
  nav2_scan_core::BsplineOptimizer optimizer;
};

TEST_F(ScanTest, NativeRayFusionClearsAndSlides) {
  Eigen::Vector3d sensor(0.05,0.05,0.35), obstacle(1.05,0.05,0.35);
  for(int i=0;i<8;++i) map->integrate({obstacle},sensor);
  EXPECT_EQ(map->getOccupancy(obstacle),1);
  EXPECT_NE(map->getInflateOccupancy(Eigen::Vector3d(0.85,0.05,0.35),0),0);
  for(int i=0;i<20;++i) map->integrate({Eigen::Vector3d(3.05,0.05,0.35)},sensor);
  EXPECT_EQ(map->getOccupancy(obstacle),0);
  map->integrate({Eigen::Vector3d(22.05,0.05,0.35)},Eigen::Vector3d(20.05,0.05,0.35));
  EXPECT_FALSE(map->isInMap(obstacle));
  EXPECT_TRUE(map->isInMap(Eigen::Vector3d(20.05,0.05,0.35)));
  EXPECT_EQ(map->getInflateOccupancy(obstacle,0),-1);
}

TEST_F(ScanTest, FreeSplineConnectsEndpointsAndStaysPlanar) {
  const auto path=nav2_scan_planner::ScanBackend::optimize(map,optimizer,pose(0,0),
    geometry_msgs::msg::Twist{},straight(),pose(3,0),0.35,0.75,0.5,0.05,500);
  ASSERT_GT(path.poses.size(),2u);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x,0);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x,3);
  for(const auto &p:path.poses) {
    EXPECT_NEAR(p.pose.position.y,0,1e-6);
    EXPECT_DOUBLE_EQ(p.pose.position.z,0);
    EXPECT_TRUE(std::isfinite(p.pose.orientation.w));
  }
}

TEST_F(ScanTest, ObstacleProducesCollisionFreeDetour) {
  for(double x=1.3;x<1.6;x+=0.1)
    for(double y=-0.2;y<0.3;y+=0.1) map->setOccupied(Eigen::Vector3d(x,y,0.35));
  auto path=nav2_scan_planner::ScanBackend::optimize(map,optimizer,pose(0,0),
    geometry_msgs::msg::Twist{},straight(),pose(3,0),0.35,0.75,0.5,0.05,1000);
  double excursion=0;
  for(const auto &p:path.poses) excursion=std::max(excursion,std::abs(p.pose.position.y));
  EXPECT_GT(excursion,0.4);
  for(size_t i=1;i<path.poses.size();++i) {
    const auto &a=path.poses[i-1].pose.position,&b=path.poses[i].pose.position;
    double yaw=std::atan2(b.y-a.y,b.x-a.x);
    for(int j=0;j<=10;++j) {
      double t=j/10.0;
      EXPECT_EQ(map->getInflateOccupancy(Eigen::Vector3d(a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),0.35),yaw),0);
    }
  }
}

TEST_F(ScanTest, DeadlineAndBlockedEndpointFail) {
  EXPECT_THROW(nav2_scan_planner::ScanBackend::optimize(map,optimizer,pose(0,0),
    geometry_msgs::msg::Twist{},straight(),pose(3,0),0.35,0.75,0.5,0.05,0.001),std::runtime_error);
  map->deadline=std::chrono::steady_clock::time_point::max();
  map->setOccupied(Eigen::Vector3d(3,0,0.35));
  EXPECT_THROW(nav2_scan_planner::ScanBackend::optimize(map,optimizer,pose(0,0),
    geometry_msgs::msg::Twist{},straight(),pose(3,0),0.35,0.75,0.5,0.05,500),std::runtime_error);
}

TEST_F(ScanTest, BackendRequiresFreshPerception) {
  auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
  nav2_scan_planner::ScanBackend backend(*node,"scan",tf,"map","base_link");
  EXPECT_FALSE(backend.ready());
  backend.activate();
  EXPECT_FALSE(backend.ready());
  EXPECT_TRUE(backend.collision(0,0,0));
  EXPECT_FALSE(backend.segmentFree(0,0,1,0));
  backend.deactivate();
  EXPECT_FALSE(backend.ready());
}

TEST_F(ScanTest, SensorFrameCloudUsesTfRayOriginAndExpires) {
  auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id="map"; transform.child_frame_id="lidar";
  transform.transform.translation.x=1.0; transform.transform.translation.z=0.3;
  transform.transform.rotation.w=1;
  ASSERT_TRUE(tf->setTransform(transform,"test",true));
  transform.child_frame_id="base_link"; transform.transform.translation.x=0;
  transform.transform.translation.z=0;
  ASSERT_TRUE(tf->setTransform(transform,"test",true));
  node->declare_parameter("scan.cloud_topic","scan_test_cloud");
  node->declare_parameter("scan.sensor_timeout",0.2);
  nav2_scan_planner::ScanBackend backend(*node,"scan",tf,"map","base_link");
  backend.activate();
  auto publisher_node=std::make_shared<rclcpp::Node>("scan_test_publisher");
  auto publisher=publisher_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "scan_test_cloud",rclcpp::SensorDataQoS());
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id="lidar";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1,"xyz"); modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
  *x=1.05; *y=0.05; *z=0.05;
  for(int i=0;i<30;++i) {
    cloud.header.stamp=node->now(); publisher->publish(cloud);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(backend.ready());
  EXPECT_TRUE(backend.collision(2.05,0.05,0));
  EXPECT_FALSE(backend.collision(0.05,0.05,0));
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  EXPECT_FALSE(backend.ready());
  EXPECT_TRUE(backend.collision(0.05,0.05,0));
  backend.deactivate();
}
