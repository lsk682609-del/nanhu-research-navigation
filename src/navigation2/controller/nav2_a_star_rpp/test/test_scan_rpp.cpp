#include <gtest/gtest.h>
#include "nav2_a_star_rpp/regulated_pure_pursuit_controller.hpp"

class ScanController : public nav2_a_star_rpp::RegulatedPurePursuitController {
public:
  void cache(const nav_msgs::msg::Path &route) {
    global_plan_=route; track_path_=route; join_idx_=10; avoid_flag_=true;
    track_path_.poses[5].pose.position.y=1;
  }
  bool avoiding() const { return avoid_flag_; }
  double cachedY() const { return track_path_.poses.at(5).pose.position.y; }
  double mapCost(double x,double y) { return costAtPose(x,y); }
  bool collides(double x,double y) { return inCollision(x,y,0); }
  auto update(std::vector<rclcpp::Parameter> p) { return dynamicParametersCallback(p); }
};

TEST(ScanRpp, EmptyCostmapShellAndSlicePreservation) {
  rclcpp::init(0,nullptr);
  {
    auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("scan_rpp_test");
    node->declare_parameter("FollowPath.local_planner_type","scan");
    node->declare_parameter("FollowPath.use_cost_regulated_linear_velocity_scaling",false);
    auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto costmap=std::make_shared<nav2_costmap_2d::Costmap2DROS>("scan_shell");
    costmap->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{}));
    costmap->on_configure(rclcpp_lifecycle::State());
    EXPECT_TRUE(costmap->getLayeredCostmap()->getPlugins()->empty());
    EXPECT_TRUE(costmap->isCurrent());
    ScanController controller;
    controller.configure(node,"FollowPath",tf,costmap);
    // Coordinates outside the shell must not trigger costmap reads.
    EXPECT_DOUBLE_EQ(controller.mapCost(1000,1000),0);
    EXPECT_TRUE(controller.collides(1000,1000)); // GridMap not ready => stop
    nav_msgs::msg::Path route; route.header.frame_id=costmap->getGlobalFrameID();
    for(int i=0;i<30;++i) {
      geometry_msgs::msg::PoseStamped p; p.header=route.header;
      p.pose.position.x=i*0.1; p.pose.orientation.w=1; route.poses.push_back(p);
    }
    controller.cache(route);
    auto advanced=route;
    advanced.poses.erase(advanced.poses.begin(),advanced.poses.begin()+2);
    controller.setPlan(advanced);
    EXPECT_TRUE(controller.avoiding()); EXPECT_DOUBLE_EQ(controller.cachedY(),1);
    auto changed=advanced; changed.poses[3].pose.position.y=2;
    controller.setPlan(changed);
    EXPECT_FALSE(controller.avoiding());
    EXPECT_FALSE(controller.update({rclcpp::Parameter("FollowPath.local_planner_type","theta_star")}).successful);
    EXPECT_FALSE(controller.update({rclcpp::Parameter("FollowPath.use_collision_detection",false)}).successful);
    controller.cleanup(); costmap->on_cleanup(rclcpp_lifecycle::State());
  }
  rclcpp::shutdown();
}
