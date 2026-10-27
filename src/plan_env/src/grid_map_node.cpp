#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "plan_env/grid_map.h"

class GridMapNode : public rclcpp::Node
{
public:
  GridMapNode() : Node("grid_map_node"), grid_map_(std::make_shared<GridMap>())
  {
    grid_map_->initMap(this);
    RCLCPP_INFO(get_logger(), "depth grid map started");
  }

private:
  GridMap::Ptr grid_map_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<GridMapNode>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("grid_map_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
