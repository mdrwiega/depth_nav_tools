#pragma once

#include <mutex>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/footprint.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_ros/buffer.h>

namespace nav_layer_from_points {

class NavLayerFromPoints : public nav2_costmap_2d::CostmapLayer {
public:
  NavLayerFromPoints();

  void onInitialize() override;

  void reset() override {}

  void updateBounds(double origin_x, double origin_y, double origin_z,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;

  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;

  void updateBoundsFromPoints( double* min_x, double* min_y,
                               double* max_x, double* max_y);

  bool isDiscretized() { return false; }

protected:

  void pointsCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr points);

  /**
   * @brief clearTransformedPoints clears points from list transformed_points_ after some time
   */
  void clearTransformedPoints();

  /// Subscriber for points
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_points_;

  geometry_msgs::msg::PolygonStamped points_list_;  ///< List of received points

  tf2_ros::Buffer tf_buffer_;

  std::list<geometry_msgs::msg::PointStamped> transformed_points_;

  // After this time points will be delete
  rclcpp::Duration points_keep_time_{0};

  std::recursive_mutex lock_;
  bool first_time_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

private:
  double point_radius_;
  double robot_radius_;
};

} // namespace nav_layer_from_points