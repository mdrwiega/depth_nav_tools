#pragma once

#include <ros/ros.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/layer.h>
#include <pluginlib/class_list_macros.h>

#include <depth_nav_msgs/Point32List.h>
#include <geometry_msgs/PointStamped.h>

#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <math.h>
#include <algorithm>

#include <angles/angles.h>

#include <dynamic_reconfigure/server.h>
#include <nav_layer_from_points/NavLayerFromPointsConfig.h>


namespace nav_layer_from_points
{
class NavLayerFromPoints : public costmap_2d::Layer
{
public:
  NavLayerFromPoints() { layered_costmap_ = nullptr; }

  void onInitialize() override;

  void updateBounds(double origin_x, double origin_y, double origin_z,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;

  void updateCosts(costmap_2d::Costmap2D& master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;

  void updateBoundsFromPoints( double* min_x, double* min_y,
                               double* max_x, double* max_y);

  bool isDiscretized() { return false; }

protected:

  void pointsCallback(const depth_nav_msgs::Point32List& points);

  /**
   * @brief clearTransformedPoints clears points from list transformed_points_ after some time
   */
  void clearTransformedPoints();

  void configure(NavLayerFromPointsConfig &config, uint32_t level);

protected: // Protected fields
  ros::Subscriber sub_points_;               ///< Subscriber for points
  depth_nav_msgs::Point32List points_list_;  ///< List of received points

  tf::TransformListener tf_;

  std::list<geometry_msgs::PointStamped> transformed_points_;

  // After this time points will be delete
  ros::Duration points_keep_time_;

  boost::recursive_mutex lock_;
  bool first_time_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

private:
  // ROS parameters configurated with config file or dynamic_reconfigure
  double point_radius_;
  double robot_radius_;

  dynamic_reconfigure::Server<NavLayerFromPointsConfig>* rec_server_;
  dynamic_reconfigure::Server<NavLayerFromPointsConfig>::CallbackType f_;
};

} // namespace nav_layer_from_points