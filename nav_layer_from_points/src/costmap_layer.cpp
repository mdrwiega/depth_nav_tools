#include <nav_layer_from_points/costmap_layer.h>

#include <fstream>

PLUGINLIB_EXPORT_CLASS(nav_layer_from_points::NavLayerFromPoints, nav2_costmap_2d::Layer)

namespace nav_layer_from_points {

NavLayerFromPoints::NavLayerFromPoints()
  : tf_buffer_(node_->get_clock())
{
  layered_costmap_ = nullptr;
}

void NavLayerFromPoints::onInitialize() {
  current_ = true;
  first_time_ = true;

  node_->declare_parameter(name_ + "." + "enabled", true);
  node_->declare_parameter(name_ + "." + "keep_time", 0.75);
  node_->declare_parameter(name_ + "." + "point_radius", 0.2);
  node_->declare_parameter(name_ + "." + "robot_radius", 0.6);

  sub_points_ = node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "points", 1, std::bind(&NavLayerFromPoints::pointsCallback, this, std::placeholders::_1));
}

void NavLayerFromPoints::pointsCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr points) {
  std::lock_guard<std::recursive_mutex> lk(lock_);
  points_list_ = *points;
}

void NavLayerFromPoints::clearTransformedPoints() {
  if (transformed_points_.size() > 10000)
    transformed_points_.clear();

  auto p_it = transformed_points_.begin();
  while (p_it != transformed_points_.end()) {
    if (node_->get_clock()->now() - (*p_it).header.stamp > points_keep_time_) {
      p_it = transformed_points_.erase(p_it);
    }
    else {
      ++p_it;
    }
  }
}

void NavLayerFromPoints::updateBounds([[maybe_unused]] double origin_x,
                                      [[maybe_unused]] double origin_y,
                                      [[maybe_unused]] double origin_z,
                                      double* min_x, double* min_y, double* max_x, double* max_y) {
  std::lock_guard<std::recursive_mutex> lk(lock_);

  std::string global_frame = layered_costmap_->getGlobalFrameID();

  // Check if there are points to remove in transformed_points list and if so, then remove it
  if (!transformed_points_.empty())
    clearTransformedPoints();

  // Add points to PointStamped list transformed_points_
  for (const auto& point : points_list_.polygon.points) {
    geometry_msgs::msg::PointStamped tpt;
    geometry_msgs::msg::PointStamped pt, out_pt;

    tpt.point = nav2_costmap_2d::toPoint(point);

    try {
      pt.point.x = tpt.point.x;
      pt.point.y = 0;
      pt.point.z =  tpt.point.z;
      pt.header.frame_id = points_list_.header.frame_id;

      tf_buffer_.transform(pt, out_pt, global_frame);
      tpt.point.x = out_pt.point.x;
      tpt.point.y = out_pt.point.y;
      tpt.point.z = out_pt.point.z;

      tpt.header.stamp = pt.header.stamp;
      transformed_points_.push_back(tpt);
    }
    catch(tf2::LookupException& ex) {
      RCLCPP_ERROR(node_->get_logger(), "No Transform available Error: %s\n", ex.what());
      continue;
    }
    catch(tf2::ConnectivityException& ex) {
      RCLCPP_ERROR(node_->get_logger(), "Connectivity Error: %s\n", ex.what());
      continue;
    }
    catch(tf2::ExtrapolationException& ex) {
      RCLCPP_ERROR(node_->get_logger(), "Extrapolation Error: %s\n", ex.what());
      continue;
    }
  }

  updateBoundsFromPoints(min_x, min_y, max_x, max_y);

  if (first_time_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    first_time_ = false;
  }
  else {
    double a = *min_x, b = *min_y, c = *max_x, d = *max_y;
    *min_x = std::min(last_min_x_, *min_x);
    *min_y = std::min(last_min_y_, *min_y);
    *max_x = std::max(last_max_x_, *max_x);
    *max_y = std::max(last_max_y_, *max_y);
    last_min_x_ = a;
    last_min_y_ = b;
    last_max_x_ = c;
    last_max_y_ = d;
  }
}

void NavLayerFromPoints::updateBoundsFromPoints(double* min_x, double* min_y, double* max_x, double* max_y) {
  double radius = point_radius_ + robot_radius_;

  for (const auto& point : transformed_points_) {
    *min_x = std::min(*min_x, point.point.x - radius);
    *min_y = std::min(*min_y, point.point.y - radius);
    *max_x = std::max(*max_x, point.point.x + radius);
    *max_y = std::max(*max_y, point.point.y + radius);
  }
}

void NavLayerFromPoints::updateCosts([[maybe_unused]] nav2_costmap_2d::Costmap2D& master_grid,
                                     int min_i, int min_j, int max_i, int max_j) {
  std::lock_guard<std::recursive_mutex> lk(lock_);

  if (!enabled_)
    return;

  if (points_list_.polygon.points.empty())
    return;

  auto costmap = layered_costmap_->getCostmap();
  const auto resolution = costmap->getResolution();

  for (const auto& point : transformed_points_) {
    geometry_msgs::msg::Point pt = point.point;

    int size = std::max(1, int( (point_radius_ + robot_radius_) / resolution ));
    unsigned map_x, map_y;
    int size_x = size, size_y = size;
    int start_x, start_y, end_x, end_y;

    // Check if point is on map
    // Convert from world coordinates to map coordinates with checking for legal bounds
    if (costmap->worldToMap(pt.x, pt.y, map_x, map_y)) {
      start_x = map_x - size_x / 2;
      start_y = map_y - size_y / 2;
      end_x = map_x + size_x / 2;
      end_y = map_y + size_y / 2;

      if (start_x < min_i) start_x = min_i;
      if (end_x > max_i)   end_x = max_i;
      if (start_y < min_j) start_y = min_j;
      if (end_y > max_j)   end_y = max_j;

      for(int j = start_y; j < end_y; j++) {
        for(int i = start_x; i < end_x; i++) {
          unsigned char old_cost = costmap->getCost(i, j);

          if(old_cost == nav2_costmap_2d::NO_INFORMATION)
            continue;

          costmap->setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }
}

} // namespace nav_layer_from_points