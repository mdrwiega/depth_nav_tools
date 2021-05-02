#include <nav_layer_from_points/costmap_layer.h>

#include <fstream>

PLUGINLIB_EXPORT_CLASS(nav_layer_from_points::NavLayerFromPoints, nav2_costmap_2d::Layer)

namespace nav_layer_from_points {

void NavLayerFromPoints::onInitialize() {
  current_ = true;
  first_time_ = true;

  node_->declare_parameter(name_ + "." + "enabled", rclcpp::ParameterValue(true));

  // sub_points_ = nh.subscribe("/downstairs_detector/points", 1,
                              // &NavLayerFromPoints::pointsCallback, this);

  // rec_server_ = new dynamic_reconfigure::Server<NavLayerFromPointsConfig>(nh);
  // f_ = boost::bind(&NavLayerFromPoints::configure, this, _1, _2);
  // rec_server_->setCallback(f_);
}

// void NavLayerFromPoints::configure(NavLayerFromPointsConfig &config, [[maybe_unused]] uint32_t level) {
//   points_keep_time_ = ros::Duration(config.keep_time);
//   enabled_ = config.enabled;

//   point_radius_ = config.point_radius;
//   robot_radius_ = config.robot_radius;
// }

void NavLayerFromPoints::pointsCallback(const geometry_msgs::msg::PolygonStamped& points) {
  std::lock_guard<std::recursive_mutex> lk(lock_);
  points_list_ = points;
}

void NavLayerFromPoints::clearTransformedPoints() {
  std::list<geometry_msgs::msg::PointStamped>::iterator p_it;
  p_it = transformed_points_.begin();

  if (transformed_points_.size() > 10000)
    transformed_points_.clear();

  while (p_it != transformed_points_.end()) {
    // if (node_->get_clock()->now() - (*p_it).header.stamp > points_keep_time_) {
    //   p_it = transformed_points_.erase(p_it);
    // }
    // else {
    //   ++p_it;
    // }
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
  for (const auto point : points_list_.polygon.points) {
    geometry_msgs::msg::PointStamped tpt;
    geometry_msgs::msg::PointStamped pt, out_pt;

    tpt.point = nav2_costmap_2d::toPoint(point);

    try {
      pt.point.x = tpt.point.x;
      pt.point.y = 0;
      pt.point.z =  tpt.point.z;
      pt.header.frame_id = points_list_.header.frame_id;

      // tf_.transformPoint(global_frame, pt, out_pt);
      tpt.point.x = out_pt.point.x;
      tpt.point.y = out_pt.point.y;
      tpt.point.z = out_pt.point.z;

      tpt.header.stamp = pt.header.stamp;
      //s << " ( " << tpt.point.x << " , " << tpt.point.y << " , " << tpt.point.z << " ) ";
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
  // std::ostringstream s;
  // s << " list_size = " << transformed_points_.size() << "   ";
  // s << " min_x = " << *min_x << " max_x = " << *max_x <<
        // " min_y = " << *min_y << " max_y = " << *max_y << "      ";
  // ROS_INFO_STREAM_THROTTLE(2,"transformed_points = " << s.str());

}

void NavLayerFromPoints::updateBoundsFromPoints(double* min_x, double* min_y, double* max_x, double* max_y) {
  std::list<geometry_msgs::msg::PointStamped>::iterator p_it;

  double radius = point_radius_ + robot_radius_;

  for (p_it = transformed_points_.begin(); p_it != transformed_points_.end(); ++p_it) {
    geometry_msgs::msg::PointStamped pt = *p_it;

    *min_x = std::min(*min_x, pt.point.x - radius);
    *min_y = std::min(*min_y, pt.point.y - radius);
    *max_x = std::max(*max_x, pt.point.x + radius);
    *max_y = std::max(*max_y, pt.point.y + radius);
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
  double res = costmap->getResolution();

  for (auto p_it = transformed_points_.begin(); p_it != transformed_points_.end(); ++p_it) {
    geometry_msgs::msg::Point pt = (*p_it).point;

    int size = std::max(1, int( (point_radius_ + robot_radius_) / res ));
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

      if (start_x < min_i)
        start_x = min_i;
      if (end_x > max_i)
        end_x = max_i;
      if (start_y < min_j)
        start_y = min_j;
      if (end_y > max_j)
        end_y = max_j;

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