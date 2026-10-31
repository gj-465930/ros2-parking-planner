#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "parking_planner/core/geometry/convex_polygon2d.h"
#include "parking_planner/core/geometry/oriented_rectangle2d.h"
#include "parking_planner/core/geometry/pose2d.h"
#include "parking_planner/core/geometry/vec2d.h"
#include "parking_planner/core/map/environment.h"
#include "parking_planner/core/scenario/scenario_loader.h"
#include "parking_planner/core/vehicle/vehicle_params.h"

namespace parking_planner::core::scenario {

Scenario loadScenarioFromFile(const std::string &file_path) {
  std::string context = "root";

  try {
    // 1. 打开并且解析 YAML 文件
    if (file_path.empty()) {
      throw std::runtime_error("file path must not be empty");
    }

    const YAML::Node root = YAML::LoadFile(file_path);

    if (!root.IsMap()) {
      throw std::runtime_error("root must be a mapping");
    }

    // 2. 场景名称
    context = "name";
    const YAML::Node name_node = root["name"];

    if (!name_node.IsScalar()) {
      throw std::runtime_error("name must be a scalar");
    }

    std::string name = name_node.as<std::string>();

    if (name.find_first_not_of(" \t\r\n") == std::string::npos) {
      throw std::runtime_error("name must not be empty or whitespace");
    }

    // 3. 车辆参数
    context = "vehicle";
    const YAML::Node vehicle_node = root["vehicle"];

    if (!vehicle_node.IsMap()) {
      throw std::runtime_error("vehicle must be a mapping");
    }

    context = "vehicle.wheelbase";
    const double vehicle_wheelbase = vehicle_node["wheelbase"].as<double>();

    context = "vehicle.width";
    const double vehicle_width = vehicle_node["width"].as<double>();

    context = "vehicle.front_overhang";
    const double vehicle_front_overhang =
        vehicle_node["front_overhang"].as<double>();

    context = "vehicle.rear_overhang";
    const double vehicle_rear_overhang =
        vehicle_node["rear_overhang"].as<double>();

    context = "vehicle.max_steering_angle";
    const double vehicle_max_steering_angle =
        vehicle_node["max_steering_angle"].as<double>();

    context = "vehicle";
    const vehicle::VehicleParams params(
        vehicle_wheelbase, vehicle_width, vehicle_front_overhang,
        vehicle_rear_overhang, vehicle_max_steering_angle);

    // 4. 起点中心后轴位姿
    context = "start";
    const YAML::Node start_node = root["start"];

    if (!start_node.IsMap()) {
      throw std::runtime_error("start must be a mapping");
    }

    context = "start.x";
    const double start_x = start_node["x"].as<double>();

    context = "start.y";
    const double start_y = start_node["y"].as<double>();

    context = "start.theta";
    const double start_theta = start_node["theta"].as<double>();

    context = "start";
    if (!std::isfinite(start_x) || !std::isfinite(start_y) ||
        !std::isfinite(start_theta)) {
      throw std::runtime_error("x, y and theta must be finite");
    }

    geometry::Pose2d start_pose{start_x, start_y, start_theta};

    // 5. 目标后轴中心位姿
    context = "goal";
    const YAML::Node goal_node = root["goal"];

    if (!goal_node.IsMap()) {
      throw std::runtime_error("goal must be a mapping");
    }

    context = "goal.x";
    const double goal_x = goal_node["x"].as<double>();

    context = "goal.y";
    const double goal_y = goal_node["y"].as<double>();

    context = "goal.theta";
    const double goal_theta = goal_node["theta"].as<double>();

    context = "goal";

    if (!std::isfinite(goal_x) || !std::isfinite(goal_y) ||
        !std::isfinite(goal_theta)) {
      throw std::runtime_error("x, y, and theta must be finite");
    }

    geometry::Pose2d goal_pose{goal_x, goal_y, goal_theta};

    // 6. 地图参数
    context = "map";
    const YAML::Node map_node = root["map"];

    if (!map_node.IsMap()) {
      throw std::runtime_error("map must be a mapping");
    }

    context = "map.origin";
    const YAML::Node origin_node = map_node["origin"];

    if (!origin_node.IsSequence() || origin_node.size() != 2) {
      throw std::runtime_error("origin must contain exactly [x, y]");
    }

    const double origin_x = origin_node[0].as<double>();
    const double origin_y = origin_node[1].as<double>();

    geometry::Vec2d origin(origin_x, origin_y);

    context = "map.resolution";
    const double resolution = map_node["resolution"].as<double>();

    context = "map.width";
    const int map_width = map_node["width"].as<int>();

    context = "map.height";
    const int map_height = map_node["height"].as<int>();

    // 7. 障碍物列表
    context = "obstacles";
    const YAML::Node obstacles_node = root["obstacles"];

    if (!obstacles_node.IsSequence()) {
      throw std::runtime_error("obstacles must be a sequence");
    }

    std::vector<geometry::ConvexPolygon2d> obstacles;
    obstacles.reserve(obstacles_node.size());

    for (std::size_t i = 0; i < obstacles_node.size(); ++i) {
      const std::string obstacle_path = "obstacles[" + std::to_string(i) + "]";

      context = obstacle_path;

      const YAML::Node obstacle_node = obstacles_node[i];

      if (!obstacle_node.IsMap()) {
        throw std::runtime_error("obstacle must be a mapping");
      }

      const YAML::Node vertices_node = obstacle_node["vertices"];

      if (!vertices_node.IsSequence()) {
        throw std::runtime_error("vertices must be a sequence");
      }

      std::vector<geometry::Vec2d> vertices;
      vertices.reserve(vertices_node.size());

      for (std::size_t j = 0; j < vertices_node.size(); ++j) {
        context = obstacle_path + ".vertices[" + std::to_string(j) + "]";
        const YAML::Node point_node = vertices_node[j];
        if (!point_node.IsSequence() || point_node.size() != 2) {
          throw std::runtime_error("vertex must contain exactly [x, y]");
        }

        const double x = point_node[0].as<double>();
        const double y = point_node[1].as<double>();

        vertices.emplace_back(x, y);
      }
      context = obstacle_path;
      obstacles.emplace_back(vertices);
    }

    // 8. 目标车位矩形
    context = "parking_space";
    const YAML::Node parking_node = root["parking_space"];

    if (!parking_node.IsMap()) {
      throw std::runtime_error("parking_space must be a mapping");
    }

    context = "parking_space.center";
    const YAML::Node center_node = parking_node["center"];

    if (!center_node.IsSequence() || center_node.size() != 2) {
      throw std::runtime_error("center must contain exactly [x, y]");
    }

    const double center_x = center_node[0].as<double>();
    const double center_y = center_node[1].as<double>();

    geometry::Vec2d center(center_x, center_y);

    context = "parking_space.heading";
    const double parking_space_heading = parking_node["heading"].as<double>();

    context = "parking_space.length";
    const double parking_space_length = parking_node["length"].as<double>();

    context = "parking_space.width";
    const double parking_space_width = parking_node["width"].as<double>();

    context = "parking_space";
    geometry::OrientedRectangle2d parking_space(center, parking_space_heading,
                                                parking_space_length,
                                                parking_space_width);

    // 9. 创建环境
    context = "map / environment";
    map::Environment environment(origin, resolution, map_width, map_height,
                                 obstacles);

    // 10. 检查起终点完整车框是否碰撞或越界。
    context = "start";

    if (environment.hasCollision(start_pose, params)) {
      throw std::runtime_error(
          "vehicle collides with an obstacle or map boundary");
    }

    context = "goal";

    if (environment.hasCollision(goal_pose, params)) {
      throw std::runtime_error(
          "vehicle collides with an obstacle or map boundary");
    }

    // 11. 目标车辆必须完整位于车位内，允许接触车位边界。
    context = "goal / parking_space";
    const geometry::OrientedRectangle2d goal_footprint =
        params.footprint(goal_pose);

    for (const geometry::Vec2d &corner : goal_footprint.corners()) {
      if (!parking_space.isPointIn(corner)) {
        throw std::runtime_error(
            "goal vehicle footprint must fit inside parking_space");
      }
    }

    // return
    return Scenario{std::move(name), params,    std::move(environment),
                    start_pose,      goal_pose, parking_space};

  } catch (const YAML::Exception &error) {
    throw std::runtime_error("failed to load '" + file_path + "' at " +
                             context + ": " + error.what());
  } catch (const std::logic_error &error) {
    throw std::runtime_error("failed to load '" + file_path + "' at " +
                             context + ": " + error.what());
  } catch (const std::runtime_error &error) {
    throw std::runtime_error("failed to load '" + file_path + "' at " +
                             context + ": " + error.what());
  }
}

} // namespace parking_planner::core::scenario