#include "parking_planner/core/scenario/scenario_loader.h"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>

#include "parking_planner/core/geometry/angle.h"
#include "parking_planner/core/geometry/vec2d.h"

namespace parking_planner::core::scenario {
namespace {

constexpr double kTolerance = 1e-9;

// 独立的小场景，用于每次只修改一项的错误测试
// 车长 2 米，几何中心在后轴前方 0.5 米
// 目标后轴 (6, 6)，对应车框中心 (6.5, 6)
constexpr const char *kValidYaml = R"(
name: loader_test

map:
  origin: [0.0, 0.0]
  resolution: 0.5
  width: 20
  height: 20

vehicle:
  wheelbase: 1.0
  width: 1.0
  front_overhang: 0.5
  rear_overhang: 0.5
  max_steering_angle: 0.6

start:
  x: 2.0
  y: 2.0
  theta: 0.0

goal:
  x: 6.0
  y: 6.0
  theta: 0.0

parking_space:
  center: [6.5, 6.0]
  heading: 0.0
  length: 3.0
  width: 2.0

obstacles:
  - vertices:
      - [4.0, 4.0]
      - [5.0, 4.0]
      - [5.0, 5.0]
      - [4.0, 5.0]
)";

// TEST_F 中的每个测试都会获得新的数据和临时目录
class ScenarioLoaderTest : public ::testing::Test {
protected:
  void SetUp() override {
    data_ = YAML::Load(kValidYaml);

    const std::string pattern = (std::filesystem::temp_directory_path() /
                                 "parking_scenario_test_XXXXXX")
                                    .string();

    // mkdtemp 修改字符串末尾的 X，创建一个不与其他测试冲突的目录
    std::string directory_template = pattern;
    const char *directory = ::mkdtemp(directory_template.data());

    ASSERT_NE(nullptr, directory);

    directory_ = directory;
    file_path_ = (directory_ / "scene.yaml").string();
  }

  void TearDown() override {
    if (!directory_.empty()) {
      std::error_code error;
      std::filesystem::remove_all(directory_, error);
    }
  }

  void writeInput() {
    std::ofstream output(file_path_);

    if (!output) {
      throw std::runtime_error("cannot create temporary YAML file");
    }

    output << data_;
    output.close();

    if (!output) {
      throw std::runtime_error("cannot write temporary YAML file");
    }
  }

  // 除了抛 runtime_error，还检查错误是否包含文件和处理位置
  void expectRejected(const std::string &expected_context) {
    writeInput();

    try {
      const auto scene = loadScenarioFromFile(file_path_);
      (void)scene;
      FAIL() << "Expected the scene to be rejected";
    } catch (const std::runtime_error &error) {
      const std::string message = error.what();

      EXPECT_NE(std::string::npos, message.find(file_path_)) << message;
      EXPECT_NE(std::string::npos, message.find(expected_context)) << message;
    }
  }

  YAML::Node data_;
  std::filesystem::path directory_;
  std::string file_path_;
};

TEST(ScenarioLoaderResourceTest, LoadsPerpendicularParkingScene) {
  const std::string file_path =
      std::string(PARKING_SCENARIO_DIR) + "/perpendicular_parking.yaml";

  const Scenario scene = loadScenarioFromFile(file_path);

  EXPECT_EQ("perpendicular_parking", scene.name);

  EXPECT_DOUBLE_EQ(2.8, scene.vehicle.wheelbase());
  EXPECT_DOUBLE_EQ(1.8, scene.vehicle.width());
  EXPECT_DOUBLE_EQ(1.0, scene.vehicle.frontOverhang());
  EXPECT_DOUBLE_EQ(1.0, scene.vehicle.rearOverhang());
  EXPECT_DOUBLE_EQ(0.6, scene.vehicle.maxSteeringAngle());
  EXPECT_NEAR(4.8, scene.vehicle.length(), kTolerance);

  const auto &grid = scene.environment.grid();

  EXPECT_DOUBLE_EQ(0.0, grid.origin().x());
  EXPECT_DOUBLE_EQ(0.0, grid.origin().y());
  EXPECT_DOUBLE_EQ(0.2, grid.resolution());
  EXPECT_EQ(100, grid.width());
  EXPECT_EQ(80, grid.height());
  EXPECT_NEAR(20.0, grid.maxX(), kTolerance);
  EXPECT_NEAR(16.0, grid.maxY(), kTolerance);

  EXPECT_DOUBLE_EQ(4.0, scene.start.x);
  EXPECT_DOUBLE_EQ(11.0, scene.start.y);
  EXPECT_DOUBLE_EQ(0.0, scene.start.theta);

  EXPECT_DOUBLE_EQ(10.0, scene.goal.x);
  EXPECT_DOUBLE_EQ(2.6, scene.goal.y);
  EXPECT_NEAR(geometry::kPi / 2.0, scene.goal.theta, kTolerance);

  EXPECT_DOUBLE_EQ(10.0, scene.parking_space.center().x());
  EXPECT_DOUBLE_EQ(4.0, scene.parking_space.center().y());
  EXPECT_NEAR(geometry::kPi / 2.0, scene.parking_space.heading(), kTolerance);
  EXPECT_DOUBLE_EQ(5.6, scene.parking_space.length());
  EXPECT_DOUBLE_EQ(2.8, scene.parking_space.width());

  const auto &obstacles = scene.environment.obstacles();

  ASSERT_EQ(2u, obstacles.size());
  ASSERT_EQ(4u, obstacles[0].vertices().size());
  ASSERT_EQ(4u, obstacles[1].vertices().size());

  EXPECT_DOUBLE_EQ(5.0, obstacles[0].vertices()[0].x());
  EXPECT_DOUBLE_EQ(1.0, obstacles[0].vertices()[0].y());
  EXPECT_DOUBLE_EQ(12.0, obstacles[1].vertices()[0].x());
  EXPECT_DOUBLE_EQ(1.0, obstacles[1].vertices()[0].y());

  // 后轴中心和车框中心不能混淆
  const auto goal_footprint = scene.vehicle.footprint(scene.goal);

  EXPECT_NEAR(10.0, goal_footprint.center().x(), kTolerance);
  EXPECT_NEAR(4.0, goal_footprint.center().y(), kTolerance);

  // 验证加载后的障碍物确实生成了占用格
  const auto occupied_index = grid.worldToGrid(geometry::Vec2d(6.0, 2.0));

  ASSERT_TRUE(occupied_index.has_value());
  EXPECT_TRUE(grid.isOccupied(*occupied_index));
}

TEST_F(ScenarioLoaderTest, LoadsSceneWithoutObstacles) {
  data_["obstacles"] = YAML::Load("[]");
  writeInput();

  const Scenario scene = loadScenarioFromFile(file_path_);

  EXPECT_EQ("loader_test", scene.name);
  EXPECT_TRUE(scene.environment.obstacles().empty());
}

TEST_F(ScenarioLoaderTest, RejectsEmptyFilePath) {
  EXPECT_THROW(loadScenarioFromFile(""), std::runtime_error);
}

TEST_F(ScenarioLoaderTest, RejectsMissingFile) {
  // 临时目录已存在，但尚未创建 scene.yaml
  EXPECT_THROW(loadScenarioFromFile(file_path_), std::runtime_error);
}

TEST_F(ScenarioLoaderTest, RejectsMalformedYaml) {
  {
    std::ofstream output(file_path_);
    ASSERT_TRUE(output);
    output << "map: [";
  }

  EXPECT_THROW(loadScenarioFromFile(file_path_), std::runtime_error);
}

TEST_F(ScenarioLoaderTest, RejectsSequenceAtRoot) {
  data_ = YAML::Load("[1, 2]");
  expectRejected("root");
}

TEST_F(ScenarioLoaderTest, RejectsWhitespaceOnlyName) {
  data_["name"] = " \t\n";
  expectRejected("name");
}

TEST_F(ScenarioLoaderTest, RejectsMissingRequiredField) {
  data_["vehicle"].remove("wheelbase");
  expectRejected("vehicle.wheelbase");
}

TEST_F(ScenarioLoaderTest, RejectsNullRequiredField) {
  data_["vehicle"]["wheelbase"] = YAML::Node(YAML::NodeType::Null);
  expectRejected("vehicle.wheelbase");
}

TEST_F(ScenarioLoaderTest, RejectsWrongScalarType) {
  data_["vehicle"]["wheelbase"] = "not_a_number";
  expectRejected("vehicle.wheelbase");
}

TEST_F(ScenarioLoaderTest, RejectsInvalidMapStructure) {
  data_["map"] = 123;
  expectRejected("map");
}

TEST_F(ScenarioLoaderTest, RejectsOriginWithWrongCoordinateCount) {
  data_["map"]["origin"] = YAML::Load("[0.0]");
  expectRejected("map.origin");
}

TEST_F(ScenarioLoaderTest, RejectsNonIntegerGridWidth) {
  data_["map"]["width"] = 1.5;
  expectRejected("map.width");
}

TEST_F(ScenarioLoaderTest, RejectsInvalidGridDimensions) {
  for (const int width : {0, -1}) {
    SCOPED_TRACE(width);
    data_["map"]["width"] = width;
    expectRejected("map / environment");
  }
}

TEST_F(ScenarioLoaderTest, RejectsInvalidResolution) {
  for (const double resolution : {0.0, -0.5}) {
    SCOPED_TRACE(resolution);
    data_["map"]["resolution"] = resolution;
    expectRejected("map / environment");
  }
}

TEST_F(ScenarioLoaderTest, RejectsNegativeVehicleWidth) {
  data_["vehicle"]["width"] = -1.0;
  expectRejected("vehicle");
}

TEST_F(ScenarioLoaderTest, RejectsNonFiniteValues) {
  data_["vehicle"]["wheelbase"] = ".inf";
  expectRejected("vehicle");

  data_ = YAML::Load(kValidYaml);
  data_["goal"]["theta"] = ".nan";
  expectRejected("goal");

  data_ = YAML::Load(kValidYaml);
  data_["map"]["origin"] = YAML::Load("[.nan, 0.0]");
  expectRejected("map.origin");
}

TEST_F(ScenarioLoaderTest, RejectsObstacleListWithWrongStructure) {
  data_["obstacles"] = YAML::Load("{}");
  expectRejected("obstacles");
}

TEST_F(ScenarioLoaderTest, RejectsObstacleWithWrongStructure) {
  data_["obstacles"][0] = YAML::Load("[1, 2]");
  expectRejected("obstacles[0]");
}

TEST_F(ScenarioLoaderTest, RejectsVertexWithWrongCoordinateCount) {
  data_["obstacles"][0]["vertices"][0] = YAML::Load("[4.0]");
  expectRejected("obstacles[0].vertices[0]");
}

TEST_F(ScenarioLoaderTest, RejectsConcaveObstacle) {
  data_["obstacles"][0]["vertices"] =
      YAML::Load("[[4.0, 4.0], [5.0, 4.0], [4.4, 4.4], "
                 "[5.0, 5.0], [4.0, 5.0]]");

  expectRejected("obstacles[0]");
}

TEST_F(ScenarioLoaderTest, RejectsStartVehicleCollision) {
  data_["start"]["x"] = 4.0;
  data_["start"]["y"] = 4.5;
  expectRejected("start");
}

TEST_F(ScenarioLoaderTest, RejectsGoalVehicleCollision) {
  data_["goal"]["x"] = 4.0;
  data_["goal"]["y"] = 4.5;
  expectRejected("goal");
}

TEST_F(ScenarioLoaderTest, RejectsMapBoundaryContactAndOutside) {
  // 后悬为 0.5 米，因此 x=0.5 时车尾恰好接触地图左边界
  for (const double x : {0.5, 0.4}) {
    SCOPED_TRACE(x);
    data_["start"]["x"] = x;
    expectRejected("start");
  }
}

TEST_F(ScenarioLoaderTest, RejectsGoalOutsideParkingSpace) {
  // 车辆仍在地图内且不碰障碍，但不在目标车位内
  data_["goal"]["x"] = 8.0;
  expectRejected("goal / parking_space");
}

TEST_F(ScenarioLoaderTest, RejectsNegativeParkingLength) {
  data_["parking_space"]["length"] = -1.0;
  expectRejected("parking_space");
}

TEST_F(ScenarioLoaderTest, AllowsVehicleToTouchParkingSpaceBoundary) {
  // 车框长 2 米、宽 1 米，车位缩到完全相同的尺寸
  data_["parking_space"]["length"] = 2.0;
  data_["parking_space"]["width"] = 1.0;
  writeInput();

  EXPECT_NO_THROW(loadScenarioFromFile(file_path_));
}

TEST_F(ScenarioLoaderTest, AllowsGoalCenterOffsetInsideParkingSpace) {
  // 车位中心向右移 0.2 米，车辆仍完整位于车位中
  data_["parking_space"]["center"][0] = 6.7;
  writeInput();

  EXPECT_NO_THROW(loadScenarioFromFile(file_path_));
}

TEST(ScenarioLoaderResourceTest, LoadsParallelParkingScene) {
  const std::string file_path =
      std::string(PARKING_SCENARIO_DIR) + "/parallel_parking.yaml";

  const Scenario scene = loadScenarioFromFile(file_path);

  EXPECT_EQ("parallel_parking", scene.name);

  const auto &grid = scene.environment.grid();

  EXPECT_DOUBLE_EQ(0.2, grid.resolution());
  EXPECT_EQ(100, grid.width());
  EXPECT_EQ(80, grid.height());

  EXPECT_DOUBLE_EQ(4.0, scene.start.x);
  EXPECT_DOUBLE_EQ(11.0, scene.start.y);
  EXPECT_DOUBLE_EQ(0.0, scene.start.theta);

  EXPECT_DOUBLE_EQ(9.6, scene.goal.x);
  EXPECT_DOUBLE_EQ(4.0, scene.goal.y);
  EXPECT_DOUBLE_EQ(0.0, scene.goal.theta);

  // 后轴中心在 (9.6, 4)，车框中心在其前方 1.4 米
  const auto footprint = scene.vehicle.footprint(scene.goal);

  EXPECT_NEAR(11.0, footprint.center().x(), kTolerance);
  EXPECT_NEAR(4.0, footprint.center().y(), kTolerance);

  EXPECT_DOUBLE_EQ(11.0, scene.parking_space.center().x());
  EXPECT_DOUBLE_EQ(4.0, scene.parking_space.center().y());
  EXPECT_DOUBLE_EQ(0.0, scene.parking_space.heading());
  EXPECT_DOUBLE_EQ(5.6, scene.parking_space.length());
  EXPECT_DOUBLE_EQ(2.8, scene.parking_space.width());

  const auto &obstacles = scene.environment.obstacles();

  ASSERT_EQ(3u, obstacles.size());

  for (const auto &obstacle : obstacles) {
    ASSERT_EQ(4u, obstacle.vertices().size());
  }

  // 左侧车辆、右侧车辆、下方路沿
  EXPECT_DOUBLE_EQ(2.0, obstacles[0].vertices()[0].x());
  EXPECT_DOUBLE_EQ(3.0, obstacles[0].vertices()[0].y());
  EXPECT_DOUBLE_EQ(7.0, obstacles[0].vertices()[2].x());
  EXPECT_DOUBLE_EQ(5.0, obstacles[0].vertices()[2].y());

  EXPECT_DOUBLE_EQ(15.0, obstacles[1].vertices()[0].x());
  EXPECT_DOUBLE_EQ(3.0, obstacles[1].vertices()[0].y());
  EXPECT_DOUBLE_EQ(19.0, obstacles[1].vertices()[2].x());
  EXPECT_DOUBLE_EQ(5.0, obstacles[1].vertices()[2].y());

  EXPECT_DOUBLE_EQ(1.0, obstacles[2].vertices()[0].x());
  EXPECT_DOUBLE_EQ(1.0, obstacles[2].vertices()[0].y());
  EXPECT_DOUBLE_EQ(19.0, obstacles[2].vertices()[2].x());
  EXPECT_DOUBLE_EQ(2.0, obstacles[2].vertices()[2].y());

  // 路沿内部必须占用，目标车位中心应为空闲
  const auto curb_index = grid.worldToGrid(geometry::Vec2d(10.0, 1.5));
  const auto parking_index = grid.worldToGrid(geometry::Vec2d(11.0, 4.0));

  ASSERT_TRUE(curb_index.has_value());
  ASSERT_TRUE(parking_index.has_value());

  EXPECT_TRUE(grid.isOccupied(*curb_index));
  EXPECT_FALSE(grid.isOccupied(*parking_index));
}

TEST(ScenarioLoaderResourceTest, LoadsNarrowPassageScene) {
  const std::string file_path =
      std::string(PARKING_SCENARIO_DIR) + "/narrow_passage.yaml";

  const Scenario scene = loadScenarioFromFile(file_path);

  EXPECT_EQ("narrow_passage", scene.name);

  const auto &grid = scene.environment.grid();

  EXPECT_DOUBLE_EQ(0.2, grid.resolution());
  EXPECT_EQ(100, grid.width());
  EXPECT_EQ(80, grid.height());

  EXPECT_DOUBLE_EQ(4.0, scene.start.x);
  EXPECT_DOUBLE_EQ(7.5, scene.start.y);
  EXPECT_DOUBLE_EQ(0.0, scene.start.theta);

  EXPECT_DOUBLE_EQ(10.0, scene.goal.x);
  EXPECT_DOUBLE_EQ(7.5, scene.goal.y);
  EXPECT_NEAR(geometry::kPi, scene.goal.theta, kTolerance);

  // 车头朝左，几何中心位于后轴中心的世界 -x 方向
  const auto footprint = scene.vehicle.footprint(scene.goal);

  EXPECT_NEAR(8.6, footprint.center().x(), kTolerance);
  EXPECT_NEAR(7.5, footprint.center().y(), kTolerance);

  EXPECT_DOUBLE_EQ(8.6, scene.parking_space.center().x());
  EXPECT_DOUBLE_EQ(7.5, scene.parking_space.center().y());
  EXPECT_NEAR(geometry::kPi, scene.parking_space.heading(), kTolerance);
  EXPECT_DOUBLE_EQ(5.6, scene.parking_space.length());
  EXPECT_DOUBLE_EQ(2.8, scene.parking_space.width());

  const auto &obstacles = scene.environment.obstacles();

  ASSERT_EQ(2u, obstacles.size());

  for (const auto &obstacle : obstacles) {
    ASSERT_EQ(4u, obstacle.vertices().size());
  }

  // 下方障碍物到 y=5，上方障碍物从 y=10 开始
  EXPECT_DOUBLE_EQ(0.0, obstacles[0].vertices()[0].x());
  EXPECT_DOUBLE_EQ(0.0, obstacles[0].vertices()[0].y());
  EXPECT_DOUBLE_EQ(13.0, obstacles[0].vertices()[2].x());
  EXPECT_DOUBLE_EQ(5.0, obstacles[0].vertices()[2].y());

  EXPECT_DOUBLE_EQ(0.0, obstacles[1].vertices()[0].x());
  EXPECT_DOUBLE_EQ(10.0, obstacles[1].vertices()[0].y());
  EXPECT_DOUBLE_EQ(13.0, obstacles[1].vertices()[2].x());
  EXPECT_DOUBLE_EQ(16.0, obstacles[1].vertices()[2].y());

  const auto lower_index = grid.worldToGrid(geometry::Vec2d(6.0, 2.0));
  const auto upper_index = grid.worldToGrid(geometry::Vec2d(6.0, 12.0));
  const auto passage_index = grid.worldToGrid(geometry::Vec2d(6.0, 7.5));
  const auto open_area_index = grid.worldToGrid(geometry::Vec2d(16.0, 3.0));

  ASSERT_TRUE(lower_index.has_value());
  ASSERT_TRUE(upper_index.has_value());
  ASSERT_TRUE(passage_index.has_value());
  ASSERT_TRUE(open_area_index.has_value());

  EXPECT_TRUE(grid.isOccupied(*lower_index));
  EXPECT_TRUE(grid.isOccupied(*upper_index));
  EXPECT_FALSE(grid.isOccupied(*passage_index));
  EXPECT_FALSE(grid.isOccupied(*open_area_index));
}

} // namespace
} // namespace parking_planner::core::scenario