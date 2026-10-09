#include <gtest/gtest.h>

#include <algorithm>
#include <vector>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unistd.h>

#include "cartesian_impedance_controller/debug_csv.hpp"

TEST(DebugCsv, SavesLatestCompleteSampleWithoutDuplicateRows)
{
  // 使用临时文件验证快照覆盖、字段对齐及落盘；不连接控制器或真机。
  const auto path = std::filesystem::temp_directory_path() /
      ("cartesian-debug-test-" + std::to_string(getpid()) + ".csv");
  {
    cartesian_impedance_controller::DebugCsv csv(path.string(), "# test");
    cartesian_impedance_controller::DebugSample sample;
    sample.time_ns = 123;
    sample.target_sequence = 8;
    sample.error_raw[2] = .02;
    sample.error[2] = .008;
    sample.torque_raw[0] = 80;
    sample.torque[0] = 60;
    sample.write_ok = true;
    csv.capture(sample);
    sample.target_sequence = 9;
    csv.capture(sample);
    csv.flush_latest();
    csv.flush_latest();
  }
  std::ifstream stream(path);
  std::string metadata, header, row, extra;
  ASSERT_TRUE(static_cast<bool>(std::getline(stream, metadata)));
  ASSERT_TRUE(static_cast<bool>(std::getline(stream, header)));
  ASSERT_TRUE(static_cast<bool>(std::getline(stream, row)));
  EXPECT_FALSE(static_cast<bool>(std::getline(stream, extra)));
  const auto split = [](const std::string& line) {
    std::vector<std::string> result;
    std::istringstream input(line);
    std::string field;
    while (std::getline(input, field, ',')) { result.push_back(field); }
    return result;
  };
  const auto keys = split(header), values = split(row);
  ASSERT_EQ(keys.size(), values.size());
  EXPECT_EQ(values[0], "123");
  EXPECT_EQ(values[1], "9");
  const auto value = [&](const std::string& key) {
    const auto found = std::find(keys.begin(), keys.end(), key);
    EXPECT_NE(found, keys.end());
    return std::stod(values.at(std::distance(keys.begin(), found)));
  };
  EXPECT_DOUBLE_EQ(value("error_raw_2"), .02);
  EXPECT_DOUBLE_EQ(value("error_2"), .008);
  EXPECT_DOUBLE_EQ(value("torque_raw_0"), 80);
  EXPECT_DOUBLE_EQ(value("torque_0"), 60);
  EXPECT_DOUBLE_EQ(value("write_ok"), 1);
  std::filesystem::remove(path);
}
