#pragma once

#include <array>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <stdexcept>
#include <string>

#include "cartesian_impedance_controller/impedance_law.hpp"

namespace cartesian_impedance_controller
{
struct DebugSample
{
  std::int64_t time_ns{ 0 };
  std::uint64_t target_sequence{ 0 };
  std::array<double, 7> current{}, target{}, reference{};
  Vector6 error_raw{}, error{}, twist{}, spring{}, damper{}, wrench_raw{}, wrench{};
  Vector6 torque_task{}, coriolis{}, torque_raw{}, torque{};
  bool write_ok{ false };
};

// 实时线程只尝试复制固定大小的快照；磁盘操作由非实时定时器完成。
class DebugCsv
{
public:
  explicit DebugCsv(const std::string& path, const std::string& metadata) : stream_(path)
  {
    if (!stream_) {
      throw std::runtime_error("无法创建阻抗调试日志：" + path);
    }
    stream_ << metadata << "\ntime_ns,target_sequence";
    for (const auto* field : { "current", "target", "reference" }) {
      for (const auto* axis : { "x", "y", "z", "qx", "qy", "qz", "qw" }) {
        stream_ << ',' << field << '_' << axis;
      }
    }
    for (const auto* field : { "error_raw", "error", "twist", "spring", "damper", "wrench_raw", "wrench",
                               "torque_task", "coriolis", "torque_raw", "torque" }) {
      for (int i = 0; i < 6; ++i) {
        stream_ << ',' << field << '_' << i;
      }
    }
    stream_ << ",write_ok\n" << std::setprecision(17);
    stream_.flush();
  }

  void capture(const DebugSample& sample)
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      sample_ = sample;
      ++generation_;
    }
  }

  void flush_latest()
  {
    DebugSample sample;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (generation_ == saved_generation_) {
        return;
      }
      sample = sample_;
      saved_generation_ = generation_;
    }
    stream_ << sample.time_ns << ',' << sample.target_sequence;
    for (const auto* values : { &sample.current, &sample.target, &sample.reference }) {
      for (double value : *values) {
        stream_ << ',' << value;
      }
    }
    for (const auto* values : { &sample.error_raw, &sample.error, &sample.twist, &sample.spring,
                                &sample.damper, &sample.wrench_raw, &sample.wrench, &sample.torque_task,
                                &sample.coriolis, &sample.torque_raw, &sample.torque }) {
      for (double value : *values) {
        stream_ << ',' << value;
      }
    }
    stream_ << ',' << sample.write_ok << '\n';
    stream_.flush();
    if (!stream_) {
      throw std::runtime_error("阻抗调试日志写入失败");
    }
  }

private:
  std::ofstream stream_;
  std::mutex mutex_;
  DebugSample sample_;
  std::uint64_t generation_{ 0 }, saved_generation_{ 0 };
};
}  // namespace cartesian_impedance_controller
