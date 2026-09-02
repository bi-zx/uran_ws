#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <src/CYdLidar.h>

namespace {

double Percentile(std::vector<float> values, double ratio)
{
  if (values.empty()) {
    return -1.0;
  }
  std::sort(values.begin(), values.end());
  const std::size_t index = static_cast<std::size_t>(
    ratio * static_cast<double>(values.size() - 1));
  return values[index];
}

}  // namespace

int main()
{
  using namespace ydlidar;

  os_init();
  CYdLidar laser;

  std::string port = "/dev/ttyTHS0";
  int int_value = 512000;
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  laser.setlidaropt(LidarPropSerialBaudrate, &int_value, sizeof(int_value));

  int_value = TYPE_TOF;
  laser.setlidaropt(LidarPropLidarType, &int_value, sizeof(int_value));
  int_value = YDLIDAR_TYPE_SERIAL;
  laser.setlidaropt(LidarPropDeviceType, &int_value, sizeof(int_value));
  int_value = 10;
  laser.setlidaropt(LidarPropSampleRate, &int_value, sizeof(int_value));
  int_value = 4;
  laser.setlidaropt(LidarPropAbnormalCheckCount, &int_value, sizeof(int_value));

  bool bool_value = false;
  laser.setlidaropt(LidarPropFixedResolution, &bool_value, sizeof(bool_value));
  bool_value = true;
  laser.setlidaropt(LidarPropReversion, &bool_value, sizeof(bool_value));
  bool_value = false;
  laser.setlidaropt(LidarPropInverted, &bool_value, sizeof(bool_value));
  bool_value = true;
  laser.setlidaropt(LidarPropAutoReconnect, &bool_value, sizeof(bool_value));
  bool_value = false;
  laser.setlidaropt(LidarPropSingleChannel, &bool_value, sizeof(bool_value));
  bool_value = true;
  laser.setlidaropt(LidarPropIntenstiy, &bool_value, sizeof(bool_value));

  float float_value = 90.0f;
  laser.setlidaropt(LidarPropMaxAngle, &float_value, sizeof(float_value));
  float_value = -90.0f;
  laser.setlidaropt(LidarPropMinAngle, &float_value, sizeof(float_value));
  float_value = 30.0f;
  laser.setlidaropt(LidarPropMaxRange, &float_value, sizeof(float_value));
  float_value = 0.01f;
  laser.setlidaropt(LidarPropMinRange, &float_value, sizeof(float_value));
  float_value = 10.0f;
  laser.setlidaropt(LidarPropScanFrequency, &float_value, sizeof(float_value));

  if (!laser.initialize()) {
    std::cerr << "SDK initialize failed: " << laser.DescribeError() << std::endl;
    return 2;
  }
  if (!laser.turnOn()) {
    std::cerr << "SDK turnOn failed: " << laser.DescribeError() << std::endl;
    laser.disconnecting();
    return 3;
  }

  LaserScan scan;
  std::vector<float> front_ranges;
  std::vector<int> point_counts;
  std::vector<int> front_counts;
  int received = 0;
  int attempts = 0;
  const float front_limit = static_cast<float>(18.0 * M_PI / 180.0);

  while (received < 20 && attempts < 60 && os_isOk()) {
    ++attempts;
    if (!laser.doProcessSimple(scan)) {
      continue;
    }

    ++received;
    point_counts.push_back(static_cast<int>(scan.points.size()));
    int front_count = 0;
    for (const auto & point : scan.points) {
      if (
        std::abs(point.angle) > front_limit ||
        !std::isfinite(point.range) || point.range <= 0.0f ||
        point.range < scan.config.min_range || point.range > scan.config.max_range ||
        point.intensity <= 0.0f)
      {
        continue;
      }
      ++front_count;
      front_ranges.push_back(point.range);
    }
    front_counts.push_back(front_count);
  }

  laser.turnOff();
  laser.disconnecting();
  os_shutdown();

  std::cout << "received_frames: " << received << std::endl;
  if (received == 0 || front_ranges.empty()) {
    std::cout << "no trusted front returns" << std::endl;
    return received == 0 ? 4 : 0;
  }

  std::sort(point_counts.begin(), point_counts.end());
  std::sort(front_counts.begin(), front_counts.end());
  std::cout << "sdk_points_per_frame_median: "
            << point_counts[point_counts.size() / 2] << std::endl;
  std::cout << "trusted_front_points_per_frame_median: "
            << front_counts[front_counts.size() / 2] << std::endl;
  std::cout << "trusted_front_min_m: " << Percentile(front_ranges, 0.0) << std::endl;
  std::cout << "trusted_front_p10_m: " << Percentile(front_ranges, 0.1) << std::endl;
  std::cout << "trusted_front_median_m: " << Percentile(front_ranges, 0.5) << std::endl;
  std::cout << "trusted_front_le_0_65: "
            << std::count_if(
    front_ranges.begin(), front_ranges.end(),
    [](float value) {return value <= 0.65f;}) << std::endl;
  std::cout << "trusted_front_1_2_to_2_2: "
            << std::count_if(
    front_ranges.begin(), front_ranges.end(),
    [](float value) {return value >= 1.2f && value <= 2.2f;}) << std::endl;
  return 0;
}
