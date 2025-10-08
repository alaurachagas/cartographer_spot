#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <cstring>   // memcpy

using sensor_msgs::msg::PointCloud2;

enum class Mode { ZERO, MONOTONIC, SHIFT, SORTED };

/* ---------- existing simple modes ---------- */

static void set_time_zero(PointCloud2 & msg) {
  for (sensor_msgs::PointCloud2Iterator<float> it(msg, "time"); it != it.end(); ++it)
    *it = 0.0f;
}

static void write_monotonic_negative_time(PointCloud2 & msg) {
  const size_t n = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  if (!n) return;
  const float step = 1e-6f;
  size_t idx = 0;
  for (sensor_msgs::PointCloud2Iterator<float> it(msg, "time"); it != it.end(); ++it, ++idx)
    *it = -step * static_cast<float>((n - 1) - idx);
}

static void shift_last_to_zero(PointCloud2 & msg) {
  const size_t n = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  if (!n) return;
  sensor_msgs::PointCloud2ConstIterator<float> it_last(msg, "time");
  it_last += (n - 1);
  const float last_t = *it_last;
  for (sensor_msgs::PointCloud2Iterator<float> it(msg, "time"); it != it.end(); ++it)
    *it -= last_t;
}

/* ---------- helpers for SORTED mode ---------- */

struct FieldInfo { int offset = -1; int datatype = 0; };

static bool find_field(const PointCloud2 &msg, const std::string &name, FieldInfo &out) {
  for (const auto &f : msg.fields) {
    if (f.name == name) { out.offset = f.offset; out.datatype = f.datatype; return true; }
  }
  return false;
}

static inline float read_float32(const uint8_t* p) {
  float v; std::memcpy(&v, p, sizeof(float)); return v;
}
static inline uint16_t read_uint16(const uint8_t* p) {
  uint16_t v; std::memcpy(&v, p, sizeof(uint16_t)); return v;
}

struct SortPlan {
  FieldInfo x, y, ring, timef;
  bool has_ring=false, has_time=false;
  uint32_t point_step=0;
  size_t npts=0;
};

static bool build_sort_plan(const PointCloud2 &msg, SortPlan &sp) {
  sp.point_step = msg.point_step;
  sp.npts = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  bool okx = find_field(msg, "x", sp.x);
  bool oky = find_field(msg, "y", sp.y);
  sp.has_ring = find_field(msg, "ring", sp.ring);
  sp.has_time = find_field(msg, "time", sp.timef);
  return okx && oky && sp.npts>0 && sp.point_step>0;
}

static inline double azimuth_of(const SortPlan& sp, const uint8_t* base) {
  float x = read_float32(base + sp.x.offset);
  float y = read_float32(base + sp.y.offset);
  double a = std::atan2(static_cast<double>(y), static_cast<double>(x));
  if (a < 0.0) a += 2.0*M_PI;
  return a;
}

static void sort_by_scan_order(PointCloud2 &msg) {
  SortPlan sp;
  if (!build_sort_plan(msg, sp)) return;

  // indices 0..n-1
  std::vector<size_t> idx(sp.npts);
  std::iota(idx.begin(), idx.end(), 0);

  const auto* data = msg.data.data();

  // comparator: (ring, azimuth) if ring exists, else azimuth
  auto cmp = [&](size_t i, size_t j){
    const uint8_t* pi = data + i*sp.point_step;
    const uint8_t* pj = data + j*sp.point_step;
    if (sp.has_ring) {
      uint16_t ri = read_uint16(pi + sp.ring.offset);
      uint16_t rj = read_uint16(pj + sp.ring.offset);
      if (ri != rj) return ri < rj;
    }
    return azimuth_of(sp, pi) < azimuth_of(sp, pj);
  };

  std::stable_sort(idx.begin(), idx.end(), cmp);

  // reorder full record (all fields) by point_step
  std::vector<uint8_t> out(msg.data.size());
  for (size_t new_i=0; new_i<sp.npts; ++new_i) {
    size_t old_i = idx[new_i];
    std::memcpy(out.data() + new_i*sp.point_step,
                data       + old_i*sp.point_step, sp.point_step);
  }
  msg.data.swap(out);
}

static void write_scaled_times(PointCloud2 &msg, double scan_period_sec) {
  FieldInfo tf;
  if (!find_field(msg, "time", tf)) return;
  const size_t n = static_cast<size_t>(msg.width) * msg.height;
  if (n < 2) return;
  const float denom = static_cast<float>(n - 1);
  for (size_t i=0; i<n; ++i) {
    float t = -static_cast<float>(scan_period_sec) * static_cast<float>((n-1)-i) / denom;
    std::memcpy(msg.data.data() + i*msg.point_step + tf.offset, &t, sizeof(float));
  }
}

/* ---------- node ---------- */

class Pc2TimeNormalizer : public rclcpp::Node {
public:
  Pc2TimeNormalizer() : rclcpp::Node("pc2_time_normalizer") {
    input_  = declare_parameter<std::string>("input",  "/velodyne_points_decoded");
    output_ = declare_parameter<std::string>("output", "/velodyne_points_cartographer");
    const std::string mode_s = declare_parameter<std::string>("mode", "zero"); // zero|monotonic|shift|sorted
    if      (mode_s == "monotonic") mode_ = Mode::MONOTONIC;
    else if (mode_s == "shift")     mode_ = Mode::SHIFT;
    else if (mode_s == "sorted")    mode_ = Mode::SORTED;
    else                            mode_ = Mode::ZERO;

    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)); sub_qos.reliable(); // match decoded publisher
    auto pub_qos = rclcpp::SensorDataQoS(); // best-effort to Cartographer

    pub_ = create_publisher<PointCloud2>(output_, pub_qos);
    sub_ = create_subscription<PointCloud2>(input_, sub_qos,
      [this](PointCloud2::UniquePtr msg){
        bool has_time = false;
        for (const auto& f : msg->fields) if (f.name == "time") { has_time = true; break; }

        if (has_time) {
          switch (mode_) {
            case Mode::ZERO:
              set_time_zero(*msg);
              break;
            case Mode::MONOTONIC:
              write_monotonic_negative_time(*msg);
              break;
            case Mode::SHIFT:
              shift_last_to_zero(*msg);
              break;
            case Mode::SORTED: {
              // 1) sort by scan order (ring, azimuth)
              sort_by_scan_order(*msg);
              // 2) estimate scan period from header delta, clamp to [0.05, 0.20] s
              const rclcpp::Time now = msg->header.stamp;
              double dt = 0.1; // default 10 Hz
              if (have_prev_) {
                dt = (now - prev_stamp_).seconds();
                if (dt < 0.05) dt = 0.05;
                if (dt > 0.20) dt = 0.20;
              }
              prev_stamp_ = now; have_prev_ = true;
              // 3) write scaled per-point times (last = 0)
              write_scaled_times(*msg, dt);
              break;
            }
          }
        }

        static int cnt=0;
        if ((cnt++ % 30)==0 && has_time) {
          sensor_msgs::PointCloud2ConstIterator<float> it0(*msg, "time");
          sensor_msgs::PointCloud2ConstIterator<float> itL(*msg, "time"); itL += (msg->width*msg->height - 1);
          RCLCPP_INFO(get_logger(), "time[0]=%.6g time[last]=%.6g (mode=%d)",
                      *it0, *itL, static_cast<int>(mode_));
        }
        pub_->publish(std::move(msg));
      }
    );

    RCLCPP_INFO(get_logger(), "Normalizing: %s -> %s (mode=%s)",
                input_.c_str(), output_.c_str(),
                (mode_==Mode::ZERO?"zero":mode_==Mode::MONOTONIC?"monotonic":mode_==Mode::SHIFT?"shift":"sorted"));
  }

private:
  Mode mode_;
  std::string input_, output_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
  rclcpp::Time prev_stamp_;
  bool have_prev_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pc2TimeNormalizer>());
  rclcpp::shutdown();
  return 0;
}
