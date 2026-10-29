/**
 * patchworkpp_node.cpp —— 实车最小地面分割节点 (GPU ROI 裁剪 + patchwork++)
 *
 * 为什么要有这个节点
 * ------------------
 * 实车上 patchwork++ 被包在 obs_fitting 包里, 而 obs_fitting 的 main 节点跑的是
 * 一整条重管线:
 *     点云 -> GPU 裁剪/降采样 -> patchwork++ 地面分割 -> GPU 二次降采样
 *          -> CUDA 欧式聚类 -> 矩形拟合 -> 椭圆拟合 -> SAT 重叠合并 -> TTL 跟踪
 * ego-planner 只需要第一步的 nonground 点云。本节点只做
 * 「GPU ROI 裁剪 -> 地面分割 -> 发 ground/nonground」, **不改 obs_fitting 一行代码**,
 * 复用它的 patchworkpp.hpp(header-only) 和 libcudafilter.so。
 *
 * ROI 为什么重要 (不只是省算力)
 * -----------------------------
 * patchworkpp.hpp:1026-1045 的 pc2czm() 里:
 *     if ((r <= max_range_) && (r > min_range_))  { 正常分区做地面估计 }
 *     else                                        { cloud_nonground.push_back(pt); }
 * 即**半径小于 min_r(默认 1.5m) 或大于 max_r 的点, 一律被当成障碍物**。
 * 车身自身被雷达扫到的点全落在 min_r 以内, 于是车周围形成一圈假障碍
 * (teb_tracking_params.yaml 里"车身自干扰点形成 ~1m 固定点环"说的就是这个),
 * 经 grid_map 的 obstacles_inflation(0.5m) 膨胀后规划器会以为自己被围死。
 *
 * GPU / CPU 分工
 * --------------
 * GPU (libcudafilter.so, 与 obs_fitting 用的是同一个库, 同样的 PASSTHROUGH 链):
 *     x / y / z 三次轴对齐裁剪 + 可选 VOXELGRID 降采样 —— 干掉绝大部分点
 * CPU (只处理 GPU 裁完剩下的少量点, 通常几万点, 耗时可忽略):
 *     车身框剔除、min_radius 剔除、以及偏航角非 90 度整数倍时的精确旋转框细化
 * 原因: cudaFilter 的 PASSTHROUGH 只能做**轴对齐**单维区间, 无法直接表达
 *       「旋转矩形」和「挖掉一个盒子」(挖盒子是 6 个半空间的 OR, 不是 AND)。
 *
 * 没有 CUDA 时(比如 x86 仿真机)自动退化为纯 CPU 路径, 结果完全一致。
 *
 * 坐标约定
 * --------
 * ROI 参数一律用**车体语义**(前/后/左/右)填写, 四个方向都填正数表示该方向的距离。
 * 节点内部按 lidar_yaw_offset_deg 换算, 你不需要关心雷达怎么装。
 * 本车实测: 雷达 +x = 车辆左侧, +y = 车辆正后方, 故 lidar_yaw_offset_deg = -90。
 * 输出点云保持**原始雷达系坐标**(只筛选不改坐标), 下游各自用 TF 变换到 odom。
 *
 * 话题
 * ----
 * 订阅: ~cloud_topic (默认 /velodyne_points)
 * 发布: ~ground, ~nonground, ~roi (裁剪后送进 patchwork 的点云, 调 ROI 时看这个)
 */

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#ifdef USE_CUDA_FILTER
#include <cuda_runtime.h>
#include "cudaFilter.h"
#endif

#include <patchworkpp/patchworkpp.hpp>

using PointType = pcl::PointXYZI;

namespace
{

// 车体系下的一个轴对齐盒子, 四个方向都填正的距离值
struct Box
{
  bool enable = true;
  double front = 0.0, back = 0.0, left = 0.0, right = 0.0;
  double z_min = 0.0, z_max = 0.0;

  // xv 朝车头, yv 朝车左, z 为雷达系 z(朝上, 不受偏航影响)
  bool contains(double xv, double yv, double z) const
  {
    return xv <= front && xv >= -back && yv <= left && yv >= -right &&
           z >= z_min && z <= z_max;
  }

  void load(ros::NodeHandle &nh, const std::string &ns, bool default_enable)
  {
    nh.param(ns + "/enable", enable, default_enable);
    nh.param(ns + "/front", front, front);
    nh.param(ns + "/back", back, back);
    nh.param(ns + "/left", left, left);
    nh.param(ns + "/right", right, right);
    nh.param(ns + "/z_min", z_min, z_min);
    nh.param(ns + "/z_max", z_max, z_max);
  }
};

ros::Publisher g_pub_ground, g_pub_nonground, g_pub_roi;
boost::shared_ptr<PatchWorkpp<PointType>> g_seg;

Box g_roi;
Box g_self;
double g_min_radius = 0.0;
// 以**车辆参考点**(后轴中心)为圆心的净空半径。见 main() 里的说明。
double g_ref_clear_r = 0.0;
double g_ref_offset_x = 0.0;
double g_voxel_leaf = 0.0;
double g_cos_yaw = 1.0, g_sin_yaw = 0.0;
bool g_yaw_is_axis_aligned = true;   // 偏航角是 90 度整数倍 -> AABB 即精确框
bool g_use_gpu = false;

// 旋转后 ROI 在雷达系下的轴对齐包围盒(GPU 用)
double g_aabb_x_min = 0.0, g_aabb_x_max = 0.0;
double g_aabb_y_min = 0.0, g_aabb_y_max = 0.0;

/* ------------------------------------------------------------------ */
/*  GPU 裁剪: 与 obs_fitting 的 gpuCropAndDownsample 相同的 PASSTHROUGH 链  */
/* ------------------------------------------------------------------ */
#ifdef USE_CUDA_FILTER
class GpuCropper
{
public:
  ~GpuCropper()
  {
    if (h_buf_) cudaFreeHost(h_buf_);
    if (d_a_) cudaFree(d_a_);
    if (d_b_) cudaFree(d_b_);
    if (d_c_) cudaFree(d_c_);
    delete filter_;
    if (stream_) cudaStreamDestroy(stream_);
  }

  bool init()
  {
    if (cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking) != cudaSuccess)
      return false;
    filter_ = new cudaFilter(stream_);
    return true;
  }

  // 返回 false 表示 GPU 路径失败, 调用方应回退 CPU
  bool crop(const pcl::PointCloud<PointType> &in, pcl::PointCloud<PointType> &out,
            double xmin, double xmax, double ymin, double ymax,
            double zmin, double zmax, double voxel)
  {
    const unsigned int n = static_cast<unsigned int>(in.size());
    if (n == 0) return false;
    if (!ensureCapacity(n)) { fail("显存/锁页内存分配失败"); return false; }

    // 点云 -> 交错 float4 (x,y,z,intensity), 与 cudaFilter 约定一致
    for (unsigned int i = 0; i < n; ++i)
    {
      const auto &p = in.points[i];
      float *d = h_buf_ + i * 4;
      d[0] = p.x; d[1] = p.y; d[2] = p.z; d[3] = p.intensity;
    }

    const size_t bytes = static_cast<size_t>(n) * 4 * sizeof(float);
    cudaError_t e = cudaMemcpyAsync(d_a_, h_buf_, bytes, cudaMemcpyHostToDevice, stream_);
    if (e != cudaSuccess) { fail(std::string("H2D 拷贝失败: ") + cudaGetErrorString(e)); return false; }

    FilterParam_t param;
    // 必须零初始化: PASSTHROUGH 时 voxelX/Y/Z 用不到, 但库内部会读这些字段,
    // 留栈垃圾会导致 filter() 返回非零 (obs_fitting 里没清零, 属于侥幸)
    std::memset(&param, 0, sizeof(param));
    param.voxelX = param.voxelY = param.voxelZ = 0.01f;
    unsigned int left = n;

    // x -> y -> z 三次 PASSTHROUGH, 在 d_a_/d_b_/d_c_ 之间乒乓
    if (!pass(param, 0, xmin, xmax, d_b_, &left, d_a_, left)) { fail("x 维 PASSTHROUGH 失败"); return false; }
    if (left == 0) { out.clear(); return true; }
    if (!pass(param, 1, ymin, ymax, d_c_, &left, d_b_, left)) { fail("y 维 PASSTHROUGH 失败"); return false; }
    if (left == 0) { out.clear(); return true; }
    if (!pass(param, 2, zmin, zmax, d_b_, &left, d_c_, left)) { fail("z 维 PASSTHROUGH 失败"); return false; }
    if (left == 0) { out.clear(); return true; }

    float *src = d_b_;
    if (voxel > 0.0)
    {
      param.type = VOXELGRID;
      param.voxelX = param.voxelY = param.voxelZ = static_cast<float>(voxel);
      filter_->set(param);   // 同上: set() 返回值无意义, 只看 filter()
      if (filter_->filter(d_c_, &left, d_b_, left) != 0) { fail("VOXELGRID filter 失败"); return false; }
      src = d_c_;
      if (left == 0) { out.clear(); return true; }
    }

    e = cudaMemcpyAsync(h_buf_, src, static_cast<size_t>(left) * 4 * sizeof(float),
                        cudaMemcpyDeviceToHost, stream_);
    if (e != cudaSuccess) { fail(std::string("D2H 拷贝失败: ") + cudaGetErrorString(e)); return false; }
    e = cudaStreamSynchronize(stream_);
    if (e != cudaSuccess) { fail(std::string("流同步失败: ") + cudaGetErrorString(e)); return false; }

    out.clear();
    out.reserve(left);
    for (unsigned int i = 0; i < left; ++i)
    {
      const float *d = h_buf_ + i * 4;
      PointType p;
      p.x = d[0]; p.y = d[1]; p.z = d[2]; p.intensity = d[3];
      out.push_back(p);
    }
    return true;
  }

private:
  static void fail(const std::string &why)
  {
    ROS_WARN_THROTTLE(5.0, "[patchworkpp] GPU 裁剪失败(%s), 回退 CPU", why.c_str());
  }
  bool pass(FilterParam_t &param, int dim, double lo, double hi,
            float *dst, unsigned int *left, float *src, unsigned int n)
  {
    param.type = PASSTHROUGH;
    param.limitsNegative = false;
    param.dim = dim;
    param.downFilterLimits = static_cast<float>(lo);
    param.upFilterLimits = static_cast<float>(hi);
    // 注意: cudaFilter::set() 的返回值是垃圾(实测返回 -623431296 这类随机值),
    // 该库并不通过它报错 —— obs_fitting 的 gpuCropAndDownsample 也是直接丢弃
    // set() 的返回值, 只检查 filter()。这里必须照做, 否则会误判成失败。
    filter_->set(param);
    *left = n;
    int rc = filter_->filter(dst, left, src, n);
    if (rc != 0)
    {
      ROS_WARN_THROTTLE(5.0, "[patchworkpp] cudaFilter::filter(dim=%d, n=%u) 返回 %d",
                        dim, n, rc);
      return false;
    }
    return true;
  }

  bool ensureCapacity(unsigned int n)
  {
    if (n <= cap_) return true;
    unsigned int want = std::max(n, cap_ ? cap_ * 2 : 1u << 16);
    const size_t bytes = static_cast<size_t>(want) * 4 * sizeof(float);
    if (h_buf_) cudaFreeHost(h_buf_);
    if (d_a_) cudaFree(d_a_);
    if (d_b_) cudaFree(d_b_);
    if (d_c_) cudaFree(d_c_);
    h_buf_ = nullptr; d_a_ = d_b_ = d_c_ = nullptr;
    if (cudaHostAlloc(&h_buf_, bytes, cudaHostAllocDefault) != cudaSuccess) { cap_ = 0; return false; }
    if (cudaMalloc(&d_a_, bytes) != cudaSuccess) { cap_ = 0; return false; }
    if (cudaMalloc(&d_b_, bytes) != cudaSuccess) { cap_ = 0; return false; }
    if (cudaMalloc(&d_c_, bytes) != cudaSuccess) { cap_ = 0; return false; }
    cap_ = want;
    return true;
  }

  cudaStream_t stream_ = nullptr;
  cudaFilter *filter_ = nullptr;
  float *h_buf_ = nullptr, *d_a_ = nullptr, *d_b_ = nullptr, *d_c_ = nullptr;
  unsigned int cap_ = 0;
};
GpuCropper *g_gpu = nullptr;
#endif  // USE_CUDA_FILTER

template <typename T>
sensor_msgs::PointCloud2 toMsg(const pcl::PointCloud<T> &cloud, const std_msgs::Header &hdr)
{
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header = hdr;   // 时间戳与 frame_id 沿用输入, 下游 TF 时间对齐依赖这个
  return msg;
}

// CPU 精修: 车身框、min_radius、以及非 90 度倍数偏航时的精确旋转框
void cpuRefine(const pcl::PointCloud<PointType> &in, pcl::PointCloud<PointType> &out,
               size_t &n_self, size_t &n_near, size_t &n_out_roi, size_t &n_ref)
{
  const double r2 = g_min_radius * g_min_radius;
  const double rr2 = g_ref_clear_r * g_ref_clear_r;
  out.clear();
  out.reserve(in.size());
  for (const auto &p : in.points)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

    if (g_min_radius > 0.0 && (double)p.x * p.x + (double)p.y * p.y < r2) { ++n_near; continue; }

    const double xv = g_cos_yaw * p.x + g_sin_yaw * p.y;
    const double yv = -g_sin_yaw * p.x + g_cos_yaw * p.y;

    // 以车辆参考点(后轴中心)为圆心的净空: 参考点在车体系 (ref_offset_x, 0)
    if (g_ref_clear_r > 0.0)
    {
      const double ex = xv - g_ref_offset_x;
      if (ex * ex + yv * yv < rr2) { ++n_ref; continue; }
    }

    if (!g_yaw_is_axis_aligned && g_roi.enable && !g_roi.contains(xv, yv, p.z)) { ++n_out_roi; continue; }
    if (g_self.enable && g_self.contains(xv, yv, p.z)) { ++n_self; continue; }
    out.push_back(p);
  }
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::PointCloud<PointType> cloud_in;
  pcl::fromROSMsg(*msg, cloud_in);
  if (cloud_in.empty())
  {
    ROS_WARN_THROTTLE(5.0, "[patchworkpp] 收到空点云, 跳过");
    return;
  }

  const ros::Time t_crop0 = ros::Time::now();
  pcl::PointCloud<PointType> cropped, cloud_roi;
  size_t n_self = 0, n_near = 0, n_out_roi = 0, n_ref = 0;
  bool gpu_ok = false;

#ifdef USE_CUDA_FILTER
  if (g_use_gpu && g_roi.enable)
  {
    gpu_ok = g_gpu->crop(cloud_in, cropped,
                         g_aabb_x_min, g_aabb_x_max, g_aabb_y_min, g_aabb_y_max,
                         g_roi.z_min, g_roi.z_max, g_voxel_leaf);
    if (!gpu_ok)
      ROS_WARN_THROTTLE(5.0, "[patchworkpp] GPU 裁剪失败, 本帧回退 CPU");
  }
#endif

  if (!gpu_ok)
  {
    // CPU 路径: 先做 ROI 轴对齐/旋转框裁剪
    cropped.clear();
    cropped.reserve(cloud_in.size());
    for (const auto &p : cloud_in.points)
    {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      if (g_roi.enable)
      {
        const double xv = g_cos_yaw * p.x + g_sin_yaw * p.y;
        const double yv = -g_sin_yaw * p.x + g_cos_yaw * p.y;
        if (!g_roi.contains(xv, yv, p.z)) { ++n_out_roi; continue; }
      }
      cropped.push_back(p);
    }
    if (g_voxel_leaf > 0.0 && !cropped.empty())
    {
      pcl::PointCloud<PointType>::Ptr tmp(new pcl::PointCloud<PointType>(cropped));
      pcl::VoxelGrid<PointType> vg;
      vg.setInputCloud(tmp);
      vg.setLeafSize(g_voxel_leaf, g_voxel_leaf, g_voxel_leaf);
      pcl::PointCloud<PointType> o;
      vg.filter(o);
      cropped.swap(o);
    }
  }

  // CPU 精修 (点数已很少)
  cpuRefine(cropped, cloud_roi, n_self, n_near, n_out_roi, n_ref);
  const double crop_ms = (ros::Time::now() - t_crop0).toSec() * 1000.0;

  if (cloud_roi.empty())
  {
    ROS_WARN_THROTTLE(2.0,
                      "[patchworkpp] ROI 过滤后点云为空 (输入 %zu 点), ROI 是不是设得太小?",
                      cloud_in.size());
    return;
  }

  if (g_pub_roi.getNumSubscribers() > 0)
    g_pub_roi.publish(toMsg(cloud_roi, msg->header));

  pcl::PointCloud<PointType> ground, nonground;
  double t = 0.0;
  g_seg->estimate_ground(cloud_roi, ground, nonground, t);

  g_pub_ground.publish(toMsg(ground, msg->header));
  g_pub_nonground.publish(toMsg(nonground, msg->header));

  ROS_INFO_THROTTLE(5.0,
                    "[patchworkpp] in=%zu -> roi=%zu (丢: 近距%zu 参考点净空%zu 框外%zu 车身%zu) "
                    "-> ground=%zu nonground=%zu | 裁剪 %.1fms(%s) 分割 %.1fms",
                    cloud_in.size(), cloud_roi.size(), n_near, n_ref, n_out_roi, n_self,
                    ground.size(), nonground.size(), crop_ms,
                    gpu_ok ? "GPU" : "CPU", t * 1000.0);
}
}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "patchworkpp");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string cloud_topic;
  pnh.param<std::string>("cloud_topic", cloud_topic, "/velodyne_points");

  double yaw_deg = 0.0;
  pnh.param("lidar_yaw_offset_deg", yaw_deg, -90.0);
  g_cos_yaw = std::cos(yaw_deg * M_PI / 180.0);
  g_sin_yaw = std::sin(yaw_deg * M_PI / 180.0);
  g_yaw_is_axis_aligned = std::fabs(std::fmod(std::fabs(yaw_deg), 90.0)) < 1e-6 ||
                          std::fabs(std::fabs(std::fmod(std::fabs(yaw_deg), 90.0)) - 90.0) < 1e-6;

  // ROI 默认值: 按 ego-planner 实际需求给 (grid_map/local_update_range=4m,
  // teb/obstacle_radius=3m)。ROI 体积直接决定分割耗时, 不要给大。
  g_roi.front = 6.0;  g_roi.back = 4.0;
  g_roi.left = 4.0;   g_roi.right = 4.0;
  g_roi.z_min = -1.2; g_roi.z_max = 1.0;
  g_roi.load(pnh, "roi", true);

  // 车身框默认值严格对应 obs_fitting/config/obs.yaml 的 car_area:
  //   雷达系 x∈[-0.5,0.5] y∈[-0.67,1.1] z∈[-1.01286,0.2]
  // 按 yaw=-90 换算成车体语义即下面这组
  g_self.front = 0.67;     g_self.back = 1.1;
  g_self.left = 0.5;       g_self.right = 0.5;
  g_self.z_min = -1.01286; g_self.z_max = 0.2;
  g_self.load(pnh, "self_filter", true);

  pnh.param("min_radius", g_min_radius, 1.5);
  // 以车辆参考点(后轴中心)为圆心的净空半径。
  // 为什么需要: min_radius 的空洞是以**激光雷达**为圆心的, 而 odom_rear_axle 把定位
  // 参考点搬到了后轴(雷达后方 |ref_offset_x|)。于是从参考点量, 空洞在车尾方向只剩
  // min_radius-|ref_offset_x|, 障碍会出现在比 teb 要求的净空
  // (robot_radius + teb/min_obstacle_dist) 更近的地方 -> TEB 每周期都在试图逃离一个
  // 逃不掉的障碍 -> 转向指令持续抖动。这里补一个以参考点为圆心的剔除圈。
  pnh.param("ref_clearance_radius", g_ref_clear_r, 0.0);
  pnh.param("ref_offset_x", g_ref_offset_x, 0.0);
  pnh.param("voxel_leaf", g_voxel_leaf, 0.05);  // 见 patchworkpp_params.yaml: 0 会让分割耗时从 5ms 涨到 109ms

  bool want_gpu = true;
  pnh.param("use_gpu", want_gpu, true);

  // 把车体系 ROI 旋到雷达系, 取轴对齐包围盒 (90 度倍数时 AABB 即精确框)
  {
    const double cx[4] = {g_roi.front, g_roi.front, -g_roi.back, -g_roi.back};
    const double cy[4] = {g_roi.left, -g_roi.right, g_roi.left, -g_roi.right};
    g_aabb_x_min = g_aabb_y_min = 1e18;
    g_aabb_x_max = g_aabb_y_max = -1e18;
    for (int i = 0; i < 4; ++i)
    {
      // 车体系 -> 雷达系: p_L = Rz(yaw) * p_B
      const double x = g_cos_yaw * cx[i] - g_sin_yaw * cy[i];
      const double y = g_sin_yaw * cx[i] + g_cos_yaw * cy[i];
      g_aabb_x_min = std::min(g_aabb_x_min, x); g_aabb_x_max = std::max(g_aabb_x_max, x);
      g_aabb_y_min = std::min(g_aabb_y_min, y); g_aabb_y_max = std::max(g_aabb_y_max, y);
    }
  }

#ifdef USE_CUDA_FILTER
  if (want_gpu)
  {
    int dev_count = 0;
    if (cudaGetDeviceCount(&dev_count) == cudaSuccess && dev_count > 0)
    {
      g_gpu = new GpuCropper();
      if (g_gpu->init()) g_use_gpu = true;
      else { delete g_gpu; g_gpu = nullptr; ROS_WARN("[patchworkpp] CUDA 初始化失败, 用 CPU 裁剪"); }
    }
    else
      ROS_WARN("[patchworkpp] 未检测到 CUDA 设备, 用 CPU 裁剪");
  }
#else
  if (want_gpu)
    ROS_WARN("[patchworkpp] 本次编译未启用 CUDA (缺 libcudafilter.so 或 CUDA 工具链), 用 CPU 裁剪");
#endif

  g_seg.reset(new PatchWorkpp<PointType>(&pnh));

  g_pub_ground = pnh.advertise<sensor_msgs::PointCloud2>("ground", 10);
  g_pub_nonground = pnh.advertise<sensor_msgs::PointCloud2>("nonground", 10);
  g_pub_roi = pnh.advertise<sensor_msgs::PointCloud2>("roi", 10);

  ros::Subscriber sub = nh.subscribe(cloud_topic, 1, cloudCallback);

  ROS_INFO("[patchworkpp] 订阅 %s, 发布 %s/{ground,nonground,roi}",
           cloud_topic.c_str(), ros::this_node::getName().c_str());
  ROS_INFO("[patchworkpp] 裁剪后端: %s", g_use_gpu ? "GPU (libcudafilter.so)" : "CPU");
  ROS_INFO("[patchworkpp] yaw_offset=%.1f deg%s", yaw_deg,
           g_yaw_is_axis_aligned ? " (90度倍数, GPU 轴对齐框即精确框)"
                                 : " (非90度倍数, GPU 出包围盒后由 CPU 精确细化)");
  if (g_roi.enable)
    ROS_INFO("[patchworkpp] ROI(车体系): 前%.1f 后%.1f 左%.1f 右%.1f z[%.2f, %.2f]"
             " -> 雷达系 x[%.2f, %.2f] y[%.2f, %.2f]",
             g_roi.front, g_roi.back, g_roi.left, g_roi.right, g_roi.z_min, g_roi.z_max,
             g_aabb_x_min, g_aabb_x_max, g_aabb_y_min, g_aabb_y_max);
  else
    ROS_WARN("[patchworkpp] ROI 已关闭, 全量点云送入分割");
  if (g_self.enable)
    ROS_INFO("[patchworkpp] 车身框: 前%.1f 后%.1f 左%.1f 右%.1f z[%.2f, %.2f]",
             g_self.front, g_self.back, g_self.left, g_self.right, g_self.z_min, g_self.z_max);
  ROS_INFO("[patchworkpp] min_radius=%.2f m (小于此半径直接丢弃, 因 patchwork 会把 "
           "min_r 内的点无条件判为障碍), voxel_leaf=%.2f", g_min_radius, g_voxel_leaf);

  ros::spin();

#ifdef USE_CUDA_FILTER
  delete g_gpu;
#endif
  return 0;
}
