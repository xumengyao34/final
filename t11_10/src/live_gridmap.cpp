// 建图时实时把 FAST-LIO 的累积地图点云 (/Laser_map, camera_init 系) 转成 2D 占据栅格图,
// 过滤逻辑: 体素降采样 0.2m + 局部地面自适应剔除
// 不退出想中途存一份:
//   ros2 service call /save_map std_srvs/srv/Trigger
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

namespace fs = std::filesystem;

class LiveGridmap : public rclcpp::Node {
public:
    LiveGridmap() : Node("live_gridmap") {
        // 过滤参数默认值
        cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/Laser_map");
        grid_resolution_ = this->declare_parameter<double>("grid_resolution", 0.05);
        voxel_leaf_ = this->declare_parameter<double>("voxel_leaf", 0.2);
        ground_res_ = this->declare_parameter<double>("ground_res", 0.5);
        h_min_ = this->declare_parameter<double>("h_min", 0.2);
        h_max_ = this->declare_parameter<double>("h_max", 3.0);
        min_col_height_ = this->declare_parameter<double>("min_col_height", 0.6);
        publish_rate_ = this->declare_parameter<double>("publish_rate", 1.0);
        save_dir_ = this->declare_parameter<std::string>(
            "save_dir", "/home/xmy/final/src/mbf_test/maps");
        save_name_ = this->declare_parameter<std::string>("save_name", "real_map");

        // /map 用 transient_local, 晚启动的 RViz 也能收到
        rclcpp::QoS map_qos(10);
        map_qos.transient_local();
        map_qos.reliable();
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", map_qos);

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, rclcpp::QoS(10).reliable(),
            std::bind(&LiveGridmap::cloud_cb, this, std::placeholders::_1));

        // /save_map: 不退出也随时手动保存一份
        save_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "save_map",
            std::bind(&LiveGridmap::save_cb, this, std::placeholders::_1, std::placeholders::_2));

        publish_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate_),
            std::bind(&LiveGridmap::publish_tick, this));

        RCLCPP_INFO(this->get_logger(),
                    "live_gridmap 启动: 订阅 %s -> /map, ctrl-C 自动保存到 %s/%s.yaml",
                    cloud_topic_.c_str(), save_dir_.c_str(), save_name_.c_str());
    }

    // main 里 spin 返回后 (收到 SIGINT) 调用: 用最新点云重建并保存
    bool save_now() { return save(); }

private:
    void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *cloud);
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_cloud_ = cloud;
    }

    void publish_tick() {
        nav_msgs::msg::OccupancyGrid::SharedPtr grid;
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            if (!latest_cloud_ || latest_cloud_->empty()) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "还没收到点云 (%s), 等 FAST-LIO 出 /Laser_map ...",
                                     cloud_topic_.c_str());
                return;
            }
            // 累积地图只增不删, 点数没变说明没新内容, 跳过重建 (transient_local 已锁存旧图)
            if (latest_cloud_->size() == last_size_ && last_grid_) return;
            grid = build_grid(*latest_cloud_);
            last_size_ = latest_cloud_->size();
        }
        if (!grid) return;
        last_grid_ = grid;
        map_pub_->publish(*grid);
        cloud_pub_->publish(*grid_to_cloud(*grid));
        RCLCPP_INFO(this->get_logger(), "发布 /map: %dx%d 栅格, %ld 个障碍格, 点云 %zu 点",
                    grid->info.width, grid->info.height,
                    std::count(grid->data.begin(), grid->data.end(), 100), last_size_);
    }

    // 占用格 -> 点云 (RViz Map 显示坏掉时的替代渲染路径, 见文件头注释)
    sensor_msgs::msg::PointCloud2::SharedPtr grid_to_cloud(
        const nav_msgs::msg::OccupancyGrid& grid) {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud.reserve(std::count(grid.data.begin(), grid.data.end(), 100));
        const float r = grid.info.resolution;
        for (uint32_t row = 0; row < grid.info.height; row++) {
            for (uint32_t col = 0; col < grid.info.width; col++) {
                if (grid.data[static_cast<size_t>(row) * grid.info.width + col] != 100) continue;
                pcl::PointXYZRGB p;
                p.x = grid.info.origin.position.x + (col + 0.5f) * r;
                p.y = grid.info.origin.position.y + (row + 0.5f) * r;
                p.z = 0.05f;
                p.r = 200; p.g = 200; p.b = 200;
                cloud.push_back(p);
            }
        }
        auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(cloud, *msg);
        msg->header = grid.header;
        return msg;
    }

    // 流程: 体素降采样 + 按 XY 列统计局部地面
    nav_msgs::msg::OccupancyGrid::SharedPtr build_grid(
        const pcl::PointCloud<pcl::PointXYZ>& cloud) {
        // 1. 体素降采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud.makeShared());
        vg.setLeafSize(static_cast<float>(voxel_leaf_), static_cast<float>(voxel_leaf_),
                       static_cast<float>(voxel_leaf_));
        vg.filter(*down);

        // 2. 按 XY 栅格统计每列的最低点 (局部地面) 和最高点
        std::unordered_map<int64_t, std::pair<float, float>> cols;  // key -> (min_z, max_z)
        for (const auto& pt : down->points) {
            int64_t gx = static_cast<int64_t>(std::floor(pt.x / ground_res_));
            int64_t gy = static_cast<int64_t>(std::floor(pt.y / ground_res_));
            int64_t key = (gx << 32) | (gy & 0xffffffffLL);
            auto it = cols.find(key);
            if (it == cols.end()) {
                cols[key] = {pt.z, pt.z};
            } else {
                it->second.first = std::min(it->second.first, pt.z);
                it->second.second = std::max(it->second.second, pt.z);
            }
        }

        // 3. 障碍物提取: 列高差 > min_col_height, 且点在局部地面上 [h_min, h_max] 内
        std::vector<const pcl::PointXYZ*> kept;
        kept.reserve(down->size());
        for (const auto& pt : down->points) {
            int64_t gx = static_cast<int64_t>(std::floor(pt.x / ground_res_));
            int64_t gy = static_cast<int64_t>(std::floor(pt.y / ground_res_));
            int64_t key = (gx << 32) | (gy & 0xffffffffLL);
            auto it = cols.find(key);
            if (it == cols.end()) continue;
            float ground_z = it->second.first;
            float col_height = it->second.second - ground_z;
            float dz = pt.z - ground_z;
            if (col_height > min_col_height_ && dz >= h_min_ && dz <= h_max_) {
                kept.push_back(&pt);
            }
        }
        if (kept.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "没有提取到障碍物, 检查 h_min/h_max/min_col_height");
            return nullptr;
        }

        // 4. 2D 边界 + 栅格
        float min_x = 1e6f, max_x = -1e6f, min_y = 1e6f, max_y = -1e6f;
        for (const auto* p : kept) {
            min_x = std::min(min_x, p->x);
            max_x = std::max(max_x, p->x);
            min_y = std::min(min_y, p->y);
            max_y = std::max(max_y, p->y);
        }
        int width = static_cast<int>((max_x - min_x) / grid_resolution_) + 1;
        int height = static_cast<int>((max_y - min_y) / grid_resolution_) + 1;

        auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        grid->header.frame_id = "map";  // RViz Fixed Frame 设 map
        grid->header.stamp = this->now();
        grid->info.resolution = static_cast<float>(grid_resolution_);
        grid->info.width = width;
        grid->info.height = height;
        grid->info.origin.position.x = min_x;
        grid->info.origin.position.y = min_y;
        grid->info.origin.orientation.w = 1.0;
        grid->data.assign(static_cast<size_t>(width) * height, 0);  // 0 = 空闲

        for (const auto* p : kept) {
            int col = static_cast<int>((p->x - min_x) / grid_resolution_);
            int row = static_cast<int>((p->y - min_y) / grid_resolution_);
            if (col >= 0 && col < width && row >= 0 && row < height) {
                grid->data[static_cast<size_t>(row) * width + col] = 100;  // 100 = 占用
            }
        }
        return grid;
    }

    bool save() {
        // 用最新点云重建 (比退出前最后一次发布更新); 没点云则退回已发布的地图
        nav_msgs::msg::OccupancyGrid::SharedPtr grid;
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            if (latest_cloud_ && !latest_cloud_->empty()) grid = build_grid(*latest_cloud_);
        }
        if (!grid) grid = last_grid_;
        if (!grid) {
            std::cout << "[live_gridmap] 保存失败: 没有可用地图数据" << std::endl;
            return false;
        }

        fs::path dir(save_dir_);
        fs::path pgm_path = dir / (save_name_ + ".pgm");
        fs::path yaml_path = dir / (save_name_ + ".yaml");

        // P5 PGM, 值与 map_saver_cli 一致: 占用->0, 空闲->254, 未知->205
        // (map_server 加载时 occ=(255-v)/255, 配 negate:0 + 0.65/0.25 实测约定)
        // [2026-10-29] 行序必须倒写: grid 行 0 = 最小 y, 而 map_server 读 PGM 时
        // 第 0 行 = 最大 y (图像顶部 = 地图高处)。原代码正序写 -> 载入的地图
        // y 轴镜像, AMCL 收敛到镜像位姿 (x, -y, -θ): 实车表现为"车向左走 TF
        // 向右走"、代价地图左右反 (10-29 实车确诊, 200 像素采样 200/200 命中
        // 翻转解释)。这里倒序写入 (行 height-1-row), 载入后与真实世界一致。
        int w = grid->info.width, h = grid->info.height;
        std::vector<uint8_t> img(static_cast<size_t>(w) * h, 254);
        for (int row = 0; row < h; row++) {
            for (int col = 0; col < w; col++) {
                int8_t v = grid->data[static_cast<size_t>(row) * w + col];
                uint8_t out = (v == 100) ? 0 : ((v == -1) ? 205 : 254);
                img[static_cast<size_t>(h - 1 - row) * w + col] = out;
            }
        }
        std::ofstream pgm(pgm_path, std::ios::binary);
        pgm << "P5\n" << w << " " << h << "\n255\n";
        pgm.write(reinterpret_cast<const char*>(img.data()),
                  static_cast<std::streamsize>(img.size()));
        pgm.close();

        // yaml 与 nav2 map_server 地图格式一致
        std::ofstream yaml(yaml_path);
        yaml << "image: " << save_name_ << ".pgm\n"
             << "mode: trinary\n"
             << "resolution: " << grid->info.resolution << "\n"
             << "origin: [" << grid->info.origin.position.x << ", "
             << grid->info.origin.position.y << ", "
             << grid->info.origin.position.z << "]\n"
             << "negate: 0\n"
             << "occupied_thresh: 0.65\n"
             << "free_thresh: 0.25\n";
        yaml.close();

        std::cout << "[live_gridmap] 地图已保存" << yaml_path << " (" << w << "x" << h
                  << ")" << std::endl;
        return true;
    }

    void save_cb(const std_srvs::srv::Trigger::Request::SharedPtr,
                 std_srvs::srv::Trigger::Response::SharedPtr res) {
        bool ok = save();
        res->success = ok;
        res->message = ok ? "saved" : "no map data";
    }

    std::string cloud_topic_;
    double grid_resolution_, voxel_leaf_, ground_res_, h_min_, h_max_, min_col_height_;
    double publish_rate_;
    std::string save_dir_, save_name_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::mutex cloud_mutex_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;
    size_t last_size_ = 0;
    nav_msgs::msg::OccupancyGrid::SharedPtr last_grid_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LiveGridmap>();
    rclcpp::spin(node);
    // spin 返回 = 收到 SIGINT (ctrl-C): 保存地图再退出
    node->save_now();
    rclcpp::shutdown();
    return 0;
}
