#include "hins/xingsong_driver.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <time.h>
#include <sys/time.h>
#include "hins_laser_interfaces/srv/hins_srv.hpp"
using namespace hins;

HSGetAreaDataPackage Command;     // 全局变量，获取区域数据指令
bool Area1, Area2,Area3;                   // 三个区域的bool值
int NowChannel;
void hins_callback(const hins_laser_interfaces::srv::HinsSrv::Request::SharedPtr request, const hins_laser_interfaces::srv::HinsSrv::Response::SharedPtr response);
void ParamInit(rclcpp::Node::SharedPtr &node, ConnectionAddress &laser_conn_info, XingSongLaserParam &laser_param, ShadowsFilterParam &shadows_filter_param);
unsigned long long last_system_time_stamp = 0;
int main(int argc, char *argv[]) 
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("hins_le_node");
  //auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 1);


  // 声明一个服务回调组
  rclcpp::CallbackGroup::SharedPtr callback_group_service_;
  // 声明一个服务端
  rclcpp::Service<hins_laser_interfaces::srv::HinsSrv>::SharedPtr hins_server;
  hins_server = node->create_service<hins_laser_interfaces::srv::HinsSrv>("HinsLESrv", hins_callback);
  

  std::string frame_id;
  float max_angle, min_angle;
  node->declare_parameter("frame_id", "laser_link");
  node->get_parameter_or<std::string>("frame_id", frame_id, "laser_link");
  node->declare_parameter("max_angle", 0.0f);
  node->get_parameter_or<float>("min_angle", min_angle, 0.0f);
  node->declare_parameter("min_angle", 360.0f);
  node->get_parameter_or<float>("max_angle", max_angle, 360.0f);

  max_angle = max_angle*M_PI/180.0f;
  min_angle = min_angle*M_PI/180.0f;
  
  std::cout << "max_angle:" << max_angle << "  min_angle:" << min_angle << std::endl;

  // 2. 雷达驱动
  ConnectionAddress laser_conn_info;
  XingSongLaserParam laser_param;
  ShadowsFilterParam shadows_filter_param;
    // 区域数据
  
  Command.angle = 0;
  Command.channel = 0;                     // 通道指定模式使用  
  Command.channel_group = 2;      // 智能通道选择模式使用
  Command.speed = 0;
  Command.mode = 1;

  // 雷达参数
  laser_param.run_state = "run";
  // 初始化参数
  ParamInit(node, laser_conn_info,laser_param,shadows_filter_param);
    // 启动雷达
  XingSongDriverHdr driver_hdr = std::make_shared<XingSongDriver>(laser_conn_info, laser_param, shadows_filter_param);
  auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
  rclcpp::WallRate loop_rate(50);

  rclcpp::Time start_scan_time;
  rclcpp::Time end_scan_time;
  double scan_duration;

  while (rclcpp::ok()) {
    start_scan_time = node->now();
    ScanData data = driver_hdr->GetFullScan();                                              // 获取测量数据
    if(data.distance_data.size()<10)
    {
      continue;
    }
    end_scan_time = node->now();
    scan_duration = (end_scan_time - start_scan_time).seconds();
    // 系统时间
    unsigned long sec, nsec;  
    unsigned long long system_time_stamp;
    unsigned long long dsys_time; // 系统时间间隔(nsec)
    struct timeval timeofday;
    gettimeofday(&timeofday,NULL);
    sec  = timeofday.tv_sec;
    nsec = timeofday.tv_usec * 1000;
    
    system_time_stamp = (unsigned long long)sec*1000000000ll + (unsigned long long)nsec;// - data.time_increment*1000;
    dsys_time = system_time_stamp - last_system_time_stamp;
    
    // scan_msg->header.stamp.fromNSec((uint64_t) (system_time_stamp-(system_time_stamp - last_system_time_stamp)));
    scan_msg->header.stamp.sec = (system_time_stamp-dsys_time)*1.e-9;
    scan_msg->header.stamp.nanosec = (system_time_stamp-(system_time_stamp - last_system_time_stamp)) % 1000000000ll;
    // 雷达测量一周所需的时间/测量点数
    //scan_msg->time_increment = static_cast<float>(dsys_time)*1.e-9/ static_cast<float>(data.distance_data.size());         // 每根激光间的时间差
    scan_msg->time_increment = scan_duration/static_cast<float>(data.distance_data.size()); 
    scan_msg->header.frame_id = frame_id;
    scan_msg->scan_time = scan_duration;                                        // 两帧数据之间的时间差（秒）
    rclcpp::Duration last_scan_duration(0, data.time_increment*1000);                          // 本帧的测量持续时间（nanoseconds纳秒）
    scan_msg->header.stamp = start_scan_time;                             // 本帧开始的时间戳
    // Full 360° LaserScan: N evenly spaced rays, consistent with slam_toolbox (N samples).
    const size_t n = data.distance_data.size();
    if (n <= 1) {
      scan_msg->angle_increment = 0.0f;
      scan_msg->angle_min = 0.0f;
      scan_msg->angle_max = 0.0f;
    } else {
      scan_msg->angle_increment = 2.0f * static_cast<float>(M_PI) / static_cast<float>(n);
      scan_msg->angle_min = 0.0f;
      scan_msg->angle_max =
          scan_msg->angle_min + static_cast<float>(n - 1) * scan_msg->angle_increment;
    }

    scan_msg->range_min = 0;                                                                // 测量距离的最大值
    scan_msg->range_max = 60;                                                               // 测量距离的最小值

    scan_msg->ranges.resize(data.distance_data.size());
    scan_msg->intensities.resize(data.distance_data.size());
    
    float angle_index = 0.0f;
    for(size_t i=0; i < data.distance_data.size(); i++) {
      if(angle_index < min_angle || angle_index > max_angle)                                 // 角度过滤             
        scan_msg->ranges[i] = 100;
      else
        scan_msg->ranges[i] = data.distance_data[i]/1000.0f;
      scan_msg->intensities[i] = data.amplitude_data[i];
      angle_index += scan_msg->angle_increment;
    }
    laser_pub->publish(*scan_msg);
    driver_hdr->SetAreaCommand(Command);                                                      // 设置指定通道
    Area1 = driver_hdr->GetBlock1Value();
    Area2 = driver_hdr->GetBlock2Value();
    Area3 = driver_hdr->GetBlock3Value();
    NowChannel = driver_hdr->GetResponseChannel();
  
    if(!rclcpp::ok()) 
    {
      break;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
}


void ParamInit(rclcpp::Node::SharedPtr &node, ConnectionAddress &laser_conn_info, XingSongLaserParam &laser_param,
               ShadowsFilterParam &shadows_filter_param)
{
  std::string laser_ip, frame_id;
  int laser_port, shadows_filter_level,
      shadows_filter_neighbors, shadows_filter_window, shadows_traverse_step;
  float shadows_filter_max_angle, shadows_filter_min_angle;
  bool change_param, block_enable, use_udp;
  string measure_frequency_kHz, motor_speed, point_sampling, filter_level;

  // 雷达连接参数
  node->declare_parameter("laser_ip", "192.168.1.88");
  node->declare_parameter("laser_port", 8080);
  node->get_parameter_or<std::string>("laser_ip", laser_ip, "192.168.1.88");
  // node->get_parameter<std::string>("laser_ip", laser_ip); 
  node->get_parameter_or<int>("laser_port", laser_port, 8080);
  laser_conn_info.SetAddress(laser_ip);
  laser_conn_info.SetPort(laser_port);
  std::cout << "laser_ip:" << laser_ip << std::endl;
  // 雷达配置参数
  node->declare_parameter("measure_frequency_kHz", "200");
  node->declare_parameter("motor_speed", "30");
  node->declare_parameter("point_sampling", "1");
  node->declare_parameter("filter_level", "1");
  node->declare_parameter("change_param", false);
  node->declare_parameter("block_enable", false);
  node->declare_parameter("use_udp", use_udp);

  node->get_parameter_or<string>("measure_frequency_kHz", measure_frequency_kHz, "200");
  node->get_parameter_or<string>("motor_speed", motor_speed, "30");
  node->get_parameter_or<string>("point_sampling", point_sampling, "1");
  node->get_parameter_or<string>("filter_level", filter_level, "1");
  node->get_parameter_or<bool>("change_param", change_param, false);
  node->get_parameter_or<bool>("block_enable", block_enable, false);
  node->get_parameter_or<bool>("use_udp", use_udp, false);
  if(use_udp)
    laser_conn_info.UseUdp();   //// udp
  laser_param.measure_frequency_kHz = measure_frequency_kHz;
  laser_param.spin_frequency_Hz = motor_speed;
  laser_param.sampling_size_per_position = point_sampling;
  laser_param.noise_filter_level = filter_level;
  laser_param.change_flag = change_param;
  laser_param.block_enable = block_enable;

  // 防拖尾算法参数
  node->declare_parameter("shadows_filter_level", 1);
  node->declare_parameter("shadows_filter_max_angle", 175.0f);
  node->declare_parameter("shadows_filter_min_angle", 5.0f);
  node->declare_parameter("shadows_filter_neighbors", 1);
  node->declare_parameter("shadows_filter_window",2);
  node->declare_parameter("shadows_traverse_step", 1);
  node->get_parameter_or<int>("shadows_filter_level", shadows_filter_level, 1);
  node->get_parameter_or<float>("shadows_filter_max_angle", shadows_filter_max_angle, 175.0f);
  node->get_parameter_or<float>("shadows_filter_min_angle", shadows_filter_min_angle, 5.0f);
  node->get_parameter_or<int>("shadows_filter_neighbors", shadows_filter_neighbors, 1);
  node->get_parameter_or<int>("shadows_filter_window", shadows_filter_window, 2);
  node->get_parameter_or<int>("shadows_traverse_step", shadows_traverse_step, 1);
  shadows_filter_param.shadows_filter_level = shadows_filter_level;
  shadows_filter_param.max_angle = shadows_filter_max_angle;
  shadows_filter_param.min_angle = shadows_filter_min_angle;
  shadows_filter_param.neighbors = shadows_filter_neighbors;
  shadows_filter_param.window = shadows_filter_window;
  shadows_filter_param.traverse_step = shadows_traverse_step;

} 



void hins_callback(const hins_laser_interfaces::srv::HinsSrv::Request::SharedPtr request,
        const hins_laser_interfaces::srv::HinsSrv::Response::SharedPtr response)
{
  Command.channel = request->channel;
  if(NowChannel == request->channel)                 //  判断请求的通道与现在的通道是否相同，是则success为true
    response->success = true;
  else
    response->success = false;
  
  response->area1 = Area1;
  response->area2 = Area2;
  response->area3 = Area3;

  // std::cout  << "Area1: " << Area1 << " "
  //                   << "Area2: " << Area2 << " "
  //                   << "Area3: " << Area3 << " "
  //                   << "NowChannel: " << NowChannel << " "
  //                   << "request->channel:" << int(request->channel) << " "
  //                   << "Command.channel:" << int(Command.channel) << " "
  //                   << std::endl;
}
