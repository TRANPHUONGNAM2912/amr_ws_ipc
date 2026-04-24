#include "hins/laser_data_receiver.h"
#include "hins/protoc.h"
#include "utils.h"
#include <fstream>
#include <iostream>

// #define LASER_RECEIVER_DEBUG
namespace hins {

LaserDataReceiver::LaserDataReceiver(const ConnectionAddress& conn_info)
    : conn_info_(conn_info)
    , is_connected_(false)
    , inbuf_(4096)
    , instream_(&inbuf_)
    , use_udp_(conn_info.GetUseUdp())
    , tcp_socket_ptr_(0)
    , udp_socket_ptr_(0)
    , ring_buffer_(65536)
    , scan_data_()
    , last_data_time_(hins::Now())
    , scan_all_point_num_(0)
    , laser_steady_time(500)
    , last_time_(0)
    // , last_begin_angle_(0)
{
    if(use_udp_)
        RCLCPP_INFO_STREAM(rclcpp::get_logger("hins_le_node"),"Connecting to UDP data channel at hostname: " << conn_info.GetAddress()
                        << " udp_port:" << conn_info.GetPort());
    else
        RCLCPP_INFO_STREAM(rclcpp::get_logger("hins_le_node"),"Connecting to TCP data channel at hostname: " << conn_info.GetAddress()
                        << " tcp_port:" << conn_info.GetPort());

    if (Connect())
        RCLCPP_INFO_STREAM(rclcpp::get_logger("hins_le_node"), "Lidar connect success.");
    else
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("hins_le_node"), "Lidar connect failed.");
}

LaserDataReceiver::~LaserDataReceiver()
{
    Disconnect();

    if(udp_socket_ptr_ && use_udp_)
    {
        Disconnect();
        delete udp_socket_ptr_;
    }
    else
        if(tcp_socket_ptr_)
        delete tcp_socket_ptr_;
}

short int LaserDataReceiver::GetLaserSteadyTime()
{
    return laser_steady_time;
}

bool LaserDataReceiver::IsConnected()
{
    return is_connected_;
}

bool LaserDataReceiver::Disconnect()
{
    // is_connected_ = false;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("hins_le_node"),"Disconnect.");
    try {
        if( udp_socket_ptr_ != nullptr && use_udp_)
            udp_socket_ptr_->close();
        else if( tcp_socket_ptr_ != nullptr )
            tcp_socket_ptr_->close();

        io_service_.stop();
        if (boost::this_thread::get_id() != io_service_thread_.get_id())
            io_service_thread_.join();
        is_connected_ = false;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("hins_le_node"),"Disconnect succeed");
        return true;
        
    } catch (std::exception& e) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("hins_le_node"),"Exception:" << e.what());
        RCLCPP_WARN_STREAM(rclcpp::get_logger("hins_le_node"),"Disconnect fail");
        return false;
    }    
}

bool LaserDataReceiver::CheckConnection()
{
    if (!IsConnected())
        return false;
    uint64_t now_time = hins::Now();
    if ((now_time - last_data_time_) > 1000) // 1s 断线
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("hins_le_node"), "disconnect over time, timeout:" << (now_time - last_data_time_)
                         << " now:" << now_time
                         << " last_data_time_:" << last_data_time_);
        Disconnect();
        return false;
    }
    return true;
}

ScanData LaserDataReceiver::GetFullScan()
{
    std::unique_lock<std::mutex> lock(scan_mutex_);
    while (CheckConnection() && scan_data_.size() < 2)
        data_notifier_.wait_for(lock, std::chrono::seconds(1));

    ScanData data;
    if (scan_data_.size() >= 2 && IsConnected()) {
        data = ScanData(std::move(scan_data_.front()));
        scan_data_.pop_front();
    } else
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("hins_le_node"), "null data");

#ifdef LASER_RECEIVER_DEBUG
    RCLCPP_INFO_STREAM(rclcpp::get_logger("hins_le_node"),"scan_all_point_num_:" << scan_all_point_num_
              << "     data.distance_data.size():" << data.distance_data.size());
#endif

    if (static_cast<int>(data.distance_data.size()) != scan_all_point_num_)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("hins_le_node"), "data.distance_data.size() != scan_all_point_num_");
        data.distance_data.clear();
    }
    // data.distance_data.clear();
    return data;
}

void LaserDataReceiver::HandleSocketRead(const boost::system::error_code& error)
{
    if (!error) {
        // 1. 将所有数据推送到缓冲区
        instream_.clear();
        if(use_udp_)
        {
            char buf[4096];
            int bytes_read = udp_socket_ptr_->receive(boost::asio::buffer(buf));
            // std::cout << "bytes_read:" << bytes_read <<std::endl;
            WriteBufferBack(buf, bytes_read);       // 将 buf 的数据写到 ring_buffer_ 里
        }
        else
            while(!instream_.eof())
            {
                char buf[4096];
                instream_.read(buf, 4096);
                int bytes_read = instream_.gcount();
                WriteBufferBack(buf, bytes_read);       // 将 buf 的数据写到 ring_buffer_ 里
            }

        // 2. 继续读取数据，直到数据被读取完
        while (HandleNextPacket()) { }

    // 3. 继续异步读取数据
        if(use_udp_)
        {
            udp_socket_ptr_->async_receive(boost::asio::null_buffers(), boost::bind(&LaserDataReceiver::HandleSocketRead, this, boost::asio::placeholders::error));
    //      udp_socket_ptr_->async_wait(boost::asio::ip::udp::socket::wait_read, boost::bind(&LaserDataReceiver::HandleSocketRead, this, boost::asio::placeholders::error));
        }
        else
            boost::asio::async_read(*tcp_socket_ptr_, inbuf_, boost::bind(&LaserDataReceiver::HandleSocketRead, this, boost::asio::placeholders::error));
   
    } 
    else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("hins_le_node"), "HandleSocketRead ---- ERROR");
        if (error.value() != 995)
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("hins_le_node"), "ERROR: data connection error:"
                    << error.message()
                    << " "
                    << error.value());
        Disconnect();
    }
}

int LaserDataReceiver::SyncWrite()
{
    boost::system::error_code ec;
    int ret;
    if(use_udp_)
    {
        udp_socket_ptr_->send(boost::asio::buffer(kStartCapture));
        ret = 1;
    }
    else
        ret = tcp_socket_ptr_->write_some(boost::asio::buffer(kStartCapture), ec);
  
    if (ec) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("hins_le_node"), "write kStartCapture failed:" << boost::system::system_error(ec).what());
        ret = -1;
    }
// #ifdef LASER_RECEIVER_DEBUG
    RCLCPP_INFO_STREAM(rclcpp::get_logger("hins_le_node"),"SyncWrite kStartCapture done");
// #endif
    return ret;
}

int LaserDataReceiver::SyncWrite(const std::array<unsigned char, 12> command)
{
    boost::system::error_code ec;
    int ret;
    if(use_udp_)
    {
        udp_socket_ptr_->send(boost::asio::buffer(command));
        ret = 1;
    }
    else
        ret = tcp_socket_ptr_->write_some(boost::asio::buffer(command), ec);

    if (ec) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("hins_le_node"), "write 12command failed:" << boost::system::system_error(ec).what());
        ret = -1;
    }
#ifdef LASER_RECEIVER_DEBUG
    std::cout << "SyncWrite command done" << std::endl;
#endif
    return ret;
}

int LaserDataReceiver::SyncWrite(const std::array<unsigned char, 18> command)
{
    boost::system::error_code ec;
    int ret;
    if(use_udp_)
    {
        udp_socket_ptr_->send(boost::asio::buffer(command));
        ret = 1;
    }
    else
        ret = tcp_socket_ptr_->write_some(boost::asio::buffer(command), ec);

    if(ec)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("hins_le_node"), "write 18command failed:" << boost::system::system_error(ec).what());
        ret = -1;
    }
    #ifdef DEBUG
    RCLCPP_INFO_STREAM(rclcpp::get_logger("hins_le_node"),"SyncWrite command done");
    #endif
    
    return ret;
}


// CRC循环校验码-12位命令
// 修改command的最后两位
// 返回循环校验码的10进制数字
unsigned int CRC_Verify_len12(std::array<unsigned char, 12>* command)
{
    unsigned int i, j;
    unsigned int wCrc = 0xffff;
    unsigned int wPolynom = 0xA001;
    /*-------------------------------------*/
    for (i = 0; i < 12 - 2; i++) // 后两位为校验位
    {
        wCrc ^= command->at(i);
        for (j = 0; j < 8; j++) {
            if (wCrc & 0x0001) {
                wCrc = (wCrc >> 1) ^ wPolynom;
            } else {
                wCrc = wCrc >> 1;
            }
        }
    }
    unsigned int tmp1 = (wCrc & 0xff00) >> 8; // 高位
    unsigned int tmp2 = (wCrc & 0x00ff); // 低位
    command->at(10) = tmp2;
    command->at(11) = tmp1;
    return wCrc;
}

/**
 * @brief 生成设置雷达参数的控制命令
 * @param[in] param 雷达参数
 * @return 雷达控制命令
 */
std::array<unsigned char, 12> LaserDataReceiver::GenerateParamCommand(XingSongLaserParam param)
{
    string run_state = param.run_state;
    string measure_frequency_kHz = param.measure_frequency_kHz;
    string spin_frequency_Hz = param.spin_frequency_Hz;
    string sampling_size_per_position = param.sampling_size_per_position;
    string noise_filter_level = param.noise_filter_level;

    std::array<unsigned char, 12> set_param_command;
    set_param_command.fill(0x00); // 命令初始化全为0

    //----------设置数据帧头----------//
    set_param_command.at(0) = 0x53;
    set_param_command.at(1) = 0x43;
    set_param_command.at(2) = 0x74;
    set_param_command.at(3) = 0x72;
    set_param_command.at(4) = 0x6c;

    //----------设置休眠控制----------//
    if (run_state == "run")
        set_param_command.at(5) = 0x00;
    else if (run_state == "stop")
        set_param_command.at(5) = 0x01;
    else
        RCLCPP_WARN_STREAM(rclcpp::get_logger("hins_le_node"),"run_state error!");// 若输入非法,则为run状态

    //----------设置测量频率----------//
    if (measure_frequency_kHz == "50")
        set_param_command.at(6) = 0x00;
    else if (measure_frequency_kHz == "100")
        set_param_command.at(6) = 0x01;
    else if (measure_frequency_kHz == "150")
        set_param_command.at(6) = 0x02;
    else if (measure_frequency_kHz == "200")
        set_param_command.at(6) = 0x03;
    else // 若输入非法,则为200kHz
    {
        set_param_command.at(6) = 0x03;
        RCLCPP_WARN_STREAM(rclcpp::get_logger("hins_le_node"),"measure_frequency_KHz error!");
    }

    //----------设置扫描频率----------//
    if (spin_frequency_Hz == "10")
        set_param_command.at(7) = 0x00;
    else if (spin_frequency_Hz == "15")
        set_param_command.at(7) = 0x01;
    else if (spin_frequency_Hz == "20")
        set_param_command.at(7) = 0x02;
    else if (spin_frequency_Hz == "25")
        set_param_command.at(7) = 0x03;
    else if (spin_frequency_Hz == "30")
        set_param_command.at(7) = 0x04;
    else // 若输入非法,则为30kHz
    {
        set_param_command.at(7) = 0x04;
        RCLCPP_WARN_STREAM(rclcpp::get_logger("hins_le_node"),"spin_frequency_Hz error!");
        std::cout << "spin_frequency_Hz:" << spin_frequency_Hz << std::endl;
    }

    //----------设置采样次数----------//
    stringstream sample_size_ss;
    int sample_size_int;
    sample_size_ss << sampling_size_per_position; // string转换为int
    sample_size_ss >> sample_size_int;

    if ((sample_size_int < 0) || (sample_size_int > 50)) // 输入非法
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("hins_le_node"),"sampling_size_per_position error!");
        sample_size_int = 1;
        set_param_command.at(8) = (unsigned char)sample_size_int;
    } else
        set_param_command.at(8) = (unsigned char)sample_size_int;

    //----------设置过滤等级----------//
    stringstream filter_level_ss;
    int filter_level_int;
    filter_level_ss << noise_filter_level; // string转换为int
    filter_level_ss >> filter_level_int;

    if ((filter_level_int < 0) || (filter_level_int > 3)) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("hins_le_node"),"noise_filter_level error!");
        filter_level_int = 1;
        set_param_command.at(9) = (unsigned char)filter_level_int;
    } else
        set_param_command.at(9) = (unsigned char)filter_level_int;

    //----------设置校验位----------//
    CRC_Verify_len12(&set_param_command);

#ifdef LASER_RECEIVER_DEBUG
    std::cout << "set_param_command:";
    for (auto val = set_param_command.begin(); val < set_param_command.end(); val++) {
        cout << (int)*val << " ";
    }
    std::cout << std::endl;
#endif

    return set_param_command;
}

bool LaserDataReceiver::Connect()
{
    try {
        // boost::system::error_code error = boost::asio::error::host_not_found;
        if(use_udp_){
            if(nullptr == udp_socket_ptr_){
                udp_socket_ptr_ = new boost::asio::ip::udp::socket(io_service_);
            } else {
                delete udp_socket_ptr_;
                udp_socket_ptr_ = new boost::asio::ip::udp::socket(io_service_);
            }
            boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::address::from_string(conn_info_.GetAddress()), conn_info_.GetPort());
            RCLCPP_INFO_STREAM(rclcpp::get_logger("hins_le_node"),"begin udp async_connect.");
            udp_socket_ptr_->async_connect(endpoint, boost::bind(&LaserDataReceiver::connect_handler, this, boost::asio::placeholders::error));
            RCLCPP_INFO_STREAM(rclcpp::get_logger("hins_le_node"),"udp async_connect end.");
        } else {     
            if (nullptr == tcp_socket_ptr_) {
                tcp_socket_ptr_ = new boost::asio::ip::tcp::socket(io_service_);
            } else {
                delete tcp_socket_ptr_;
                tcp_socket_ptr_ = new boost::asio::ip::tcp::socket(io_service_);
            }
            boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(conn_info_.GetAddress()), conn_info_.GetPort());

            RCLCPP_WARN_STREAM(rclcpp::get_logger("hins_le_node"),"begin tcp async_connect.");
            tcp_socket_ptr_->async_connect(endpoint, boost::bind(&LaserDataReceiver::connect_handler, this, boost::asio::placeholders::error));
            RCLCPP_WARN_STREAM(rclcpp::get_logger("hins_le_node"),"tcp async_connect end.");
        }
        // 开始异步读取数据
        if(use_udp_){
            udp_socket_ptr_->async_receive(boost::asio::null_buffers(), boost::bind(&LaserDataReceiver::HandleSocketRead, this, boost::asio::placeholders::error));
        } else
            boost::asio::async_read(*tcp_socket_ptr_, inbuf_, boost::bind(&LaserDataReceiver::HandleSocketRead, this, boost::asio::placeholders::error));
        io_service_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
    } catch (std::exception& e) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("hins_le_node"), "Exception:" << e.what());
        is_connected_ = false;
        return false;
        }
    is_connected_ = true;
    last_data_time_ = hins::Now();
    return true;
}


// 根据协议寻找数据帧头
int16_t LaserDataReceiver::FindPacketStart()
{
  if(ring_buffer_.size() < kXSPackageHeadSize)
    return -1;

  // 兴颂雷达数据的帧头
  for(size_t i = 0; i < ring_buffer_.size() - 4; ++i)
  {
    if(0x48 == ((unsigned char)ring_buffer_[i])   &&
       0x49 == ((unsigned char)ring_buffer_[i+1]) &&
       0x53 == ((unsigned char)ring_buffer_[i+2]) &&
       0x4e == ((unsigned char)ring_buffer_[i+3]))
     {
      area_data_flag_ = false;
      return i;
    }
      else if(
      0x57 == ((unsigned char)ring_buffer_[i])   &&
       0x53 == ((unsigned char)ring_buffer_[i+1]) &&
       0x69 == ((unsigned char)ring_buffer_[i+2]) &&
       0x6d == ((unsigned char)ring_buffer_[i+3]) &&
       0x75 == ((unsigned char)ring_buffer_[i+4])
    )
    {
    //   std::cout << "区域数据帧头" << std::endl;
      area_data_flag_ = true;
      return i;
    }
  }
  return -2;
}


// 将src数据写到 ring_buffer_ 里
void LaserDataReceiver::WriteBufferBack(char* src, std::size_t num_bytes)
{
    if (ring_buffer_.size() + num_bytes > ring_buffer_.capacity())
        throw std::exception();

    ring_buffer_.resize(ring_buffer_.size() + num_bytes); // 修改 ring_buffer_ 的大小
    char* pone = ring_buffer_.array_one().first; // ring_buffer_ 的 array_one 的头指针
    std::size_t pone_size = ring_buffer_.array_one().second; // ring_buffer_ 的 array_one 的大小
    char* ptwo = ring_buffer_.array_two().first; // ring_buffer_ 的 array_two 的头指针
    std::size_t ptwo_size = ring_buffer_.array_two().second; // ring_buffer_ 的 array_two 的大小

    // 将src数据写到 ring_buffer_ 里
    if (ptwo_size >= num_bytes) {
        std::memcpy(ptwo + ptwo_size - num_bytes, src, num_bytes);
    } else {
        std::memcpy(pone + pone_size + ptwo_size - num_bytes, src, num_bytes - ptwo_size);
        std::memcpy(ptwo, src + num_bytes - ptwo_size, ptwo_size);
    }
}

bool LaserDataReceiver::HandleNextPacket()
{
    if (scan_data_.empty()) {
        scan_data_.emplace_back();
    }

    if (RetrivePacket()) {
        return true;
    } else {
        return false;
    }
}

/**
 * @brief 计算雷达上帧和本帧的时间差
 * @param[in] now 本次时间戳
 * @param[in] last 上次时间戳
 * @return 时间差
 */
uint16_t TimeIncrement(uint16_t now, uint16_t last)
{
    // std::cout << "now:" << now << "  "
    //                     << "last:" << last << "  "
    //                     << std::endl;
    if (now < last) // 将一圈的时间放进scan_data.time_increment里
        {
            // std::cout << "TimeIncrement:" << 65535 - last + now << std::endl;
            return 65535 -last + now;
        }
    else
        // std::cout << "TimeIncrement:" <<  now - last << std::endl;
        return now - last;

}


bool LaserDataReceiver::RetrivePacket()
{
  bool ret = false;
  int16_t head_index = FindPacketStart();                     // 寻找帧头
  if(head_index < 0)
    return ret;

  if(area_data_flag_)
  {
    return GetAreaData(head_index);
  }
  else
    return GetRangeData(head_index);
}

/**
 * @brief 获取雷达距离数据
 */
bool LaserDataReceiver::GetRangeData(int16_t head_index)
{
   bool ret = false;

    if (head_index < 0)
        return ret;

    // 寻找帧头并处理数据
    if (ring_buffer_.size() - head_index >= kXSPackageHeadSize) {
        ring_buffer_.erase_begin(head_index); // 删除 帧头前的数据
        head_index = 0;

        // 1. 解析数据帧帧头
        char head_buf[kXSPackageHeadSize];
        ReadBufferFront(head_buf, kXSPackageHeadSize);

        XSPackageHeader header;
        header.start_angle = (unsigned char)head_buf[4] << 8; // 起始角度
        header.start_angle |= (unsigned char)head_buf[5];
        header.end_angle = (unsigned char)head_buf[6] << 8; // 终止角度
        header.end_angle |= (unsigned char)head_buf[7];
        header.data_size = (unsigned char)head_buf[8] << 8; // 测量点总数
        header.data_size |= (unsigned char)head_buf[9];
        header.data_position = (unsigned char)head_buf[10] << 8; // 当前帧测量点位置
        header.data_position |= (unsigned char)head_buf[11];
        header.measure_size = (unsigned char)head_buf[12] << 8; // 当前帧测量点数量
        header.measure_size |= (unsigned char)head_buf[13];
        header.time = (unsigned char)head_buf[14] << 8; // 当前帧测量时间戳
        header.time |= (unsigned char)head_buf[15];
        // if (header.data_size > header.measure_size) {
        //     header.data_size = header.measure_size;
        // }

        if (header.start_angle == 0 && laser_steady_time > 0) // 前几帧数据异常
        {
            scan_all_point_num_ = header.measure_size * 15;
            // rec_begin_flag_ += 1;
            laser_steady_time--;
        }

        std::unique_lock<std::mutex> lock(scan_mutex_);

        short int begin_point_index = 0;
        // 2. 解析帧内容
        ScanData& scan_data = scan_data_.back();
    
        // scan_data.time_increment += TimeIncrement(header.time, last_time_);
        // std::cout << "scan_data.time_increment:" << scan_data.time_increment <<std::endl; 
        last_time_ = header.time;
        float angle_increment = int(header.end_angle)-int(header.start_angle);
        if(angle_increment<0)
            angle_increment = 360+angle_increment;
        angle_increment/=header.measure_size;
        // begin_point_index = header.start_angle * header.measure_size/24 + header.data_position - header.data_size;
        begin_point_index = header.start_angle/angle_increment+header.data_position-header.data_size;
        int all_point_num = 360.0f/angle_increment;
        scan_data.distance_data.resize(all_point_num, kMaxDistance);
        scan_data.amplitude_data.resize(all_point_num, 0);

        uint16_t body_size = kXSPackageHeadSize + header.data_size * 4;
        if ((ring_buffer_.size() - head_index) >= body_size) {
            #ifdef LASER_RECEIVER_DEBUG
            std::cout << "begin:" << header.start_angle
                      << "  end:" << header.end_angle
                      << "  data_size:" << header.data_size
                      << "  data_position:" << header.data_position
                      << "  measure_size:" << header.measure_size
                      << "  time:" << header.time
                      << "  laser_steady_time:" << laser_steady_time
                      << "  begin_point_index:" << begin_point_index
                      << "  angle_increment:" << angle_increment
                      << "  all_point_num:" << all_point_num
                      << std::endl;
            #endif
            if(header.start_angle != 336 && header.end_angle == 0)
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("hins_le_node"), "begin:" << header.start_angle
                                    << "  end:" << header.end_angle
                                    << "  data_size:" << header.data_size
                                    << "  data_position:" << header.data_position
                                    << "  measure_size:" << header.measure_size
                                    << "  time:" << header.time
                                    << "  laser_steady_time:" << laser_steady_time
                                    << "  begin_point_index:" << begin_point_index
                                    << "  angle_increment:" << angle_increment
                                    << "  all_point_num:" << all_point_num
                                    );

            char* body_buf = new char[body_size];
            ReadBufferFront(body_buf, body_size); // 将当前帧的数据复制到 body_buf 中

            ring_buffer_.erase_begin(head_index + body_size); // 删除 ring_buffer_ 的数据

            for (int i = 0; i < header.data_size; i++) {

                unsigned short int distance;
                unsigned short int intensity;

                // 获取强度和距离数据
                distance = (unsigned char)body_buf[i * 4 + 17] * 256;
                distance |= (unsigned char)body_buf[i * 4 + 16];
                intensity = (unsigned char)body_buf[i * 4 + 19] * 256;
                intensity |= (unsigned char)body_buf[i * 4 + 18];

                if (distance > kMaxDistance) {
                    distance = kMaxDistance;
                }


                // 强度值大于IntensityThreshold设为0
                if (intensity > IntensityThreshold) {
                    // std::cout << header.data_position+i << ": " << intensity << std::endl;
                    intensity = 0;
                }
                // try {
                    scan_data.distance_data.at(begin_point_index+i) = distance;
                    scan_data.amplitude_data.at(begin_point_index+i) = intensity;
                // } catch (std::exception& e) {
                //     RCLCPP_ERROR_STREAM(rclcpp::get_logger("hins_le_node"), "Exception:" << e.what());
                //     RCLCPP_INFO_STREAM(rclcpp::get_logger("hins_le_node"),"begin:" << header.start_angle
                //                     << "  end:" << header.end_angle
                //                     << "  data_size:" << header.data_size
                //                     << "  data_position:" << header.data_position
                //                     << "  measure_size:" << header.measure_size
                //                     << "  time:" << header.time
                //                     << "  laser_steady_time:" << laser_steady_time
                //                     << "  begin_point_index:" << begin_point_index
                //                     << "  angle_increment:" << angle_increment
                //                     << "  all_point_num:" << all_point_num
                //                     << "  i:" << i);
                // }
            }

            delete[] body_buf;
            uint64_t now_time = hins::Now();
            if ((now_time - last_data_time_) > 1000) // 与上帧时间超过1s
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("hins_le_node"), "begin:" << header.start_angle
                                << "  end:" << header.end_angle
                                << "  data_size:" << header.data_size
                                << "  data_position:" << header.data_position
                                << "  measure_size:" << header.measure_size
                                << "  time:" << header.time
                                << "  laser_steady_time:" << laser_steady_time
                                << "  begin_point_index:" << begin_point_index
                                << "  angle_increment:" << angle_increment
                                << "  all_point_num:" << all_point_num
                                << "  now_time:" << now_time
                                << "  last_data_time_:" << last_data_time_
                                << "  now_time - last_data_time_:" << now_time - last_data_time_
                                );
            }
            // 接收完一圈数据
            if (header.end_angle == 0 && header.data_position == header.measure_size)
            {                
                scan_data_.emplace_back();
                scan_data_.back().time_increment = 0;
                if (scan_data_.size() > 5) {
                    scan_data_.pop_front();
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("hins_le_node"),"buffer data too many, drops it");
                }
                data_notifier_.notify_one();
                last_data_time_ = hins::Now();
                ret = true;
                // std::cout << std::endl;
            }

            if (FindPacketStart() >= 0) {
                ret = true;
            }
        }
    }

    return ret;
}

/**
 * @brief 获取避障数据
 */
bool LaserDataReceiver::GetAreaData(int16_t head_index)
{
    // std::cout << "GetAreaData begin." << std::endl;
    bool ret = false;
    // 寻找帧头并处理数据
    if(ring_buffer_.size() - head_index >= kHSAreaDataPackageSize)
    {
        ring_buffer_.erase_begin(head_index);                     // 删除 帧头前的数据
        head_index = 0;

        // 获取帧
        char data_buf[kHSAreaDataPackageSize];
        ReadBufferFront(data_buf, kHSAreaDataPackageSize);
        have_block_ = uint8_t(data_buf[7]);
        _now_channel_ = uint8_t(data_buf[5]);

        #ifdef DEBUG
        if(data_buf[7]==0x00)           // 00为检测到物体
          have_block_ = true;
        else
          have_block_ = false;

        std::cout << "当前通道值：" << int(data_buf[5]) << " "
                           << "输出状态：" << int(data_buf[7]) << " "
                           << "have_block_:" << have_block_ << std::endl;
        
        for(int i = 0; i < 23;i++ )
        {
            std::cout << int(data_buf[i]) << "    " ; 
        }
        std::cout << std::endl;
    #endif

        ret = true;
        ring_buffer_.erase_begin(head_index + kHSAreaDataPackageSize);       // 删除 ring_buffer_ 的数据
        return ret;
    }
    return ret;    
}



// 将ring_buffer_前num_bytes的数据拷贝到dst
void LaserDataReceiver::ReadBufferFront(char* dst, const uint16_t& num_bytes)
{
    if (ring_buffer_.size() < num_bytes)
        throw std::exception();

    char* pone = ring_buffer_.array_one().first; // 指向环形缓冲区ring_buffer_的array_one的开始地址
    std::size_t pone_size = ring_buffer_.array_one().second; // ring_buffer_的array_one的大小
    char* ptwo = ring_buffer_.array_two().first; // 指向环形缓冲区ring_buffer_的array_two的开始地址

    if (pone_size >= num_bytes) // ring_buffer_的array_one的大小大于num_bytes
    {
        std::memcpy(dst, pone, num_bytes); // 复制环形缓冲区的数据到dst中
    } else {
        std::memcpy(dst, pone, pone_size); // 复制环形缓冲区的array_one数据到dst中
        std::memcpy(dst + pone_size, ptwo, num_bytes - pone_size); // 复制环形缓冲区的array_two数据到dst中
    }
}



void LaserDataReceiver::connect_handler(const boost::system::error_code& error)
{
  if (!error)
  {
    is_connected_ = true;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("hins_le_node"),"async_connect succeed");
  }
  else
  {
    is_connected_ = false;
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("hins_le_node"), "Exception:" << error);
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("hins_le_node"),"async_connect fail");
  }
}
}
