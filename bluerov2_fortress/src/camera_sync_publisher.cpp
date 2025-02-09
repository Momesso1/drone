#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <mutex>
#include <sensor_msgs/msg/imu.hpp>
namespace gazebo_sync
{

class CameraSyncPublisher : public rclcpp::Node
{
public:
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using CompressedImage = sensor_msgs::msg::CompressedImage;
using IMU = sensor_msgs::msg::Imu;
  CameraSyncPublisher()
  : Node("camera_sync_publisher")
  {
    // Definição do QoS
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().reliable();

       imu_sub_ = this->create_subscription<IMU>(
      "/bluerov2/imu", qos,
      std::bind(&CameraSyncPublisher::imu_callback, this, std::placeholders::_1));

    // Publicador do tópico de IMU sincronizado
    imu_pub_ = this->create_publisher<IMU>("/sync/imu", qos);
    // Subscrição para os tópicos de câmera
    left_camera_info_sub_ = this->create_subscription<CameraInfo>(
      "/gazebo/stereo_camera/left/camera_info", qos,
      std::bind(&CameraSyncPublisher::left_camera_info_callback, this, std::placeholders::_1));

    right_camera_info_sub_ = this->create_subscription<CameraInfo>(
      "/gazebo/stereo_camera/right/camera_info", qos,
      std::bind(&CameraSyncPublisher::right_camera_info_callback, this, std::placeholders::_1));

    left_image_sub_ = this->create_subscription<CompressedImage>(
      "/gazebo/stereo_camera/left/compressed", qos,
      std::bind(&CameraSyncPublisher::left_image_callback, this, std::placeholders::_1));

    right_image_sub_ = this->create_subscription<CompressedImage>(
      "/gazebo/stereo_camera/right/compressed", qos,
      std::bind(&CameraSyncPublisher::right_image_callback, this, std::placeholders::_1));

    // Publicadores de tópicos
    left_camera_info_pub_ = this->create_publisher<CameraInfo>("/stereo_camera/left/camera_info_throttle", qos);
    left_image_pub_ = this->create_publisher<CompressedImage>("/stereo_camera/left/image_raw_throttle/compressed", qos);
    right_camera_info_pub_ = this->create_publisher<CameraInfo>("/stereo_camera/right/camera_info_throttle", qos);
    right_image_pub_ = this->create_publisher<CompressedImage>("/stereo_camera/right/image_raw_throttle/compressed", qos);

    // Timer para publicar
    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(80),  // 20 Hz = 50 ms
      std::bind(&CameraSyncPublisher::publish_messages, this));

    RCLCPP_INFO(this->get_logger(), "CameraSyncPublisher initialized");
  }

private:
  rclcpp::Subscription<CameraInfo>::SharedPtr left_camera_info_sub_;
  rclcpp::Subscription<CompressedImage>::SharedPtr left_image_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr right_camera_info_sub_;
  rclcpp::Subscription<CompressedImage>::SharedPtr right_image_sub_;

  rclcpp::Publisher<CameraInfo>::SharedPtr left_camera_info_pub_;
  rclcpp::Publisher<CompressedImage>::SharedPtr left_image_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr right_camera_info_pub_;
  rclcpp::Publisher<CompressedImage>::SharedPtr right_image_pub_;
 rclcpp::Subscription<IMU>::SharedPtr imu_sub_;
  rclcpp::Publisher<IMU>::SharedPtr imu_pub_;

  IMU::SharedPtr last_imu_;

  // Mutex para proteger o acesso aos dados
  std::mutex data_mutex_;
  std::mutex imu_mutex_;
  CameraInfo::SharedPtr last_left_camera_info_;
  CompressedImage::SharedPtr last_left_image_;
  CameraInfo::SharedPtr last_right_camera_info_;
  CompressedImage::SharedPtr last_right_image_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  void imu_callback(const IMU::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  auto current_time = this->now();

  // Atualizar o timestamp para o tempo atual (segundos desde 1970)
  msg->header.stamp.sec = static_cast<int32_t>(current_time.seconds());
  msg->header.stamp.nanosec = static_cast<uint32_t>((current_time.nanoseconds()) % 1'000'000'000);

  // Calcular a diferença entre os timestamps
  if (last_left_image_) // Verifica se já há uma imagem recebida
  {
    auto image_stamp = last_left_image_->header.stamp.sec + 
                       (last_left_image_->header.stamp.nanosec / 1e9);
    auto imu_stamp = msg->header.stamp.sec + (msg->header.stamp.nanosec / 1e9);
    auto time_difference = std::abs(image_stamp - imu_stamp);

    
  }

  // Publicar a mensagem com o timestamp atualizado
  imu_pub_->publish(*msg);

  //RCLCPP_INFO(this->get_logger(), "Published IMU message with updated timestamp");
}

  // Callbacks para os tópicos
  void left_camera_info_callback(const CameraInfo::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_left_camera_info_ = msg;
    //RCLCPP_INFO(this->get_logger(), "Received left camera info message");
  }

  void right_camera_info_callback(const CameraInfo::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_right_camera_info_ = msg;
    //RCLCPP_INFO(this->get_logger(), "Received right camera info message");
  }

  void left_image_callback(const CompressedImage::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_left_image_ = msg;
    //RCLCPP_INFO(this->get_logger(), "Received left image message");
  }

  void right_image_callback(const CompressedImage::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_right_image_ = msg;
    //RCLCPP_INFO(this->get_logger(), "Received right image message");
  }

  // Função para publicar as mensagens sincronizadas
  void publish_messages()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (last_left_camera_info_ && last_left_image_ && last_right_camera_info_ && last_right_image_)
    {
      auto current_time = this->now();

      // Aqui você pode sincronizar as mensagens com base nos timestamps
      last_left_camera_info_->header.stamp = current_time;
      last_left_image_->header.stamp = current_time;
      last_right_camera_info_->header.stamp = current_time;
      last_right_image_->header.stamp = current_time;

      left_camera_info_pub_->publish(*last_left_camera_info_);
      left_image_pub_->publish(*last_left_image_);
      right_camera_info_pub_->publish(*last_right_camera_info_);
      right_image_pub_->publish(*last_right_image_);

      //RCLCPP_INFO(this->get_logger(), "Published synchronized messages at 20 Hz");
    }
    else
    {
      //RCLCPP_WARN(this->get_logger(), "Waiting for synchronized messages...");
    }
  }
};

} // namespace gazebo_sync

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gazebo_sync::CameraSyncPublisher>());
  rclcpp::shutdown();
  return 0;
}
