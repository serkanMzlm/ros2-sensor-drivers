#include "vl53l5cx_mpu6050_driver.hpp"

using namespace std::chrono_literals;

MultiSensor::MultiSensor(): Node("multi_sensor"), 
   vl53l5cx{std::make_unique<VL53L5CXDriver>()},
   mpu6050{std::make_unique<MPU6050>()}
{
    declareParameters();
    mpu6050->setGyroscopeRange(
        static_cast<GyroRange>(this->get_parameter("gyro_range").as_int())
    );
    mpu6050->setAccelerometerRange(
        static_cast<AccelRange>(this->get_parameter("accel_range").as_int())
    );
    mpu6050->setDlpfBandwidth(
        static_cast<DlpfBandwidth>(this->get_parameter("dlpf_bandwidth").as_int())
    );
    mpu6050->setGyroOffset(this->get_parameter("gyro_x_offset").as_double(),
                        this->get_parameter("gyro_y_offset").as_double(),
                        this->get_parameter("gyro_z_offset").as_double());
    mpu6050->setAccOffset(this->get_parameter("accel_x_offset").as_double(),
                        this->get_parameter("accel_y_offset").as_double(),
                        this->get_parameter("accel_z_offset").as_double());
    mpu6050->calibrated = this->get_parameter("calibrate").as_bool();
    // if (this->get_parameter("calibrate").as_bool()) {
    //     RCLCPP_INFO(this->get_logger(), "Calibrating...");
    //     mpu6050->calibrate();
    // }

    // mpu6050->printConfig();
    mpu6050->printOffsets();

     imu_pub = this->create_publisher<ImuMsg>("imu", 10);
    range_pub = this->create_publisher<RangeMsg>("height", 10);

    std::chrono::duration<int64_t, std::milli> frequency =
                        1000ms / this->get_parameter("frequency").as_int();

    if(vl53l5cx->initVL53L5CX() < 0){
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize VL53L5CX...");
        vl53l5cx_flag = false;
    }

    timer_ = this->create_wall_timer(frequency, std::bind(
                        &MultiSensor::callbackSensor, this));
}


void MultiSensor::getRange(){
    RangeMsg msg;
    msg.range = vl53l5cx->getRange();
    if(msg.range >= 0){
        range_pub->publish(msg);
    }
}

void MultiSensor::getIMU(){
    auto message = ImuMsg();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "base_link";
    message.linear_acceleration_covariance = {0};
    message.linear_acceleration.x = mpu6050->getAccelerationX();
    message.linear_acceleration.y = mpu6050->getAccelerationY();
    message.linear_acceleration.z = mpu6050->getAccelerationZ();
    message.angular_velocity_covariance[0] = {0};
    message.angular_velocity.y = mpu6050->getAngularVelocityX();
    message.angular_velocity.x = -mpu6050->getAngularVelocityY();
    message.angular_velocity.z = mpu6050->getAngularVelocityZ();
    // Invalidate quaternion
    message.orientation_covariance[0] = -1;
    message.orientation.x = 0;
    message.orientation.y = 0;
    message.orientation.z = 0;
    message.orientation.w = 0;
    imu_pub->publish(message);
}

void MultiSensor::callbackSensor(){
    if(vl53l5cx_flag && (count > 15)){
        getRange();
        count = 0;
    }
    getIMU();
    count++;
}

void MultiSensor::declareParameters(){
    this->declare_parameter<bool>("calibrate", false);
    this->declare_parameter<int>("gyro_range", GYR_250_DPS);
    this->declare_parameter<int>("accel_range", ACC_2_G);
    this->declare_parameter<int>("dlpf_bandwidth", DLPF_260_HZ);
    this->declare_parameter<double>("gyro_x_offset", 0.0);
    this->declare_parameter<double>("gyro_y_offset", 0.0);
    this->declare_parameter<double>("gyro_z_offset", 0.0);
    this->declare_parameter<double>("accel_x_offset", 0.0);
    this->declare_parameter<double>("accel_y_offset", 0.0);
    this->declare_parameter<double>("accel_z_offset", 0.0);
    this->declare_parameter<int>("frequency", 1.0);
}


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiSensor>());
    rclcpp::shutdown();
    return 0;
}