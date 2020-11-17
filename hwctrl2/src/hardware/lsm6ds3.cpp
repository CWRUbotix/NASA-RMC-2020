#include "hardware/lsm6ds3.h"

#include <ros/ros.h>

#include <linux/spi/spidev.h>


Lsm6ds3::Lsm6ds3(
    ros::NodeHandle nh, const std::string& name, uint32_t id, const std::string& topic,
	uint32_t topic_size, ros::Duration update_period, boost::shared_ptr<Spi> spi, boost::movelib::unique_ptr<Gpio> cs, 
    uint32_t samples
) : SpiSensor<PubData>(nh, name, SensorType::LSM6DS3, id, topic, topic_size, update_period, spi, LSM6DS3_SPI_SPEED, LSM6DS3_SPI_MODE, std::move(cs)),
  m_sample_buf1(samples), m_sample_buf2(samples), m_sample_buf3(samples),
  m_sample_buf4(samples), m_sample_buf5(samples), m_sample_buf6(samples)
{
}

Lsm6ds3::~Lsm6ds3() {
    // do we need something here?
}

void Lsm6ds3::update() {
    double xl_data[3] = {};
    double g_data[3] = {};
    m_spi->set_speed(m_spi_speed);
    m_spi->set_mode(m_spi_mode);
    ros::Duration(0.0005).sleep();
    read_all_data(xl_data, g_data);

    static uint32_t seq = 0;
    sensor_msgs::Imu msg;
    msg.header.seq = seq++; // do we really need this?
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = m_name;

    // do not set for now
    msg.orientation_covariance[0] = -1.0;

    if(m_calibrations.size() > 6) {
        // this seems super inefficient.
        // TODO: make this more efficient (ie: dont search list every iteration)
        const Calibration& cal_xl_x = get_calibration_by_name("IMU_XL_X").get();
        const Calibration& cal_xl_y = get_calibration_by_name("IMU_XL_Y").get();
        const Calibration& cal_xl_z = get_calibration_by_name("IMU_XL_Z").get();
        const Calibration& cal_g_x  = get_calibration_by_name("IMU_G_X").get();
        const Calibration& cal_g_y  = get_calibration_by_name("IMU_G_Y").get();
        const Calibration& cal_g_z  = get_calibration_by_name("IMU_G_Z").get();
        msg.linear_acceleration.x = cal_xl_x.scale*xl_data[0] + cal_xl_x.offset;
        msg.linear_acceleration.y = cal_xl_y.scale*xl_data[1] + cal_xl_y.offset;
        msg.linear_acceleration.z = cal_xl_z.scale*xl_data[2] + cal_xl_z.offset;
        msg.angular_velocity.x = cal_g_x.scale*g_data[0] + cal_g_x.offset;
        msg.angular_velocity.y = cal_g_y.scale*g_data[1] + cal_g_y.offset;
        msg.angular_velocity.z = cal_g_z.scale*g_data[2] + cal_g_z.offset;
    } else {
        msg.linear_acceleration.x = xl_data[0];
        msg.linear_acceleration.y = xl_data[1];
        msg.linear_acceleration.z = xl_data[2];
        msg.angular_velocity.x = g_data[0];
        msg.angular_velocity.y = g_data[1];
        msg.angular_velocity.z = g_data[2];
    }

    msg.linear_acceleration_covariance = m_cov_xl;
	msg.angular_velocity_covariance  = m_cov_g;

    // add data to buffers
    m_sample_buf1.push_back((float) xl_data[0]);
    m_sample_buf2.push_back((float) xl_data[1]);
    m_sample_buf3.push_back((float) xl_data[2]);
    m_sample_buf4.push_back((float) g_data[0]);
    m_sample_buf5.push_back((float) g_data[1]);
    m_sample_buf6.push_back((float) g_data[2]);

    // this is for recalculating the covariance matrix, dont need to do this for now
    // TODO: wait for software team to get mad at us.
    // if(m_sample_buf1.full()) {
    //     // get the average of the values here
    //     m_rm1 = get_sample_buf_mean(m_sample_buf1);
    //     m_rm2 = get_sample_buf_mean(m_sample_buf2);
    //     m_rm3 = get_sample_buf_mean(m_sample_buf3);
    //     m_rm4 = get_sample_buf_mean(m_sample_buf4);
    //     m_rm5 = get_sample_buf_mean(m_sample_buf5);
    //     m_rm6 = get_sample_buf_mean(m_sample_buf6);

    //     // recalculate variance matricies
    // }

    m_pub.publish(msg);
    m_update = false;
}

void Lsm6ds3::setup() {
    config_spi_settings();
    ros::Duration(0.001).sleep();
    soft_reset();
    ros::Duration(0.1).sleep();

    uint8_t buf[2] = {};
    buf[0] = (uint8_t) ImuReg::WHO_AM_I;
    buf[1] = 0x00;
    LSM6DS3_SET_READ_MODE(buf[0]);
    ROS_INFO("Trying the IMU");
    transfer_to_device(buf, 2);
    ros::Duration(0.002).sleep();

    if(buf[1] == LSM6DS3_WHO_AM_I_ID){
        power_on_accel(LSM6DS3_ODR_208_HZ | LSM6DS3_FS_XL_2G);
        ros::Duration(0.002).sleep();
        power_on_gyro(LSM6DS3_ODR_208_HZ | LSM6DS3_FS_G_250_DPS);
        ros::Duration(0.002).sleep();
        ROS_INFO("IMU setup success!");
        m_is_setup = true;
    }else{
        ROS_ERROR("IMU setup failed :(, read [%x , %x]...", buf[0], buf[1]);
    }
    // m_is_setup = true;
}

void Lsm6ds3::calibrate(std::vector<Calibration>& cals) {
    
}

void Lsm6ds3::power_on_accel(uint8_t config_byte) {
    // power on accel
    uint8_t buf[2] = {};
    buf[0] = (uint8_t) ImuReg::CTRL1_XL;
    buf[1] = config_byte;

    transfer_to_device(buf, 2);
}

void Lsm6ds3::power_on_gyro(uint8_t config_byte) {
    // power on gyro
    uint8_t buf[2] = {};
    buf[0] = (uint8_t) ImuReg::CTRL2_G;
    buf[1] = config_byte;

    transfer_to_device(buf, 2);
}

float Lsm6ds3::read_accel(Axis axis, float fs) {
    uint8_t buf[3] = {};
    switch(axis) {
        case Axis::X: {
            buf[0] = (uint8_t) ImuReg::OUTX_L_XL;
            break;
        }
        case Axis::Y: {
            buf[0] = (uint8_t) ImuReg::OUTY_L_XL;
            break;
        }
        case Axis::Z: {
            buf[0] = (uint8_t) ImuReg::OUTZ_L_XL;
            break;
        }
    }
    LSM6DS3_SET_READ_MODE(buf[0]);
    transfer_to_device(buf, 3);

    int16_t val = buf[1] | (buf[2] << 8);
    return (xl_fs * ((float)val))/32767.0;
}

float Lsm6ds3::read_gyro(Axis axis, float fs) {
    uint8_t buf[3] = {};

    switch(axis) {
        case Axis::X: {
            buf[0] = (uint8_t) ImuReg::OUTX_L_G;
            break;
        }
        case Axis::Y: {
            buf[0] = (uint8_t) ImuReg::OUTY_L_G;
            break;
        }
        case Axis::Z: {
            buf[0] = (uint8_t) ImuReg::OUTZ_L_G;
            break;
        }
    }
    LSM6DS3_SET_READ_MODE(buf[0]);

    transfer_to_device(buf, 3);

    int16_t val = buf[1] | (buf[2] << 8);
    return degrees_to_radians((g_fs * ((float)val))/32767.0);
}

void Lsm6ds3::read_all_data(double* xl_data, double* gyro_data) {
    uint8_t buf[13];
    memset(buf, 0, 13);
    buf[0] = (uint8_t) ImuReg::OUTX_L_G;
    LSM6DS3_SET_READ_MODE(buf[0]);

    transfer_to_device(buf,13);

    int16_t temp;
    int ind = 1;
    temp = buf[ind++];
    temp |= buf[ind++] << 8;
    gyro_data[0] = (double)(g_fs * temp)/32767.0; // x axis dps
    degrees_to_radians(gyro_data[0]);

    temp = buf[ind++];
    temp |= buf[ind++] << 8;
    gyro_data[1] = (double)(g_fs * temp)/32767.0; // y axis dps
    degrees_to_radians(gyro_data[1]);

    temp = buf[ind++];
    temp |= buf[ind++] << 8;
    gyro_data[2] = (double)(g_fs * temp)/32767.0; // z axis dps
    degrees_to_radians(gyro_data[2]);

    temp = buf[ind++];
    temp |= buf[ind++] << 8;
    xl_data[0] = (double)(xl_fs * temp)/32767.0; // x axis dps

    temp = buf[ind++];
    temp |= buf[ind++] << 8;
    xl_data[1] = (double)(xl_fs * temp)/32767.0; // y axis dps

    temp = buf[ind++];
    temp |= buf[ind++] << 8;
    xl_data[2] = (double)(xl_fs * temp)/32767.0; // z axis dps
}

void Lsm6ds3::soft_reset() {
    uint8_t buf[2];
    memset(buf, 0, 2);
    buf[0] = ImuReg::CTRL3_C;
    LSM6DS3_SET_READ_MODE(buf[0]);

    transfer_to_device(buf, 2);

    uint8_t og_val = buf[1];
    buf[1] |= LSM6DS3_BOOT; // initiate software reset and reboot memory content
    LSM6DS3_SET_WRITE_MODE(buf[0]); // write to the same register, now with modification

    // TODO: is there a better way to "spin briefly". nop operation maybe?
    int n = 0;
    for(int i = 0; i < 10000; i++){
        n ++;
        // spin briefly
    }
    transfer_to_device(buf, 2);

    n = 0;
    for(int i = 0; i < 10000; i++){
        n++;
        // spin briefly
    }

    buf[0] = CTRL3_C;
    buf[1] = og_val;
    transfer_to_device(buf, 2);
}