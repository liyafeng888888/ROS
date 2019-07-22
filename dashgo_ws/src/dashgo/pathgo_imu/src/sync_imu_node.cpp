#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <cmath>

inline double deg2rad(double deg)
{
    return deg * M_PI / 180.;
}

std::string string_to_hex(const std::string& input)
{
    static const char* const lut = "0123456789ABCDEF";
    size_t len = input.length();

    std::string output;
    output.reserve(2 * len);
    for (size_t i = 0; i < len; ++i)
    {
        const unsigned char c = input[i];
        output.push_back(lut[c >> 4]);
        output.push_back(lut[c & 15]);
    }
    return output;
}

std::string char_to_hex(char c){
   static const char* const lut = "0123456789ABCDEF";
   return ""+lut[c >> 4] + lut[c & 15]; 
}


bool zero_orientation_set = false;

bool set_zero_orientation(std_srvs::Empty::Request&,
                          std_srvs::Empty::Response&)
{
  ROS_INFO("Zero Orientation Set.");
  zero_orientation_set = false;
  return true;
}


int main(int argc, char** argv)
{
  serial::Serial ser;
  std::string port;
  std::string tf_parent_frame_id;
  std::string tf_frame_id;
  std::string imu_frame_id;
  double time_offset_in_seconds;

  tf::Quaternion orientation;
  tf::Quaternion zero_orientation;

  std::string partial_line = "";

  ros::init(argc, argv, "sync_imu_node");

  double angle_offset = 1.02;
  //ser.write("$MIA,,,,R,50,,,*EE");
  /* todo  from int  to unsinged int */
  int cfg_baud_rate_=115200;
  int cfg_freq_ = 100;
  ros::NodeHandle nh;
  nh.param<std::string>("/pathgo_imu_node/port", port, "/dev/ttyUSB0");
  nh.param<std::string>("/pathgo_imu_node/tf_parent_frame_id", tf_parent_frame_id, "imu_base");
  nh.param<std::string>("/pathgo_imu_node/tf_frame_id", tf_frame_id, "imu");
  nh.param<std::string>("/pathgo_imu_node/imu_frame_id", imu_frame_id, "imu_base");
  nh.param<double>("/pathgo_imu_node/time_offset_in_seconds", time_offset_in_seconds, 0.0);
  nh.param<double>("/pathgo_imu_node/angle_offset", angle_offset, 1.02);
  nh.param<int>("/pathgo_imu_node/imu_baudrate", cfg_baud_rate_, 115200);
  nh.param<int>("/pathgo_imu_node/imu_freq", cfg_freq_, 115200);

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 50);
  ros::Publisher imu_angle_pub = nh.advertise<std_msgs::Float32>("imu_angle", 50);
  ros::ServiceServer service = nh.advertiseService("set_zero_orientation", set_zero_orientation);


  ros::Rate r(1000); // 1000 hz

  uint8_t size=15;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Duration dur_time;
  while(ros::ok())
  {
    try
    {
      if (ser.isOpen())
      {
        if(ser.available())
        {
          std::string input_0 = ser.read(size); 
          std::string input_hex = string_to_hex(input_0); 
          //ROS_ERROR_STREAM("input_str: " << input_hex);
          std::string input;
          std::string flag;
          std::string::size_type position;
          position = input_hex.find("AA00");
          if (position != input_hex.npos || (input_hex.find("AA")==28)) {

              if (input_hex.find("AA")==28) {
                  position = 28;
              }
              //ROS_ERROR_STREAM("input_str: " << position);
              if(position!=0){
                  std::string input_1 = ser.read((position/2)); 
                  input=input_0+input_1;
              }else{
                  input=input_0;
              }
       
              char *chr = &input[0u+position/2];
              short angle = ((chr[3] & 0xFF) | ((chr[4] << 8) & 0XFF00));
              short angleRate = ((chr[5] & 0xFF) | ((chr[6] << 8) & 0XFF00));
              short int acc_x = ((chr[7] & 0xFF) | ((chr[8] << 8) & 0XFF00));
              short int acc_y = ((chr[9] & 0xFF) | ((chr[10] << 8) & 0XFF00));
              short int acc_z = ((chr[11] & 0xFF) | ((chr[12] << 8) & 0XFF00));

              current_time = ros::Time::now();
              dur_time = current_time - last_time;
              //ROS_ERROR_STREAM("dur_time: " << dur_time.toSec()<<", angle: " << (double)(-1*angle*angle_offset/100.f));
              //ROS_ERROR_STREAM("acc_x: " << acc_x<<"   acc_y: " << acc_y<<"  acc_z: " << acc_z);
              tf::Quaternion orientation = tf::createQuaternionFromYaw((double)-1*angle*angle_offset*3.14/(180*100));
              std_msgs::Float32 imu_angle_mgs;
              imu_angle_mgs.data = (double)(-1*angle*angle_offset/100.f);
              imu_angle_pub.publish(imu_angle_mgs);

              if (!zero_orientation_set)
              {
                  zero_orientation = orientation;
                  zero_orientation_set = true;
              }

              tf::Quaternion differential_rotation;
              differential_rotation = zero_orientation.inverse() * orientation;
              // calculate measurement time
              ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);
              // publish imu message
              sensor_msgs::Imu imu;
              imu.header.stamp = measurement_time;
              imu.header.frame_id = imu_frame_id;
              quaternionTFToMsg(differential_rotation, imu.orientation);
              // i do not know the orientation covariance
              imu.orientation_covariance[0] = 1000000;
              imu.orientation_covariance[1] = 0;
              imu.orientation_covariance[2] = 0;
              imu.orientation_covariance[3] = 0;
              imu.orientation_covariance[4] = 1000000;
              imu.orientation_covariance[5] = 0;
              imu.orientation_covariance[6] = 0;
              imu.orientation_covariance[7] = 0;
              imu.orientation_covariance[8] = 0.000001;
              // angular velocity is not provided
              //imu.angular_velocity_covariance[0] = -1;
              
              imu.angular_velocity.x = 0.0;
              imu.angular_velocity.y = 0.0;
              imu.angular_velocity.z = (double)-1*(angleRate*3.14/(180*100));
              /*
              imu.linear_acceleration.x = (double)(-1*((acc_x+10) * 9.80665/1000.f));
              imu.linear_acceleration.y = (double)(-1*((acc_y+24) * 9.80665/1000.f));
              imu.linear_acceleration.z = (double)(-1*((acc_z-1070) * 9.80665/1000.f));
              */
              imu.linear_acceleration_covariance[0] = -1;
              imu_pub.publish(imu);
              // publish tf transform
              static tf::TransformBroadcaster br;
              tf::Transform transform;
              transform.setRotation(differential_rotation);
              br.sendTransform(tf::StampedTransform(transform, measurement_time, tf_parent_frame_id, tf_frame_id));

          }else{
              ROS_WARN_STREAM("NOT found " );
          } 

          //ROS_ERROR_STREAM("input: " << input.size());
          //ROS_ERROR_STREAM("input_str: " << string_to_hex(input));
        }
          
      }
      else
      {
        // try and open the serial port
        try
        {
          ser.setPort(port);
          ser.setBaudrate(115200);
          serial::Timeout to = serial::Timeout::simpleTimeout(1000);
          ser.setTimeout(to);
          ser.open();
        }
        catch (serial::IOException& e)
        {
          ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
          ros::Duration(5).sleep();
        }

        if(ser.isOpen())
        {
          
            //ser.write("$MIB,RESET*87");
            //ser.write("$MIA,I,B,115200,R,25,D,Y,Y*C4");

            bool enable_transfer = true;
            char *cmd_packet;
            if (asprintf(&cmd_packet, "$MIA,I,B,%u,R,%u,D,%s,Y*  ", cfg_baud_rate_, cfg_freq_,
                   enable_transfer ? "Y" : "N") == -1)
            {
                ROS_ERROR_STREAM("Failed to create command packet");
            }

            size_t cmd_packet_len = strlen(cmd_packet);

            // calculate checksum
            unsigned int checksum = 0;
            for(size_t i = 1; i < cmd_packet_len - 3; ++i )  checksum += cmd_packet[i];
            checksum &= 0xFF;

            char checksum_str[3];
            snprintf(checksum_str, 3, "%X", checksum);
            cmd_packet[cmd_packet_len - 2] = checksum_str[0];
            cmd_packet[cmd_packet_len - 1] = checksum_str[1];

            //std::string cmd_packet_s(cmd_packet, cmd_packet_len);
            //ser.write((const uint8_t*)(cmd_packet_s.c_str()), cmd_packet_len);
            ser.write((const uint8_t*)(cmd_packet), cmd_packet_len);
            free(cmd_packet);
              ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized.");
        }
        else
        {
          //ROS_INFO_STREAM("Could not initialize serial port.");
        }

      }
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
      ser.close();
    }
    ros::spinOnce();
    r.sleep();
  }
}

