#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <stdio.h>
#include <tf/tf.h>
#define UART_BUFF_SIZE 512
#define PROTOCAL_FRAME_MAX_SIZE 44

enum step_status{STEP_HEADER_1,STEP_HEADER_2,STEP_HEADER_3,STEP_LENGTH,STEP_DATA,STEP_SUMCHECK,STEP_ADDCHECK};
void Pkg_Decode(uint8_t *inorder_buf,uint8_t mode_id);
void read_and_unpack(void);
float raw_data[6];
float euler_angle[3];
float quaternion[4];
serial::Serial ser;

int main (int argc, char** argv){
    ros::init(argc, argv, "imu6050_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    int serial_baudrate=1000000;
    std::string serial_port="/dev/ttyUSB1";
    std::string frame_id = "base_link";
    std::string topic_name = "imu";
    nh_private.getParam("serial_port", serial_port); 
    nh_private.getParam("serial_baudrate", serial_baudrate);
    nh_private.getParam("frame_id", frame_id);
    nh_private.getParam("topic_name", topic_name);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(topic_name, 1);
    sensor_msgs::Imu imu_msg;
    ros::Rate loop_rate(1300);

    while(ros::ok()){
        if(!ser.isOpen()){
            try
            {
                ser.setPort(serial_port.c_str());
                ser.setBaudrate((uint32_t)serial_baudrate);
                serial::Timeout to = serial::Timeout::simpleTimeout(1);
                ser.setTimeout(to);
                ser.open();
            }
            catch (serial::IOException& e)
            {
                ROS_ERROR_STREAM("Unable to open port ");
                // ser.close();
                //return -1;
            }

            if(ser.isOpen()){
                ROS_INFO_STREAM("Serial Port initialized");
                // break;
            }else{
                // ser.close();
                //return -1;
            }
        }else{
            size_t n = ser.available();
            if(n!=0){
                // Read Data and Decode Package
                read_and_unpack();

                imu_msg.header.stamp = ros::Time::now();
                imu_msg.header.frame_id = frame_id;

                imu_msg.angular_velocity.x = raw_data[3];
                imu_msg.angular_velocity.y = raw_data[4];
                imu_msg.angular_velocity.z = raw_data[5];

                imu_msg.linear_acceleration.x = raw_data[0];
                imu_msg.linear_acceleration.y = raw_data[1];
                imu_msg.linear_acceleration.z = raw_data[2];

                imu_msg.orientation.x = quaternion[1];
                imu_msg.orientation.y = quaternion[2];
                imu_msg.orientation.z = quaternion[3];
                imu_msg.orientation.w = quaternion[0];

                imu_pub.publish(imu_msg);
            }
        }
        ros::spinOnce();
        loop_rate.sleep();

    }
    ser.close();
    return 0;
}

void Pkg_Decode(uint8_t *inorder_buf, uint8_t mode_id){
    switch (mode_id)
    {
        case 0x01: //acc and gyro data
            for (int i=0; i < 5;i+=2){
                raw_data[i/2] = (int16_t)(inorder_buf[4+i] | inorder_buf[5+i] << 8); 
                raw_data[i/2] /= 1000.0f; 
            }
            for (int i=6; i < 11;i+=2){
                raw_data[i/2] = (int16_t)(inorder_buf[4+i] | inorder_buf[5+i] << 8); 
                raw_data[i/2] /= 1000.0f; 
            } 
        break;
        case 0x03: //euler angle 
            for (int i=0; i < 5;i+=2){
                euler_angle[i/2] = (int16_t)(inorder_buf[4+i] | inorder_buf[5+i] << 8); 
                euler_angle[i/2] /= 100.0f; 
            } 
        break;   
        case 0x04: //Quaternion  
            for (int i=0; i < 7;i+=2){
            quaternion[i/2] = (int16_t)(inorder_buf[4+i] | inorder_buf[5+i] << 8) ; 
            quaternion[i/2] /= 10000.0f; 
        } 
        break;
        default:
        break;
    }
}

void read_and_unpack(void)
{
    uint8_t mode_id;
    uint8_t byte = 0;
    uint32_t read_len;
    int32_t buff_read_index;
    uint8_t sumcheck;
    uint8_t addcheck;
    uint16_t data_count;
    uint16_t data_len;
    step_status unpack_step = STEP_HEADER_1;
    int32_t index;
    uint8_t protocol_packet[PROTOCAL_FRAME_MAX_SIZE];
    uint8_t computer_rx_buf[UART_BUFF_SIZE];

    read_len = ser.read(computer_rx_buf, UART_BUFF_SIZE);
    buff_read_index = 0;
    
    while (read_len--)
    {
        byte = computer_rx_buf[buff_read_index++];
        switch(unpack_step)
        {
            case STEP_HEADER_1:
            {
                if(byte == 0xAA)
                {
                    index = 0;   
                    unpack_step = STEP_HEADER_2;
                    protocol_packet[index++] = byte;
                }
                else
                {
                    unpack_step = STEP_HEADER_1;
                    index = 0;
                }
            }break;
            case STEP_HEADER_2:
            {
                if(byte == 0xFF){
                    unpack_step = STEP_HEADER_3;
                    protocol_packet[index++] = byte;
                }else{
                    unpack_step = STEP_HEADER_1;
                    index = 0;
                }
            }break;
            case STEP_HEADER_3:
            {
                if((byte == 0x01)||(byte == 0x03)||(byte == 0x04))
                {
                    mode_id = byte;
                    unpack_step = STEP_LENGTH;
                    protocol_packet[index++] = byte;
                }else{
                    unpack_step = STEP_HEADER_1;
                    index = 0;
                }
            }break;
            case STEP_LENGTH:
            {
                data_len = byte;
                protocol_packet[index++] = byte;
                unpack_step = STEP_DATA;
                data_count = 0;
                
            }break;
            case STEP_DATA:
            {
                if(data_count < data_len-1)
                {
                    protocol_packet[index++] = byte;
                    data_count++;
                }else{
                    protocol_packet[index++] = byte;
                    sumcheck = 0;
                    addcheck = 0;
                    unpack_step = STEP_SUMCHECK;
                }
            }break;
            case STEP_SUMCHECK:
            {
                protocol_packet[index++] = byte;
                unpack_step = STEP_ADDCHECK;
            }break;
            case STEP_ADDCHECK:
            {
                protocol_packet[index++] = byte;
                for (int i=0;i<protocol_packet[3]+4;i++){
                        sumcheck += protocol_packet[i];
                        addcheck += sumcheck;
                    }
                if ((protocol_packet[protocol_packet[3]+4]==sumcheck)&&(protocol_packet[protocol_packet[3]+5]==addcheck)){
                    Pkg_Decode(protocol_packet,mode_id);
                }
                unpack_step = STEP_HEADER_1;
                index = 0;   
            }break;  
            default:
            {
                unpack_step = STEP_HEADER_1;
                index = 0;
            }break;
        }
    }
}
