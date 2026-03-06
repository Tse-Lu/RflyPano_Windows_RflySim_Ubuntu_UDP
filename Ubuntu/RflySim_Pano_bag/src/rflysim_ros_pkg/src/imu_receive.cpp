#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

#pragma pack(push, 1)
// corresponding to VisionCaptureApi in SensorReqCopterSim (client)
struct SensorReqCopterSim {
    int32_t sensorType;    // 0 for IMU
    int32_t updateFreq;    // Frequency 200
    uint8_t IP[4];         // Ubuntu IP
    int32_t port;          // 31000
};

// 40 bytes
struct RflyImuRaw {
    int32_t checksum;    // 1234567898
    int32_t seq;
    double  timestamp;
    float   acc[3];
    float   rate[3];
};
#pragma pack(pop)

std::string winIP = "192.168.137.1"; // Windows IP
std::string ubuIP = "192.168.137.2";   // Ubuntu IP
int copterID = 1;

void sendImuReq(int copterID, const char* targetIP, const char* localIP, int freq) {
    int sendfd = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(30100 + copterID - 1); // CopterSim port
    servaddr.sin_addr.s_addr = inet_addr(targetIP);

    SensorReqCopterSim req;
    req.sensorType = 0;
    req.updateFreq = freq;
    req.port = 31000 + copterID - 1;

    int ip[4];
    sscanf(localIP, "%d.%d.%d.%d", &ip[0], &ip[1], &ip[2], &ip[3]);
    for(int i=0; i<4; i++) req.IP[i] = (uint8_t)ip[i];

    sendto(sendfd, (const char*)&req, sizeof(req), 0, (const struct sockaddr *)&servaddr, sizeof(servaddr));
    close(sendfd);
    ROS_INFO("Sent IMU request to %s, asking data back to %s:%d", targetIP, localIP, req.port);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_bridge");
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 100);

    nh.param<std::string>("windows_ip", winIP, "192.168.137.1");
    nh.param<std::string>("ubuntu_ip", ubuIP, "192.168.137.2");
    nh.param<int>("copter_id", copterID, 1);
    int port = 31000 + copterID - 1;

    sendImuReq(copterID, winIP.c_str(), ubuIP.c_str(), 200);

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(port);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        ROS_ERROR("Bind failed!");
        return -1;
    }

    ROS_INFO("Listening for IMU data...");

    char buffer[1024];
    RflyImuRaw raw_data;
    while (ros::ok()) {
        ssize_t n = recvfrom(sockfd, buffer, sizeof(buffer), 0, NULL, NULL);
        if (n == sizeof(RflyImuRaw)) {
            std::memcpy(&raw_data, buffer, sizeof(RflyImuRaw));
            if (raw_data.checksum == 1234567898) {
                sensor_msgs::Imu imu_msg;
                imu_msg.header.stamp = ros::Time(raw_data.timestamp);
                imu_msg.header.frame_id = "imu_link";
                imu_msg.linear_acceleration.x = raw_data.acc[0];
                imu_msg.linear_acceleration.y = raw_data.acc[1];
                imu_msg.linear_acceleration.z = raw_data.acc[2];
                imu_msg.angular_velocity.x = raw_data.rate[0];
                imu_msg.angular_velocity.y = raw_data.rate[1];
                imu_msg.angular_velocity.z = raw_data.rate[2];
                imu_pub.publish(imu_msg);
            }
        }
        ros::spinOnce();
    }
    close(sockfd);
    return 0;
}