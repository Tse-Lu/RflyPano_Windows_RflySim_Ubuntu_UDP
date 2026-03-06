#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <map>
#include <vector>

// RflySim Python API -> struct.pack('4i1d'), in total 24 bytes
#pragma pack(push, 1)
struct RflyPacketHeader {
    int32_t checksum;    // 1234567890
    int32_t packLen;     // total length of current package, including head
    int32_t packSeq;     // index of current slice
    int32_t totalPack;   // number of slices
    double timestamp;    // 
};
#pragma pack(pop)

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "rec_node");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    std::string camera_id;
    int PORT;
    nh.param<std::string>("camera_id", camera_id, "FrontLeft");
    nh.param<int>("port", PORT, 9999);

    image_transport::Publisher pub = it.advertise("/camera/" + camera_id, 1);

    int sockfd;
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket failed");
        return -1;
    }

    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    int nRecvBuf = 10 * 1024 * 1024; 
    setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, (const char*)&nRecvBuf, sizeof(int));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind failed");
        return -1;
    }

    const int32_t imgFlag = 1234567890;
    const int MAX_UDP_SIZE = 65535;
    char recv_buffer[MAX_UDP_SIZE];
    
    std::map<int, std::vector<char>> frame_data_map; 
    double last_timestamp = 0;
    int expected_packs = 0;

    std::cout << "Listening on UDP port " << PORT << " for RflySim images..." << std::endl;

    while (ros::ok()) {
        struct sockaddr_in remote_addr;
        socklen_t addr_len = sizeof(remote_addr);
        
        int len = recvfrom(sockfd, recv_buffer, MAX_UDP_SIZE, 0, (struct sockaddr *)&remote_addr, &addr_len);
        
        if (len < sizeof(RflyPacketHeader)) continue;

        RflyPacketHeader* header = (RflyPacketHeader*)recv_buffer;

        if (header->checksum != imgFlag || header->packLen != len) {
            continue;
        }

        if (header->timestamp != last_timestamp) {
            frame_data_map.clear();
            last_timestamp = header->timestamp;
            expected_packs = header->totalPack;
        }

        std::vector<char> pure_data(recv_buffer + sizeof(RflyPacketHeader), recv_buffer + len);
        frame_data_map[header->packSeq] = pure_data;

        if (frame_data_map.size() == expected_packs && expected_packs > 0) {
            std::vector<uchar> full_image_data;
            for (int i = 0; i < expected_packs; i++) {
                full_image_data.insert(full_image_data.end(), frame_data_map[i].begin(), frame_data_map[i].end());
            }

            cv::Mat image = cv::imdecode(full_image_data, cv::IMREAD_COLOR);
            if (!image.empty()) {
                std_msgs::Header ros_header;
                ros_header.stamp = ros::Time(header->timestamp); // header->timestamp is the time relative to the time origin of RflySim simulation
                ros_header.frame_id = camera_id;
                
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(ros_header, "bgr8", image).toImageMsg();
                pub.publish(msg);
            }
            
            frame_data_map.clear();
            expected_packs = 0;
        }
        ros::spinOnce();
    }

    close(sockfd);
    return 0;
}