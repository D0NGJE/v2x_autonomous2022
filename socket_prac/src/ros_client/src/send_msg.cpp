#include <iostream>
#include <ros/ros.h>
#include <ctime>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <vector>
#include <sys/socket.h>
#include <sys/types.h>

#include "mi_msgs/RTK.h"
#include "mi_msgs/Car.h"

#define PORT 10000
#define BUFFER_LEN 100
#define CHAT_SIZE 1024
#define BUFF_SIZE 1024
#define LISTEN_QUEUE_SIZE 5

struct PVD_MSG{
    // char entitiyID = 'CE:24:67:03';

    int year_, month_, day_, hour_, min_, sec_;
    int pre_year_, pre_month_, pre_day_, pre_hour_, pre_min_, pre_sec_;

    double lat_;
    double lon_;
    double elevation_;
    double heading_;
    double speed_;
    double transmission_;

    double pre_lat_;
    double pre_lon_;
    double pre_elevation_;
    double pre_heading_;
    double pre_speed_;
    double pre_transmission_;
};

int count = 0;


std::vector<int> arr_hour;
std::vector<int> arr_min;
std::vector<int> arr_sec;
std::vector<double> arr_lat;
std::vector<double> arr_lon;
std::vector<double> arr_elevation;
std::vector<int> arr_speed;
std::vector<double> arr_heading;
std::vector<int> arr_gear; 

void print_msg(PVD_MSG &msg){
    std::cout << "year : " << msg.year_ <<std::endl;
    std::cout << "month : " << msg.month_ <<std::endl;
    std::cout << "day : " << msg.day_ <<std::endl;
    std::cout << "hour : " << msg.hour_ <<std::endl;
    std::cout << "min : " << msg.min_ <<std::endl;
    std::cout << "sec : " << msg.sec_ <<std::endl;
    std::cout << "lat : " << msg.lat_ <<std::endl;
    std::cout << "lon : " << msg.lon_ <<std::endl;
    std::cout << "elevation : " << msg.elevation_ <<std::endl;
    std::cout << "heading : " << msg.heading_ <<std::endl;
    std::cout << "speed : " << msg.speed_ <<std::endl;
    std::cout << "transmission : " << msg.transmission_ <<std::endl;

    std::cout << "pre_year : " << msg.pre_year_ <<std::endl;
    std::cout << "pre_month : " << msg.pre_month_ <<std::endl;
    std::cout << "pre_day : " << msg.pre_day_ <<std::endl;
    std::cout << "pre_hour : " << msg.pre_hour_ <<std::endl;
    std::cout << "pre_min : " << msg.pre_min_ <<std::endl;
    std::cout << "pre_sec : " << msg.pre_sec_ <<std::endl;
    std::cout << "pre_lat : " << msg.pre_lat_ <<std::endl;
    std::cout << "pre_lon : " << msg.pre_lon_ <<std::endl;
    std::cout << "pre_elevation : " << msg.pre_elevation_ <<std::endl;
    std::cout << "pre_heading : " << msg.pre_heading_ <<std::endl;
    std::cout << "pre_speed : " << msg.pre_speed_ <<std::endl;
    std::cout << "pre_transmission : " << msg.pre_transmission_ << "\n" <<std::endl;
}

void gps_callback(const mi_msgs::RTK::ConstPtr &gps_input){
    arr_lat.push_back(gps_input->lat);
    arr_lon.push_back(gps_input->lon);
    arr_elevation.push_back(gps_input->alt);

    // std::cout << "lat : " << gps_input->lat << std::endl;
}

void car_callback(const mi_msgs::Car::ConstPtr &car_input){
    arr_speed.push_back(car_input->velocity);
    arr_heading.push_back(car_input->heading);
    arr_gear.push_back(car_input->gear);

    // std::cout << arr_speed.size() << ", " << arr_heading.size() << ", " << arr_gear.size() << std::endl;
}

void get_time(int &year, int& month, int& day, int& hour, int& min, int& sec){
    time_t timer;
    timer = time(NULL);
    struct tm* t  = localtime(&timer);

    year = t->tm_year + 1900;
    month = t->tm_mon + 1;
    day = t->tm_mday;
    hour = t->tm_hour;
    min = t->tm_min;
    sec = t->tm_sec;
    // std::cout << year << month << day << hour << min << sec << std::endl;
    arr_hour.push_back(hour);
    arr_min.push_back(min);
    arr_sec.push_back(sec);
}

void fill_msg(PVD_MSG &msg){
    int year, month, day, hour, min, sec = 0;
    get_time(year, month, day, hour, min, sec);

    msg.year_ = year;
    msg.month_ = month;
    msg.day_ = day;
    msg.hour_ = arr_hour.back();
    msg.min_ = arr_min.back();
    msg.sec_ = arr_sec.back();
    
    if(!arr_lat.empty() && !arr_lon.empty() && !arr_elevation.empty() && !arr_heading.empty() && !arr_speed.empty() && 
        !arr_gear.empty()){

        msg.lat_ = arr_lat.back();
        msg.lon_ = arr_lon.back();
        msg.elevation_ = arr_elevation.back();
        msg.heading_ = arr_heading.back();
        msg.speed_ = arr_speed.back();
        msg.transmission_ = arr_gear.back();
        print_msg(msg);
        count++;
    }

    if(count > 0){
        msg.pre_year_ = year;
        msg.pre_month_ = month;
        msg.pre_day_ = day;
        msg.pre_hour_ = arr_hour[arr_hour.size() - 2];
        msg.pre_min_ = arr_min[arr_min.size() - 2];
        msg.pre_sec_ = arr_sec[arr_sec.size() - 2];
        msg.pre_lat_ = arr_lat[arr_lat.size() - 2];
        msg.pre_lon_ = arr_lon[arr_lon.size() - 2];
        msg.pre_elevation_ = arr_elevation[arr_elevation.size() - 2];
        msg.pre_heading_ = arr_heading[arr_heading.size() - 2];
        msg.pre_speed_ = arr_speed[arr_speed.size() - 2];
        msg.pre_transmission_ = arr_gear[arr_gear.size() - 2];
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "send_msg");
    ros::NodeHandle nh;

    clock_t start, finish;
    clock_t cur_time, pre_time;
    double duration;

    ros::Subscriber gps_sub = nh.subscribe("/Pose_messages", 1, gps_callback);
    ros::Subscriber car_sub = nh.subscribe("/car_messages", 1, car_callback);

    PVD_MSG msg;

    ros::Rate loop_rate(1);

    if(argc != 2){
        std::cout << "Usage: " << argv[0] << " IPv4-address\n";
    }
    //서버에 접속할 소켓 데이터 구조 생성
    int client_socket;
    struct sockaddr_in server_addr;
    char buff[BUFF_SIZE+5];
    client_socket = socket(PF_INET, SOCK_STREAM, 0); //소켓 생성. PF_INET = ipv4, SOCK_STREAM = TCP
    if(client_socket == -1){
        std::cout << "socket 생성 실패\n";
        exit(1);
    }
    //connet file descroptor 선언
    //memset은 모든 값을 0으로 초기화 해주기위해 클라이언트 실행 시 이용
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(25000); //포트번호 1000 임의 지정
    server_addr.sin_addr.s_addr = inet_addr("192.168.0.21"); //서버 ip 지정
    //서버 접속. 서버의 주소정보로 클라이언트 소켓이 연결요청을 한다
    if(connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1){
        std::cout << "connet fail\n";
        exit(1); 
    }
    // int connect(int sock, struct sockaddr * servaddr, socklen_t addrlen);
    // 함수 성공 시 0, 실패 시 -1 반환. sock : 클라이언트의 소켓 전달. servaddr : 서버의 주소 정보가 담길 주소 값 전달. addrlen : 서버의 주소 정보의 크기를 담은 변수의 주소 값 전달
    while(ros::ok()){
        ros::spinOnce();
        
        // cur_time = clock();
        // duration = cur_time - pre_time;
        // std::cout << duration << std::endl;
        // pre_time = cur_time;
        
        fill_msg(msg);

        send(client_socket, &msg, sizeof(msg), MSG_DONTWAIT);
        // std::cout << 1 << std::endl;
        
        loop_rate.sleep();   
    }
    char buffer[BUFFER_LEN];
    int n = read(client_socket, buffer, BUFFER_LEN);
    std::cout << n << " bytes read\n";
    buffer[n] = '\0';
    fputs(buffer, stdout);
    fflush(stdout);   

    return 0;
}