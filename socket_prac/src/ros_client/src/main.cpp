#include <iostream>
#include <ros/ros.h>
#include <ctime>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>

#include "mi_msgs/RTK.h"
#include "mi_msgs/Car.h"
// #include "j2735/MessageFrame.h"

#define PORT 10000
#define BUFFER_LEN 100
#define CHAT_SIZE 1024
#define BUFF_SIZE 1024
#define LISTEN_QUEUE_SIZE 5

double lat;
double lon;
double elevation;
double heading;
double speed;
double transmission;

double pre_lat;
double pre_lon;
double pre_elevation;
double pre_heading;
double pre_speed;
double pre_transmission;

void gps_callback(const mi_msgs::RTK::ConstPtr &gps_input){
    lat = gps_input -> lat;
    lon = gps_input -> lon;
    elevation = gps_input -> alt;

    pre_lat = lat;
    pre_lon = lon;
    pre_elevation = elevation;    
}

void car_callback(const mi_msgs::Car::ConstPtr &car_input){
    heading = car_input -> heading;
    speed = car_input -> velocity;
    transmission = car_input -> gear;
    // std::cout << speed << std::endl;

    pre_heading = heading;
    pre_speed = speed;
    pre_transmission = transmission;
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

    std:: cout << year << month << day << hour << min << sec << std::endl;
}



struct PVD_MSG{
    char entitiyID = 'CE:24:67:03';

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


int main(int argc, char **argv){
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int year, month, day, hour, min, sec = 0;
    int pre_year,  pre_month,  pre_day,  pre_hour,  pre_min,  pre_sec = 0;
    get_time(year, month, day, hour, min, sec);


    ros::Subscriber gps_sub = nh.subscribe("/RTK_messages", 1, gps_callback);
    ros::Subscriber car_sub = nh.subscribe("/car_messages", 1, car_callback);

    PVD_MSG msg;
    msg.year_ = year;
    msg.month_ = month;
    msg.day_ = day;
    msg.hour_ = hour;
    msg.min_ = min;
    msg.sec_ = sec;
    msg.pre_year_ = pre_year;
    msg.pre_month_ = pre_month;
    msg.pre_day_ = pre_day;
    msg.pre_hour_  = pre_hour;
    msg.pre_min_ = pre_min;
    msg.pre_sec_ = pre_sec;
    msg.lat_ = lat;
    msg.lon_ = lon;
    msg.elevation_ = elevation;
    msg.heading_ = heading;
    msg.speed_ = speed;
    msg.transmission_ = transmission;
    msg.pre_lat_ = pre_lat;
    msg.pre_lon_ = pre_lon;
    msg.pre_elevation_ = pre_elevation;
    msg.pre_heading_ = pre_heading;
    msg.pre_speed_ = pre_speed;
    msg.pre_transmission_ = pre_transmission;


    pre_year = year;
    pre_month = month;
    pre_day = day;
    pre_hour = hour;
    pre_min = min;
    pre_sec = sec;

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
    server_addr.sin_port = htons(1000); //포트번호 1000 임의 지정
    server_addr.sin_addr.s_addr = inet_addr("127.0.0.1"); //서버 ip 지정

    //서버 접속. 서버의 주소정보로 클라이언트 소켓이 연결요청을 한다
    if(connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1){
        std::cout << "connet fail\n";
        exit(1); 
    }
    // int connect(int sock, struct sockaddr * servaddr, socklen_t addrlen);
    // 함수 성공 시 0, 실패 시 -1 반환. sock : 클라이언트의 소켓 전달. servaddr : 서버의 주소 정보가 담길 주소 값 전달. addrlen : 서버의 주소 정보의 크기를 담은 변수의 주소 값 전달

    char buffer[BUFFER_LEN];
    int n = read(client_socket, buffer, BUFFER_LEN);
    std::cout << n << " bytes read\n";
    buffer[n] = '\0';
    fputs(buffer, stdout);
    fflush(stdout);

    send(client_socket, &msg, sizeof(msg), MSG_DONTWAIT);
    

    //client 접속 종료
    close(client_socket);


    ros::spin();
    return 0;
}

