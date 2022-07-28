/*
 * UPER Based client application sample code
 *
 * Copyright (c) 2022, CEST
 *
 */

#include <stdio.h>
#include "sample.h"

unsigned long long get_clock_time(){  
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);  
    uint64_t clock = ts.tv_sec * 1000 + (ts.tv_nsec / 1000000); 
    return clock;
} 

//---------------------------------------
void error_handling(const char *message)
{
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}

int connect_ros_dj(char* ip, unsigned short port)
{
    int serv_sock = -1;

    // 소켓 FD 생성
    serv_sock = socket(PF_INET, SOCK_STREAM, 0);

    if (serv_sock == -1)
    {
        error_handling("socket() error");
    }

    // 주소 정보 초기화
    struct sockaddr_in serv_addr, clnt_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(ip);
    serv_addr.sin_port = htons(port);
    

    // 소켓에 위에서 생성한 주소 정보를 할당
    if (bind(serv_sock, (struct sockaddr*) &serv_addr, sizeof(serv_addr)) == -1)
    {
        error_handling("bind() error");
    }

    // 클라이언트가 연결 요청이 가능하도록 5크기의 대기실 생성
    if (listen(serv_sock, 2) == -1)
    {
        error_handling("listen() error");
    }

    // accept() 함수 호출을 해서 실제 데이터를 보낼 수 있는 소켓을 생성
    socklen_t clnt_addr_size = sizeof(clnt_addr);
    int clnt_sock = accept(serv_sock, (struct sockaddr*)& clnt_addr, &clnt_addr_size);
    if (clnt_sock == -1)
    {
        error_handling("accept() error");
    }    

    // 성공적으로 서버에게서 메시지를 받아오면
    // 0을 반환하고 실패하면 -1을 반환한다.
    char message[30];
    int str_len;
    while(0)
    {
        str_len = read(clnt_sock, message, sizeof(message)-1);
        printf("Send message to server: %s \n", message);
    }

    // int flag = fcntl(clnt_sock, F_GETFL, 0);
    // fcntl(clnt_sock, F_SETFL, flag | O_NONBLOCK);

    printf("DEBUG : ROS TCP [%s:%d] Connected \n",ip,port); // TCP 접속 완료

    return clnt_sock;
}
//---------------------------------------

int connect_obu_uper_tcp(char*ip, unsigned short port)
{
    int sockFd = -1;
 
    // 소켓 파일디스크립터 생성
    sockFd = socket(PF_INET, SOCK_STREAM, 0);

    // 소켓 생성이 실패 여부 확인
    if (sockFd < 0)
    {
        printf("DEBUG : step sock create error\n");
        return -1;
    }

    // TCP 서버 주소, 포트 입력
    struct sockaddr_in addr; 
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(ip);
    addr.sin_port = htons(port);

    // TCP 서버 연결 
    if (connect(sockFd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        printf("DEBUG : step sock connect error\n");
        close(sockFd);
        return -1;
    }

    // 논 블록킹 소켓으로 동작
    int flag = fcntl(sockFd, F_GETFL, 0);
    fcntl(sockFd, F_SETFL, flag | O_NONBLOCK);

    printf("DEBUG : OBU TCP [%s:%d] Connected \n",ip,port); // TCP 접속 완료 

    return sockFd;
}


//---------------------------------------
PPVDMSG receive_from_ros(int clnt_sock)
{
    // 성공적으로 서버에게서 메시지를 받아오면
    // 0을 반환하고 실패하면 -1을 반환한다.
    PVDMSG msgs;
    memset(&msgs, 0, sizeof(msgs));

    char buf[24000];

    int str_len;
    str_len = read(clnt_sock, buf, sizeof(buf));
    printf("Received message from server size: %d \n", str_len);
    // printf("\n\n---Received buf from server: %s \n", buf);

    PPVDMSG recv_buf = (PPVDMSG) buf;

    return recv_buf;
}
//---------------------------------------


int receive_from_obu(int sockFd, char *buffer, unsigned short bufferSize, int storedSize,
                     char *uperBuffer, unsigned short uperBufferSize, int *uperRes)
{
    int rxSize = -1;

    *uperRes = 0;

    if(sockFd < 0)
        return rxSize;

    int leftBufferSize = bufferSize - storedSize;

    // printf("leftBufferSize: %d\n", leftBufferSize); 

    if(leftBufferSize <= 0)
    {
        printf("socket receive deadlock\n"); 
        return -1;
    }

    rxSize = read(sockFd, buffer + storedSize, leftBufferSize);

    //read 리턴값이 0 인 경우, Socket 로부터 Read가 불가능한 상태를 의미하며,
    //대표적으로 소켓 연결이 종료된 상태가 있음
    //리턴값이 -1 인 경우, non-blocking 소켓에 대해 수신된 데이터가 존재하지 않는 경우를 의미
    if (rxSize == 0) 
       return -1; 

    if(rxSize < 0 && storedSize == 0)
        return 0;
    
    if(rxSize > 0)
        storedSize += rxSize; 
    
    // CEST OBU TCP 헤더 파싱 부문
    int headerByteLen = sizeof(struct CestObuUperPacketHeader); 
    int packetLen = headerByteLen;
    if(storedSize < headerByteLen)
    {
        return storedSize;
    } 
    struct CestObuUperPacketHeader header;
    memcpy(&header,buffer,headerByteLen);
    
    packetLen += header.payloadLen;

    if (storedSize < packetLen)
    {
        return storedSize; 
    }

    memcpy(uperBuffer,buffer + headerByteLen,header.payloadLen);
    *uperRes = header.payloadLen;
    
    storedSize -= packetLen;
    memcpy(buffer,buffer+packetLen,storedSize);

    if (header.messageType == 0x3411)
    {
        struct TxWaveUperResultPayload *payload = (struct TxWaveUperResultPayload*)uperBuffer;
        // printf("RX - \"TX_WAVE_UPER_ACK\" [%d/%d/%d]\n",payload->txWaveUperSeq,payload->resultCode,payload->size);
        *uperRes = 0;
    }
    else 
    {
        // printf("RX - \"RX_WAVE_UPER\" [%d] \n\n",header.payloadLen); 

    }

    return storedSize;
}

int request_tx_wave_obu(int sockFd, char *uper,unsigned short uperLength){

    if(sockFd < 0)
        return -1;

    int packetLen = uperLength + sizeof(struct CestObuUperPacketHeader);

    char packet[OBU_RECEIVE_BUFFER_SIZE];  // tcp header size + uper binary size 

    struct CestObuUperPacketHeader *ptrHeader = (struct CestObuUperPacketHeader *)&packet[0];
    ptrHeader->messageType = 0x4311;
    ptrHeader->seq = packetSeq++;
    ptrHeader->payloadLen = uperLength;
    ptrHeader->deviceType = 0xCE;
    memcpy(ptrHeader->deviceId,clientDeviceId,3);

    memcpy(packet + sizeof(struct CestObuUperPacketHeader), uper, uperLength);
    
    int res = write(sockFd,packet,packetLen);

    if (res > 0)
    {
        printf("TX - \"TX_WAVE_UPER\" SEQ[%d] = ",ptrHeader->seq);
        print_hex(uper,uperLength);

        if (res != packetLen)
        {            
            printf("DEBUG tcp tx purge\n");
            return -1;
        }else
            return res;
    }
}
 
int tx_v2i_pvd(int sockFd, unsigned long long *time, PPVDMSG recv_buf)
{ 
    // printf("%s\n", "here");

    unsigned long long interval = get_clock_time() - *time;  // msec;

    if(interval < PVD_INTERVAL)
        return 0;
    
    *time += (interval - interval%PVD_INTERVAL); 
 
    MessageFrame_t msg;
    char uper[MAX_UPER_SIZE]; 

    fill_j2735_pvd(&msg, recv_buf);
   
    int encodedBits = encode_j2735_uper(uper,MAX_UPER_SIZE,&msg);
 
    if(encodedBits < 0) // 인코딩 실패로 전송이 불가능한 상태
        return 0;
     
    int byteLen = encodedBits / 8 + ((encodedBits % 8)? 1:0);
      
    //print_hex(uper,byteLen);    
       
    return request_tx_wave_obu(sockFd,uper,byteLen);
}
 
int tx_v2v_bsm(int sockFd, unsigned long long *time){

    unsigned long long interval = get_clock_time() - *time;
 
    if(interval < BSM_INTERVAL)
        return 0; 
    
    *time += (interval - interval%BSM_INTERVAL);

    MessageFrame_t *msg = malloc(sizeof(MessageFrame_t));
    char uper[MAX_UPER_SIZE]; 

    fill_j2735_bsm(msg);
       
    int encodedBits = encode_j2735_uper(uper,MAX_UPER_SIZE,msg);
    
    if (encodedBits < 0) 
        return 0;
  
    int byteLen = encodedBits / 8 + ((encodedBits % 8)? 1:0);

    //print_hex(uper, byteLen);
    
    return request_tx_wave_obu(sockFd, uper, byteLen);
} 

int main(void){ 
      
    int sockFd = -1;  // OBU와 연결하기 위한 TCP Client 소켓 File descriptor
    char rxBuffer[OBU_RECEIVE_BUFFER_SIZE] , rxUperBuffer[MAX_UPER_SIZE];
    int storedSize = 0, uperSize;
    unsigned long long txPvd = get_clock_time(), txBsm = get_clock_time();

    //---------------------------------------
    int ros_sockFd = -1; // ROS 와 연결하기 위한 TCP Client 소켓 File descriptor
    char ros_rxBuffer[OBU_RECEIVE_BUFFER_SIZE] , ros_rxUperBuffer[MAX_UPER_SIZE];
    int ros_storedSize = 0, ros_uperSize;
    unsigned long long ros_txPvd = get_clock_time(), ros_txBsm = get_clock_time();
    //---------------------------------------

    printf("Main\n");

    while(1)
    {    
        // 소켓이 연결되지 않은 경우(sockFd == -1) , OBU TCP 소켓 연결 시도
        if (sockFd < 0)
        {
            // 192.168.20.11
            // test server: 118.45.183.36
            sockFd = connect_obu_uper_tcp("118.45.183.36",23000); 
            storedSize = 0;
            if(sockFd < 0)
            {
                printf("DEBUG : connect failed, retry\n");
                sleep(1);
                continue;
            } 

            //---------------------------------------
            ros_sockFd = connect_ros_dj("192.168.0.21", 25000);
            ros_storedSize = 0;
            if(ros_sockFd < 0)
            {
                printf("DEBUG : ros connect failed, retry\n");
                sleep(1);
                continue;
            } 
            //---------------------------------------
        }
        //---------------------------------------
        // 차량 데이터 받아오는 부분
        PPVDMSG res = receive_from_ros(ros_sockFd);
        //---------------------------------------

        // 소켓이 연결된 상태인 경우, OBU로부터 TCP 패킷 수신
        // WAVE 통신으로 수신된 데이터가 없을 경우, 수신되는 데이터 X
        // tx_v2v_bsm(sockFd,&txBsm) < 0) 
        if (((storedSize = receive_from_obu(sockFd,
                                            rxBuffer, OBU_RECEIVE_BUFFER_SIZE, storedSize,
                                            rxUperBuffer, MAX_UPER_SIZE, &uperSize)) < 0) ||
            ((tx_v2i_pvd(sockFd, &txPvd, res) < 0)))
        {
            // OBU와 TCP 연결이 끊어진 경우, 연결 재시도
            close(sockFd);
            sockFd = -1;
            continue;
        }
        
        if (uperSize > 0)
        {
            // OBU로부터 수신된 WAVE 메시지가 존재할 경우, UPER 디코딩 -> J2735 메시지 파싱
            MessageFrame_t *msgFrame = NULL;
            decode_j2735_uper(msgFrame, rxUperBuffer, uperSize);

            ASN_STRUCT_FREE(asn_DEF_MessageFrame, msgFrame);
        }

        usleep(1000); //1msec sleep
    }

    close(sockFd);

    return 0;
}