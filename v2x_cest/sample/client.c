// C++ TCP 클라이언트 프로그램
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

void error_handling(const char *message)
{
  fputs(message, stderr);
  fputc('\n', stderr);
  exit(1);
}
int main(int argc, char* argv[])
{
  int sock = socket(PF_INET, SOCK_STREAM, 0);
  if(sock == -1)
  {
    error_handling("socket() error");
  }

  // 클라이언트와 마찬가지로 주소정보를 초기화
  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family=AF_INET;
  serv_addr.sin_addr.s_addr=inet_addr("192.168.0.18");
  serv_addr.sin_port=htons(atoi("24000"));
  
  // 서버의 주소정보로 클라이언트 소켓이 연결요청을 한다.
  if(connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr))==-1) 
  {
    error_handling("connect() error!");
  }

  // Send a message
  char msg[] = "Hello world!";
  while (1)
  {
    write(sock, msg, sizeof(msg));
    printf("Send message to server: %s \n", msg);
    sleep(1);
  }

  close(sock);
  return 0;
}
