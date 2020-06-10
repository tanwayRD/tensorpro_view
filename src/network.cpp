/************************************************
 *  Copyright (C) 2019 Tanway Technology
 *
 *  Created on: 16-07-2019
 *  Edited on: 25-03-2020
 *  Author: Elodie Shan
 *  Editer: Elodie Shan
 *
 *  UDP interface for Tanway Tensor 3D LIDARs
**************************************************/

#include "tensorpro_view/network.h"
#include <iostream> 
#include <stdlib.h>
#include <stdio.h>
using namespace std;
UDPNetwork::UDPNetwork(){};

UDPNetwork::~UDPNetwork(){};

bool UDPNetwork::Init(std::string host_, int port_, std::string LiDARhost_, int LiDARport_, bool DualEcho_switch)
{
  host = host_;
  port = port_;
  LiDARhost = LiDARhost_;
  LiDARport = LiDARport_;

  sockfd = socket(AF_INET,SOCK_DGRAM,0);
  saddr.sin_family = AF_INET; //IPv4
  saddr.sin_port = htons(port);
  saddr.sin_addr.s_addr = inet_addr(host.data());//Convert between the binary IP address of the 32-bit network byte and the dotted decimal IP address

  ret = bind(sockfd,(struct sockaddr*)&saddr,sizeof(saddr)); //Assign a local name to an unnamed socket to establish a local bundle (host address/port number) for the socket.

  if(ret < 0) 
  {
    perror("bind fail!");
    return false;
  }

  bool status = ConnectValid() && SourceValid() && setLiDARMode(DualEcho_switch,false,0) && ConnectValid();

  if (status)
    ROS_INFO("Connect with LiDAR !");

  return status;

}

bool UDPNetwork::ConnectValid()
{
  int RECV_TIMEOUT_COUNT = 0;
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(sockfd,&readfds);
  struct timeval RECV_TIMEOUT;
  RECV_TIMEOUT.tv_sec = 10;

  while(select(sockfd+1,&readfds,NULL,NULL,&RECV_TIMEOUT)==0)//Listening to the state of the socket
  {
    if (RECV_TIMEOUT_COUNT == 5)
      return false;
    ROS_WARN("Failed to connect with LiDAR , time out!");
    sleep(1);
    RECV_TIMEOUT_COUNT++;
    FD_ZERO(&readfds);
    FD_SET(sockfd,&readfds);
    RECV_TIMEOUT.tv_sec = 30;
  } 

  return true;
}

bool UDPNetwork::SourceValid()
{
  // Check LiDAR data source
  bzero(buf_,sizeof(buf_));
  addrlen = sizeof(caddr);

  ret = recvfrom(sockfd,buf_,1500,0,(struct sockaddr*)&caddr,&addrlen); //Recieve UDP packet to update client address.

  if (inet_ntoa(caddr.sin_addr)!= LiDARhost || htons(caddr.sin_port)!= LiDARport)
  {
    ROS_ERROR("Warn data source %s: %d.  The valid host is %s: %d ",(char *)inet_ntoa(caddr.sin_addr),htons(caddr.sin_port),LiDARhost.data(),LiDARport);
    ROS_ERROR("Please check the LiDAR or the config file!");
    return false; 
  }
  return true;
}

bool UDPNetwork::setLiDARMode(bool DualEcho_switch, bool Filter_switch, int filter_thred)
{
  std::string EchoMode = "Single";
  // Single echo mode and single mirror mode by default
  unsigned char send_buf[16]={0x01,0xFE,0xFE,0xFE,0xC1,0xC1,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xD1,0xD1,0xFE,0xFE};

  if (DualEcho_switch) //Single Echo Mode
  {
    send_buf[0] = 0x02;
    EchoMode = "Dual";
    
    if (!Filter_switch)
    {
      send_buf[6] == 0xFF;
      send_buf[7] == 0xFF;
    }
    else
    {
      if (filter_thred < 20)
      {
        send_buf[6] == filter_high[filter_thred-1];
        send_buf[7] == filter_low[filter_thred-1];
      }
      else
      {
        send_buf[6] == filter_high[19];
        send_buf[7] == filter_low[19];
      }
    }

  }
  ret = sendto(sockfd, (const char*)send_buf, 16, 0, (struct sockaddr *)&caddr, sizeof(caddr));

  if(ret < 0)  
  {  
    ROS_ERROR("Failed to set Lidar's work mode!");
    return false;
  }
  
  ROS_INFO("Set LiDAR(%s:%d) in %s Echo Mode!",(char *)inet_ntoa(caddr.sin_addr),htons(caddr.sin_port),EchoMode.data());
  return true;
}

int UDPNetwork::recvUDP(char* buf)
{
  bzero(buf,sizeof(buf));
  addrlen = sizeof(caddr);

  ret = recvfrom(sockfd,buf,1500,0,(struct sockaddr*)&caddr,&addrlen); //Recieve UDP packets
  
  return ret;
}

