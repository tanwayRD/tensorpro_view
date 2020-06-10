/************************************************
 *  Copyright (C) 2019 Tanway Technology
 *
 *  Created on: 16-07-2019
 *  Author: Elodie Shan
 *
 *  UDP interface for Tanway Tensor 3D LIDARs
**************************************************/
#include <ros/ros.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>  
#include <strings.h>

#ifndef NETWORK_H_
#define NETWORK_H_

class UDPNetwork
{
public:
  UDPNetwork();

  virtual ~UDPNetwork();

  socklen_t sockfd, ret, addrlen;
  struct sockaddr_in saddr,caddr;

  unsigned char filter_high[20]={0x00,0x01,0x02,0x03,0x04,0x05,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11};
  unsigned char filter_low[20]={0xDA,0xB4,0x8F,0x69,0x44,0x1E,0xF9,0xD3,0xAE,0x88,0x63,0x3D,0x18,0xF2,0xCD,0xA7,0x82,0x5C,0x37,0x11};

  char buf_[1500];

  std::string host;
  int port;
  std::string LiDARhost;
  int LiDARport;

  // @brief Initialize　some settings.
  bool Init(std::string host_, int port_, std::string LiDARhost_, int LiDARport_, bool DualEcho_switch);

  // @brief Verify that the connection  is successful.
  bool ConnectValid();

  // @brief Verify that the data source is correct.
  bool SourceValid();

    // @brief send UDP to LiDAR to set echo mode.
  bool setLiDARMode(bool DualEcho_switch, bool Filter_switch, int filter_thred);
  
  // @brief Get the UDP data.
  int recvUDP(char* buf);

 };


#endif
