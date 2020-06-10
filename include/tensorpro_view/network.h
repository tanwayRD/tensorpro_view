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

  char buf_[1500];

  std::string host;
  int port;
  std::string LiDARhost;
  int LiDARport;

  // @brief Initialize　some settings.
  bool Init(std::string host_, int port_, std::string LiDARhost_, int LiDARport_);

  // @brief Verify that the connection  is successful.
  bool ConnectValid();

  // @brief Verify that the data source is correct.
  bool SourceValid();

  // @brief Get the UDP data.
  int recvUDP(char* buf);

 };


#endif
