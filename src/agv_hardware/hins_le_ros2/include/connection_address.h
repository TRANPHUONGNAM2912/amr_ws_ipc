/*********************************************************************
*
* Software License Agreement ()
*
*  Copyright (c) 2020, HINS, Inc.
*  All rights reserved.
*
* Author: Hu Liankui
* Create Date: 8/9/2020
*********************************************************************/

/**
 * @brief tcp连接地址描述
 */

#pragma once

#include <string>

using namespace std;

namespace hins {

class ConnectionAddress final
{
public:
  ConnectionAddress(){;}
  ConnectionAddress(const string& address, const int& port)
    : address_(address)
    , port_(port)
    , description_("Connection address and port")
    , use_udp_(false)
  {}
  ConnectionAddress(const string& address, const int& port, const string& description)
    : address_(address)
    , port_(port)
    , description_(description)
    , use_udp_(false)
  {}

  void SetAddress(const string& address)
  {
    address_ = address;
  }

  void SetPort(const int& port)
  {
    port_ = port;
  }

  void SetDescription(const string& description)
  {
    description_ = description;
  }

  const string GetAddress() const
  {
    return address_;
  }

  int GetPort() const
  {
    return port_;
  }
  void UseUdp()
  {
    use_udp_ = true;
  }

  void UseTcp()
  {
    use_udp_ = false;
  }
  bool GetUseUdp() const
  {
    return use_udp_;
  }
  const string GetDescription() const
  {
    return description_;
  }

private:
  string address_;
  int port_;
  string description_;
  bool use_udp_ = false;
};

}
