
#ifndef DEVICESCAN_H
#define DEVICESCAN_H



#include <iostream>
#include "./Udp/DataProcess.h"

class DeviceScan
{
public:
    DeviceScan();
    ~DeviceScan();
   

    //发送广播数据
    void SendScan();
    //返回设备数据
    void GetDevices(vector<std::shared_ptr<DeviceInfo>>& deviceinfos);
    //初始化网络
    void init();
    //断开开连接
    void Disconnect();
    
    

private:
    
    std::shared_ptr<DataProcess> m_dataProcess;
    int port;



};


#endif