#ifndef TYPES_H
#define TYPES_H

#include <string>
#include <atomic>
#include <vector>
#include <list>

//添加电机参数
#include "UserParametersList.h"
//加载命令
#include "Common.h"

using namespace std;
#define SDOREQUEST 0x600
#define SDORECEIVE 0x580
#define HEARTBEAT  0x700
#define PDORECEIVE 0x180

#define MOTOROTA 0x81


typedef enum : uint8_t{

    ReadData = 0x03,  //读数据
    WriteData = 0x06  //写数据

}Red_Wrt;


//网络参数
typedef struct{

     uint32_t LocalIP;               //本地IP
    uint16_t LocalPort;              //本地端口
    uint32_t TargetIP;               //目标IP

} NetWorkParams;


//FastMode参数
typedef struct{

    std::atomic<int> FastPos_;     //位置
    std::atomic<short> FastVel_;   //速度
    std::atomic<short> FastCur_;  //电流
    std::atomic<short> FastTor_e_;  //力矩
    std::atomic<uint8_t> FastStateMechine;  //stateMechine
    // std::atomic<uint8_t> FastMosTemperature; //mos 温度
    // std::atomic<uint8_t> FastWindingTemperature; //绕组温度
    std::atomic<uint8_t> FastBusVoltage;   //母线电压
    std::atomic<uint32_t> FastErrorCode;   //错误码

}FastParams;

typedef struct
{
    float MAX_CURRENT;    //最大电流R/W
    float MAX_SPEED;      //最高转速R/W
    float MAX_ACC;        //最大加速度R/W
    float MIN_POSITION;   //最小位置限制R/W
    float MAX_POSITION;   //最大位置限制R/W

}MotorLimit;

typedef struct
{
    float PID_Speed_P;  //速度环P
    float PID_Speed_I;  //速度环I
    float PID_Speed_D;  // 速度环D
    float PID_Speed_OUTMAX; //速度环最大输出

}MotorVelPID;

typedef struct
{
    float PID_Pos_P;  //位置环P
    float PID_Pos_I;  //位置环I
    float PID_Pos_D;  // 位置环D
    float PID_Pos_OUTMAX; //位置环最大输出

}MotorPosPID;

typedef struct
{
    float PD_P;  //pd环P
    float PD_D;  //pd环D
    float PD_OUTMAX; //pd环最大输出

}MotorPD;
//创建电机对象参数
typedef struct{
    uint16_t canId;     //can  id
    uint16_t canLind;   //can线 id
}MotorCan;

typedef struct
{
   uint32_t sysModelType; //驱动器型号
   uint32_t polePairs; //极对数
   uint32_t reduRatio; //模块减速比
   uint32_t hardWareVersion; //硬件版本
   uint32_t softWareVersion; //软件版本
   uint32_t encoderRawValue; //编码器值
   uint32_t openloopVoltage; //开环强脱电压 
} DevInfo;


//电机所有参数
typedef struct{

    uint32_t ControlMode_;
    uint32_t WorkMode_;

    uint32_t Pos_;
    uint32_t Vel_;
    uint32_t Cur_;
    uint32_t Tor_l_;
    uint32_t Tor_e_;
    uint32_t Fault_;
    uint32_t FaultExt2_;
    uint32_t FaultExt3_;
    uint32_t FaultExt4_;
    uint32_t MosTemp_;
    uint32_t CoilTemp_;
    uint32_t VBusTemp_;
    uint32_t State; //状态版本号
    uint32_t Major; //主版本号
    uint32_t Minor; //次版本号
    uint32_t Patch; //修订版本号
    int32_t EncoderValue; //编码器值
    std::atomic<uint16_t> MotorId;   //电机id 
    std::atomic<uint16_t> canLineId; //can线id
    std::atomic<int> ACK_;
    std::atomic<int> CurrCmd;
    std::atomic<int> Serial_Number; //传输文件序号
    char model[20];  //电机型号
    MotorLimit motorLimit_;
    MotorVelPID motorVelPID_;
    MotorPosPID motorPosPID_;
    MotorPD motorPD_;
    std::list<MotorCan> MotorCanList;
    ParamList_t paramList;
    DevInfo devInfo;
} TotalParams;


//存储升级文件组
using ByteGroup = std::vector<std::uint8_t>;



#define Profile_Position_mode 0x01//轮毂位置模式（预留）
#define Profile_Velocity_mode 0x03//轮毂速度模式（预留）
#define Profile_Torque_mode 0x04//轮毂扭矩模式（预留）
#define Homing_mode 0x06//回零模式
#define Interpolated_Position_mode 0x07//位置插补模式（预留）
#define Cyclic_Synchronous_Position_mode 0x08//周期同步位置模式
#define Cyclic_Synchronous_Velocity_mode 0x09//周期同步速度模式
#define Cyclic_Synchronous_Torque_mode 0x0A//周期同步转矩模式
#define Cyclic_Synchronous_MIT_mode 0x0B//MIT






//版本号
//主版本.次版本.修订版本
//│      │      │        
//│      │      │        
//│      │      │        
//│      │     向下兼容的 Bug 修复
//│      └ 向下兼容的功能新增
//└不兼容的 API 变更
#define VERSION "2.3.1"

#endif // TYPES_H