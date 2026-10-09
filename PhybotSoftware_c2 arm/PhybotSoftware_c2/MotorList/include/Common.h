#ifndef COMMON_H
#define COMMON_H
#include <stdint.h>
typedef enum {

    CTRL_NONE = 0,

    CTRL_SERVO_OFF = 1,    // 失能
    CTRL_SERVO_ON = 2,     // 使能
    CTRL_CLEAR_FAULT = 3,  // 清除错误

    RESET_DEFAULT = 4,  // 恢复默认参数

    // 传感器相关
    CTRL_CALIBRATE_ENCODER = 5,  // 编码器校准
    CTRL_RETURN_ZERO = 6 ,      // 执行器自动回零
    CTRL_POSITION_SET_ZERO = 7, // 执行器位置置零

    // 控制相关
    CTRL_FRICTION_IDENTIFY = 8,  // 摩擦校准
    CTRL_COGGING_IDENTIFY = 9,   // 齿槽转矩校准

    // 功能开关 0x3xxxF
    CTRL_FRICTION_COMP_OFF = 10,       // 失能摩擦补偿
    CTRL_FRICTION_COMP_ON = 11,        // 使能摩擦补偿
    CTRL_COGGING_COMP_OFF = 12,        // 失能齿槽转矩补偿
    CTRL_COGGING_COMP_ON = 13,         // 使能齿槽转矩补偿
    CTRL_TLOAD_COMP_OFF = 14,          // 失能负载补偿
    CTRL_TLOAD_COMP_ON = 15,           // 使能负载补偿
    CTRL_SOFT_POS_LIMIT_OFF = 16,      // 失能软限位
    CTRL_SOFT_POS_LIMIT_ON = 17,       // 使能软限位
    CTRL_OVER_TEMP_PROTECT_OFF = 18,   // 失能过温保护
    CTRL_OVER_TEMP_PROTECT_ON = 19,    // 使能过温保护
    CTRL_HEART_BEAT_PROTECT_OFF = 20,  // 失能心跳保护
    CTRL_HEART_BEAT_PROTECT_ON = 21,   // 使能心跳保护

} ControlWord_e;

// 电机控制模式
typedef enum {

    MOTOR_CTRL_MODE_NONE = 0,         // 无控制模式
    MOTOR_CTRL_MODE_CURRENT = 1,      // 电流控制模式
    MOTOR_CTRL_MODE_VELOCITY = 2,     // 速度控制模式
    MOTOR_CTRL_MODE_POSITION = 3,     // 位置控制模式
    MOTOR_CTRL_MODE_PD = 4,        // 转矩位置控制模式
    MOTOR_CTRL_MODE_BRAKE = 5,        // 制动模式
    MODE_LADRC_VELOCITY = 6,            // 速度控制模式
    MODE_LADRC_POSITION = 7,            // 位置控制模式
    MOTOR_CTRL_MODE_OPENLOOP = 0xff,  // 开环控制模式

} MotorCtrlMode_e;

// // 电机状态码
// typedef enum {

//     IDLE_WM = 0,                  // 空闲状态
//     Init_WM = 1,                  // 初始化状态
//     Normal_WM = 2,                // 正常控制状态
//     Fault_WM = 3,                 // 异常状态
//     ENCODER_CAIL_WM = 4,          // 特殊状态 电机编码器校准
//     LINER_HALL_CAIL_WM = 5,       // 特殊状态 电机编码器校准
//     OUTPUT_ENCODER_CAIL_WM = 6,   // 特殊状态 出轴编码器校准
//     ACTUATOR_RETURN_ZERO_WM = 7,  // 特殊状态 执行器回零模式
//     FRICTION_IDENTIFY_WM = 8,     // 特殊状态 摩擦辨识模式

// } UserWorkMode_e;

//通信控制指令
typedef enum {
    NMT = 0x0100,                 //NMT状态机开启
    ConfigNetWork = 0x600,        //网络通信参数设置
    Get_MotorID = 0x620,        // 获取电机
    Heartbeat = 0x1017,           //设置电机心跳
    MainEditionHeartbeat = 0x1018,           //设置主板心跳
    GitBmsData = 0x1019,             //BMS数据
    OTA_CMD_HandShanking = 1563u,      // OTA 握手
    OTA_CMD_UpDateStart,       // OTA 开始下载
    OTA_CMD_FileTransmission,  // OTA 文件传输
    OTA_CMD_UpDateEnd,         // OTA 下载结束
    OTA_CMD_SendVersion,       // OTA 发送版本号
    FastMode = 0x1800,            //快速上报模式
    ConfigIp = 0x1801,            //修改ip
    MainEditionError = 0x1802,         //主板错误命令
    Gripper_Command = 0x1803,	  //设置夹手
    SetId = 0x2001,               //设置电机ID
    DrvInfo = 0X2002,          //驱动器信息结构
    MotorTemperature = 0X2006,      //电机MOS温度/绕组温度
    SaveParamToFlash = 0x200A,     //保存参数到flash
    GetENABLE_DRIVER = 0x200B,     //获取使能状态
    ControlSetLimmit = 0x2011,     //设置电机限制
    ControlSetPosPID = 0x2020,    //指令位置环PID
    ControlSetVelPID = 0x2021,    //指令速度环PID
    ControlSetPD = 0x2022,        //指令PD
    ControlMode = 0x6060,         //控制模式
    ControlWord = 0x6040,         //控制字            
    ControlSetPos = 0x607A,       //指令位置
    ControlSetVel = 0x60FF,       //指令速度
    ControlSetTor =0x6071,        //指令力矩

}Command_Types;

// 设备信息结构体
typedef struct{

    char MainEditionIp[16]; //主机Ip
    uint16_t MainEditionPort; //主机端口
    uint32_t MainID; //主机ID
} DeviceInfo;


//主板错误信息结构体
typedef struct MainEdrionFaultErr{
		/* CAN communication errors */
		uint8_t MainOverheat;    // Mosfet温度        
		uint8_t motor_bus_voltage;    // 关节母线输出电压错误
	    uint8_t motor_bus_current;    // 关节母线电流错误
		uint8_t CanIdMismatch;    // CANID丢失
        uint16_t MisMatchId[16];
	
}MainEdrionFaultErr_t;

/* BMS 基本数据结构体 */ 
typedef struct {
    float voltage;           // 电池总电压 (V)，分辨率 0.1V，范围 0-35.0V
    float current;           // 电流数据 (A)，分辨率 0.1A，Offset=30000，充电为负，放电为正
    float soc;               // SOC (%)，分辨率 0.001%，范围 0-100%
    
    // 故障码原始数据
    uint16_t fault_code_0_1;     // 新故障码0-1 (寄存器地址: 0x6D)
    uint16_t fault_code_2_3;     // 新故障码2-3 (寄存器地址: 0x6E)
    uint16_t fault_code_4_5;     // 新故障码4-5 (寄存器地址: 0x6F)
    uint16_t fault_code_6_7;     // 新故障码6-7 (寄存器地址: 0x70)
    uint16_t fault_code_8_9;     // 新故障码8-9 (寄存器地址: 0x71)
    uint16_t fault_code_10_11;   // 新故障码10-11 (寄存器地址: 0x72)
    uint16_t fault_code_12_13;   // 新故障码12-13 (寄存器地址: 0x73)
} BMSBasicData_t;


#endif