#ifndef MOTORDRIVE_H
#define MOTORDRIVE_H

#include "./Udp/DataProcess.h"
#include "Types.h"
#include <string>
#include <yaml-cpp/yaml.h>
#include <iomanip>
#include <sstream>

using namespace std;

typedef struct
{
   
} Device_Param;



class MotorDriver {

public:

    /**
     * @brief 初始化类
     * @param id  电机canId
     * @param MainEditionProcess 通信指针
     * @param canlineid  can线id
     */
    MotorDriver(uint16_t id,std::shared_ptr<DataProcess> MainEditionProcess,uint16_t canlineid=0);
    ~MotorDriver();


    /**
     * @brief 设置电机ID
     * @param NewId 新的电机ID值
     * @return true 设置成功 false 设置失败
     */

    bool SetMotorId(uint8_t NewId);

    /**
     * @brief 设置控制模式
     * @param Index 控制模式索引值 详见枚举 MotorCtrlMode_e
     * @return true 设置成功 false 设置失败
     */

    bool SetControlMode(uint32_t Index);

    /**
     * @brief 设置控制字
     * @param Word 控制字值 详见枚举 ControlWord_e
     * @return true 设置成功 false 设置失败
     */

    bool SetControlWord(uint32_t Word);

    /**
     * @brief 获取使能状态
     * @param Word 使能状态
     * @return true 设置成功 false 设置失败
     */

    bool GetControlWordEnable(uint32_t& Word);

    /**
     * @brief 设置快速模式参数
     * @param RepRate 周期毫秒
     * @return  true 设置成功 false 设置失败
     */ 

    bool SetFastMode(uint16_t RepRate);

    /**
     * @brief 设置位置控制参数
     *
     * @param Pos 位置设定值 单位:rad
     * @return void
     */

    void SetPos(float Pos);


     /**
     * @brief 设置大包位置、速度和电流控制参数
     *
     * @param Pos 位置设定值 单位:rad
     * @param Vel 速度设定值 单位:rad/s
     * @param Cur 电流设定值 单位:A
     * @return void
     */

    void SetBigparam(float Pos, float Vel, float Cur=0);


     /**
     * @brief 获取大包位置、速度和电流控制参数
     *
     * @param Pos 位置设定值 单位:rad
     * @param Vel 速度设定值 单位:rad/s
     * @param Cur 电流设定值 单位:A
     * @return void
     */

    void GetBigparam(float& Pos, float& Vel, float& Cur);

    /**
     * @brief 设置速度控制参数
     * @param Vel 速度设定值 单位:rad/s
     * @return void
     */

    void SetVel(float Vel);



    /**
     * @brief 设置电流控制参数
     * @param Cur 电流设定值 单位:A
     * @return void
     */

    void SetCur(float Cur);

    
    /**
     * @brief 设置速度环 PID控制参数
     * @param VelKp 速度环 比例系数P
     * @param VelKi 速度环 比例系数I
     * @param VelKd 速度环 积分系数D
     * @return true 成功，false 失败
     */

    bool SetVelPID(float VelKp, float VelKi, float VelKd);



      /**
     * @brief 设置速度环 PID控制参数
     * @param VelKp 速度环比例系数P
     * @return true 成功，false 失败
     */

    bool SetVelP(float VelKp);

      /**
     * @brief 获取速度环 PID控制参数
     * @param VelKp 速度环比例系数P
     * @return true 成功，false 失败
     */

    bool GetVelP(float& VelKp);

      /**
     * @brief 设置速度环 PID控制参数
     * @param VelKi 速度环比例系数I
     * @return true 成功，false 失败
     */

    bool SetVelI(float VelKi);




      /**
     * @brief 获取速度环 PID控制参数
     * @param VelKi 速度环比例系数I
     * @return true 成功，false 失败
     */

    bool GetVelI(float& VelKi);


     /**
     * @brief 设置速度环 PID控制参数
     * @param VelKd 速度环比例系数D
     * @return true 成功，false 失败
     */

    bool SetVelD(float VelKd);


      /**
     * @brief 获取速度环 PID控制参数
     * @param VelKd 速度环比例系数D
     * @return true 成功，false 失败
     */

    bool GetVelD(float& VelKd);



    /**
     * @brief 设置位置环 PID控制参数
     * @param PosKp 位置环比例系数P
     * @param PosKi 位置环比例系数I
     * @param PosKd 位置环积分系数D
     * @return true 成功，false 失败
     */

    bool SetPosPID(float PosKp, float PosKi, float PosKd);



     /**
     * @brief 设置位置环 PID控制参数
     * @param PosKp 位置环比例系数P
     * @return true 成功，false 失败
     */

    bool SetPosP(float PosKp);




     /**
     * @brief 获取位置环 PID控制参数P
     * @param PosKp 位置环比例系数P
     * @return true 成功，false 失败
     */

    bool GetPosP(float& PosKp);

    

     /**
     * @brief 设置位置环 PID控制参数
     * @param PosKi 位置环比例系数I
     * @return true 成功，false 失败
     */

    bool SetPosI(float PosKi);


      /**
     * @brief 获取位置环 PID控制参数I
     * @param PosKi 位置环比例系数I
     * @return true 成功，false 失败
     */

    bool GetPosI(float& PosKi);

     /**
     * @brief 设置位置环 PID控制参数
     * @param PosKd 位置环比例系数D
     * @return true 成功，false 失败
     */

    bool SetPosD(float PosKd);


       /**
     * @brief 获取位置环 PID控制参数D
     * @param PosKd 位置环比例系数D
     * @return true 成功，false 失败
     */

    bool GetPosD(float& PosKd);


    /**
     * @brief 设置位置PD环控制参数
     * @param PDKp 位置环比例系数
     * @param PDKd 位置环微分系数
     * @return true 成功，false失败
     */

    bool SetPD(float PDKp, float PDKd);



     /**
     * @brief 设置位置PD环控制参数
     * @param PDKp 位置环比例系数
     * @return true 成功，false失败
     */

    bool SetP(float PDKp);

    /**
     * @brief 获取位置PD环控制参数P
     * @param PDKp 位置环比例系数
     * @return true 成功，false失败
     */

    bool GetP(float& PDKp);


     /**
     * @brief 设置位置PD环控制参数
     * @param PDKd 位置环比例系数
     * @return true 成功，false失败
     */

    bool Setd(float PDKd);

     /**
     * @brief 获取位置PD环控制参数d
     * @param PDKd 位置环比例系数
     * @return true 成功，false失败
     */

    bool Getd(float& PDKd);



     /**
     * @brief 设置心跳关闭
     * @param  state 设置心跳周期
     * @return true 成功，false失败
     */

    bool SetHeartbeat(uint16_t state);

      /**
     * @brief 获取电机MOS温度
     * @param  Temperature Mos温度 
     * @return true 成功，false失败
     */

    bool GetMosTemperature(uint16_t& Temperature);

     /**
     * @brief 获取电机绕组温度
     * @param  Temperature 绕组温度
     * @return true 成功，false失败
     */

    bool GetCoilTemperature(uint16_t& Temperature);


    /**
     * @brief设置电机最大电流
     *
     * @param MaxCurrent 最大电流
     * @return 设置成功返回true，失败返回false
     */

    bool  SetMotorLimitMaxCurrent(float MaxCurrent);


     /**
     * @brief获取电机最大电流
     *
     * @param MaxCurrent 最大电流
     * @return 设置成功返回true，失败返回false
     */

    bool GetMotorLimitMaxCurrent(float& MaxCurrent);



     /**
     * @brief设置电机最大速度
     *
     * @param MaxSpeed 最大速度
     * @return 设置成功返回true，失败返回false
     */

    bool SetMotorLimitMaxSpeed(uint32_t MaxSpeed);



     /**
     * @brief获取电机最大转速
     *
     * @param MaxSpeed 最大转速
     * @return 设置成功返回true，失败返回false
     */

    bool GetMotorLimitMaxSpeed(uint32_t& MaxSpeed);

     /**
     * @brief设置电机最大加速度
     *
     * @param MaxAcc 最大加速度
     * @return 设置成功返回true，失败返回false
     */

    bool SetMotorLimitMaxAcc(uint32_t MaxAcc);

     /**
     * @brief获取电机最大加速度
     *
     * @param MaxAcc 最大加速度
     * @return 设置成功返回true，失败返回false
     */
    bool GetMotorLimitMaxAcc(uint32_t& MaxAcc);


      /**
     * @brief设置电机最小位置
     *
     * @param MinPosition 最小位置
     * @return 设置成功返回true，失败返回false
     */

    bool SetMotorLimitMinPosition(uint32_t MinPosition);

      /**
     * @brief获取电机最小位置
     *
     * @param MinPosition 最小位置
     * @return 设置成功返回true，失败返回false
     */

    bool GetMotorLimitMinPosition(uint32_t& MinPosition);


     /**
     * @brief设置电机最大位置
     *
     * @param MaxPosition 最大位置
     * @return 设置成功返回true，失败返回false
     */

    bool SetMotorLimitMaxPosition(uint32_t MaxPosition);



     /**
     * @brief获取电机最大位置
     *
     * @param MaxPosition 最大位置
     * @return 设置成功返回true，失败返回false
     */

    bool GetMotorLimitMaxPosition(uint32_t& MaxPosition);

     /**
     * @brief设置电机偏移角度
     *
     * @param angle 偏移角度
     * @return 设置成功返回true，失败返回false
     */
    bool SetMotorOffsetAngle(float angle);

     /**
     * @brief 保存电机设置到flash
     * @return 设置成功返回true，失败返回false
     */

    bool SetSysSaveToFlash();

      /**
     * @brief设置电机执行器参数
     *
     * @param type 执行器类别
     * @return 设置成功返回true，失败返回false
     */

    bool SetActuatorTypeParam(ActuatorType_e type);

       /**
     * @brief获取电机执行器参数
     *
     * @param type 执行器类别
     * @return 设置成功返回true，失败返回false
     */

    bool GetActuatorTypeParam(ActuatorType_e& type);


     /**
     * @brief设置电机 执行器基础配置
     *
     * @param param 执行器基础配置
     * @return 设置成功返回true，失败返回false
     */

    bool SetActuatorBsaeParam(ActuatorBsaeParam_t param);

       /**
     * @brief获取电机 执行器基础配置
     *
     * @param param 执行器基础配置
     * @return 设置成功返回true，失败返回false
     */

    bool GetActuatorBsaeParam(ActuatorBsaeParam_t& param);


    /**
     * @brief设置电机 执行器传感器参数
     *
     * @param param 执行器传感器参数
     * @return 设置成功返回true，失败返回false
     */

    bool SetActuatorSensorParam1(ActuatorSensorParam1_t param);

       /**
     * @brief获取电机  执行器传感器参数
     *
     * @param param  执行器传感器参数
     * @return 设置成功返回true，失败返回false
     */

    bool GetActuatorSensorParam1(ActuatorSensorParam1_t& param);


     /**
     * @brief设置电机 执行器传感器参数
     *
     * @param param 执行器传感器参数
     * @return 设置成功返回true，失败返回false
     */

    bool SetActuatorSensorParam2(ActuatorSensorParam2_t param);

       /**
     * @brief获取电机  执行器传感器参数
     *
     * @param param  执行器传感器参数
     * @return 设置成功返回true，失败返回false
     */

    bool GetActuatorSensorParam2(ActuatorSensorParam2_t& param);



    //  /**
    //  * @brief设置电机 执行器传感器参数
    //  *
    //  * @param param 执行器传感器参数
    //  * @return 设置成功返回true，失败返回false
    //  */

    // bool SetActuatorSensorParam2(ActuatorSensorParam2_t param);

    //    /**
    //  * @brief获取电机  执行器传感器参数
    //  *
    //  * @param param  执行器传感器参数
    //  * @return 设置成功返回true，失败返回false
    //  */

    // bool GetActuatorSensorParam2(ActuatorSensorParam2_t& param);


     /**
     * @brief设置电机 执行器控制参数 
     *
     * @param param 执行器控制参数 
     * @return 设置成功返回true，失败返回false
     */

    bool SetActuatorControlParams(ActuatorControlParam_t param);

       /**
     * @brief获取电机  执行器控制参数 
     *
     * @param param  执行器控制参数 
     * @return 设置成功返回true，失败返回false
     */

    bool GetActuatorControlParams(ActuatorControlParam_t& param);



     /**
     * @brief设置电机 执行器功能配置 
     *
     * @param param 执行器功能配置 
     * @return 设置成功返回true，失败返回false
     */

    bool SetActuatorFuncParam(ActuatorFuncParam_t param);

       /**
     * @brief获取电机  执行器功能配置 
     *
     * @param param  执行器功能配置 
     * @return 设置成功返回true，失败返回false
     */

    bool GetActuatorFuncParam(ActuatorFuncParam_t& param);

     /**
     * @brief设置电机 电机参数
     *
     * @param param 电机参数 
     * @return 设置成功返回true，失败返回false
     */

    bool SetMotorParam(MotorParam_t param);

       /**
     * @brief获取电机  电机参数
     *
     * @param param  电机参数
     * @return 设置成功返回true，失败返回false
     */

    bool GetMotorParam(MotorParam_t& param);

      /**
     * @brief设置电机 硬件参数
     *
     * @param param 硬件参数 
     * @return 设置成功返回true，失败返回false
     */

    bool SetHardwareParam(HardwareParam_t param);

       /**
     * @brief获取电机  硬件参数
     *
     * @param param  硬件参数
     * @return 设置成功返回true，失败返回false
     */

    bool GetHardwareParam(HardwareParam_t& param);

       /**
     * @brief设置电机 强拖控制参数
     *
     * @param param 强拖控制参数 
     * @return 设置成功返回true，失败返回false
     */

    bool SetForceParam(ForceParam_t param);

       /**
     * @brief获取电机  强拖控制参数
     *
     * @param param  强拖控制参数
     * @return 设置成功返回true，失败返回false
     */

    bool GetForceParam(ForceParam_t& param);

       /**
     * @brief设置电机 无感控制参数
     *
     * @param param 无感控制参数
     * @return 设置成功返回true，失败返回false
     */

    bool SetFulxObsParam(FulxObsParam_t param);

       /**
     * @brief获取电机  无感控制参数
     *
     * @param param  无感控制参数
     * @return 设置成功返回true，失败返回false
     */

    bool GetFulxObsParam(FulxObsParam_t& param);

       /**
     * @brief设置电机 电流力矩转换三次多项式系数
     *
     * @param param 电流力矩转换三次多项式系数
     * @return 设置成功返回true，失败返回false
     */

    bool SetTorqueCalibParam(TorqueCalibParam_t param);

       /**
     * @brief获取电机  电流力矩转换三次多项式系数
     *
     * @param param  电流力矩转换三次多项式系数
     * @return 设置成功返回true，失败返回false
     */

    bool GetTorqueCalibParam(TorqueCalibParam_t& param);

       /**
     * @brief设置电机 摩擦参数
     *
     * @param param 摩擦参数
     * @return 设置成功返回true，失败返回false
     */

    bool SetFrictionParam(FrictionParam_t param);

       /**
     * @brief获取电机  摩擦参数
     *
     * @param param  摩擦参数
     * @return 设置成功返回true，失败返回false
     */

    bool GetFrictionParam(FrictionParam_t& param);

    /**
     * @brief设置电机 摄氏度电流系数
     *
     * @param param 摄氏度电流系数
     * @return 设置成功返回true，失败返回false
     */

    bool SetTempCurParam(TempCurParam_t param);

       /**
     * @brief获取电机  摄氏度电流系数
     *
     * @param param  摄氏度电流系数
     * @return 设置成功返回true，失败返回false
     */

    bool GetTempCurParam(TempCurParam_t& param);


    /**
     * @brief设置电机 安全保护模块
     *
     * @param param 安全保护模块
     * @return 设置成功返回true，失败返回false
     */

    bool SetErrDectParam(ErrDectParam_t param);

       /**
     * @brief获取电机  安全保护模块
     *
     * @param param  安全保护模块
     * @return 设置成功返回true，失败返回false
     */

    bool GetErrDectParam(ErrDectParam_t& param);

      /**
     * @brief获取编码器的值  
     *
     * @param rawValue  安全保护模块
     * @return 设置成功返回true，失败返回false
     */

    bool GetRawValue(uint32_t& rawValue);

      /**
     * @brief设置开环强脱电压 
     *
     * @param voltage  安全保护模块
     * @return 设置成功返回true，失败返回false
     */

    bool SetVoltageValue(float voltage);



    // /**
    //  * @brief 快速获取位置、速度和估计转矩 此函数用于高速采样场景
    //  * 
    //  * @param Pos 存储位置值的引用  单位:rad
    //  * @param Vel 存储速度值的引用  单位:rad/s
    //  * @param Tor_e 存储估计转矩值的引用  单位: N.m
    //  */

    // void GetPVCTFast(float &Pos, float &Vel, float &Tor_e);

     /**
     * @brief 快速获取位置、速度和估计转矩 此函数用于高速采样场景
     * 
     * @param Pos 存储位置值的引用  单位:rad
     * @param Vel 存储速度值的引用  单位:rad/s
     * @param Cur 存储电流值的引用 
     * @param Tor_e 存储估计转矩值的引用  单位: N.m
     * @param FastStateMechine;  //stateMechine
     * @param FastBusVoltage;   //母线电压
     * @param FastErrorCode;   //错误码
     */

    void GetPVCTFast(float &Pos, float &Vel, float &Cur, float &Tor_e,
      uint32_t& FastStateMechine,uint32_t& FastBusVoltage, uint32_t& FastErrorCode);




    /**
   * @brief 升级前握手（擦falsh）
   * @return 主板准备是否成功 true成功，false 失败
   * @warning
   */
    bool StartOTAUpgrade();


    /**
    * @brief 升级固件
    * @param filename   升级文件名称
    * @return 返回升级是否成功 1为成功,
    */
    int OTAUpgradeing(const string filename);



    /**
    * @brief 查看固件版本
    * @param version   版本号
    * @return 返回升级是否成功
    */
    bool GetOTAUpgradeVersion( string& version);


   /**
   * @brief 一键固件升级
   * @param filename   升级文件名称
   * @param void (*func)(int,int)  回调函数，传递进度和错误码，1为成功，非1失败 -1: 握手失败，-2:发送开始标志失败,-3:传输文件失败,-4:发送停止标志失败
   * @return 返回升级是否成功 1为成功,
   */
    int OneClickOTAUpgradeing(const string filename,void (*func)(int,int));

       /**
      * @brief 发送开始升级指令
      * @param num   升级文件按32字节读取总组数量
      * @return 返回升级是否成功 true 成功，false失败
      */
    bool OTAUpgradeSign(const uint32_t  num);


      /**
      * @brief 发送数据
      * @param num   序号
      * @param data  有效数据
      * @return 返回升级是否成功 true成功，false失败
      */
    bool sendOTAUpgradeData(const uint32_t  num, const string data);


      /**
      * @brief 发送停止升级指令
      * @return 返回升级是否成功 true成功，false失败
      */
    bool StopOTAUpgradeSign();

    uint16_t Id;
    uint16_t canLineId;

  private:

      /**
     * @brief 读取文件
     * @param filename   升级文件名称
     * @return 以32字节为一组的字节列表
     */
    std::vector<ByteGroup> load_file(const std::string& path);
    std::string toHex(uint16_t val, bool upper = false);

    std::shared_ptr<DataProcess> Process;
    
    TotalParams TotalParamsData;
    FastParams FastParamsData;

    float m_pos;
    float m_vel;
    float m_cur;



};

#endif // MOTORDRIVE_H    