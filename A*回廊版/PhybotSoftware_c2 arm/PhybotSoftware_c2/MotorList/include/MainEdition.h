#ifndef MAINEDITION_H
#define MAINEDITION_H
#include "./Udp/DataProcess.h"
#include "Types.h"
#include <list>
#include "MotorDrive.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include "DeviceScan.h"

using namespace std;



typedef struct
{
  uint16_t canId;
  uint16_t canline;
  float pos;
  float vel;
  float tor;

}PlanningPos;


class MainEdition
{
public:
   /**
     * @brief 初始化类
     * @param id  主板ID
     */
    MainEdition(uint16_t id);
    ~MainEdition();
    /**
     * @brief 配置主控板网络通信参数及建立连接
     * @param LocalIp 本地IP地址
     * @param LocalPort 本地端口号,默认15020
     * @param DestIp 设置目标IP地址，默认192.168.3.200
     * @return true 配置连接成功 false 配置连接失败
     * @warning 广播报文，必须用主板对应ID 
     */

    bool ConfigureNetworkParams(string LocalIp, uint16_t LocalPort = 15020,uint16_t DestPort=15000, string DestIp = "192.168.3.200");
    


    /**
     * @brief 修改主板ip
     * @param Destip 目标ip
     * @return true 配置连接成功 false 配置连接失败
     * @warning 修改ip后，需要重启主板
     */

    bool UpdateIp(string Destip);
     /**
     * @brief 创建列表电机对象
     * @param motorParamList 电机对象<canid,canlineid>列表
     * @param MotorList     接收返回的Motor对象列表
     * @return void
     */

    void CreateMotorDriverList(std::list<MotorCan>& motorParamList,std::list<std::shared_ptr<MotorDriver>>& MotorList);


   

      /**
     * @brief 创建单电机对象 c++接口
     * @param canId         canId
     * @param canLineId     can线Id
     * @param motorDrive    返回单个Motor对象
     * @return void
     */

    void CreateMotorDriver(uint16_t canId,uint16_t canLineId,std::shared_ptr<MotorDriver>& motorDrive);


    /**
    * @brief  将单电机对象存入主板电机对象列表中 c接口
    * @param  motorDrive    返回单个Motor对象
    * @return void
    */
    void CCreateMotorDriver(std::shared_ptr<MotorDriver>& motorDrive);
    /**
     * @brief 关闭socket任务
     * @return void
     */
    void CloseSocket();

   /**
   * @brief 从电机列表中删除单电机对象
   * @param motorDrive   单个Motor对象
   * @return void
   */
    void deleteMotorDriver(std::shared_ptr<MotorDriver>& motorDrive);
    
    /**
    * @brief 获取通信对象
    * @return 通信对象指针
    */
    std::shared_ptr<DataProcess> GetDataProcess();
    

    /**
     * @brief  根据电机列表组大包发送
     * @return void
     */

    void setPlanningPose();


    /**
    * @brief 获取在线电机ID
    * @return true 获取成功 false 获取失败
    */

    bool GetMotorOnline(list<MotorCan>& MotorList);



    /**
   * @brief 升级前握手（擦falsh）
   * @return 主板准备是否成功 true成功，false 失败
   * @warning
   */
    bool StartOTAUpgrade();


    /**
    * @brief 升级固件
    * @param filename   升级文件名称
    * @return 返回升级是否成功
    */
    int OTAUpgradeing(const string filename);



    /**
    * @brief 查看固件版本
    * @param version   版本号
    * @return 返回升级是否成功
    */
    bool GetOTAUpgradeVersion(string& version);

    /**
    * @brief 一键固件升级
    * @param filename   升级文件名称
    * @param void (*func)(int,int)  回调函数，传递进度和错误码，1为成功，非1失败 -1: 握手失败，-2:发送开始标志失败,-3:传输文件失败,-4:发送停止标志失败
    * @return 返回升级是否成功 1为成功,
    */
    int OneClickOTAUpgradeing(const string filename, void (*func)(int, int));
     
    
    /**
    * @brief 发送开始升级指令
    * @param num   升级文件按32字节读取总组数量
    * @return 返回升级是否成功 true 成功，false失败
    */
    bool OTAUpgradeSign(const uint32_t  num);



    /**
     * @brief  设置主板fast模式上传数据的频率
     * @param RepRate 频率
     * @return true 设置成功 false 设置失败
     * 
     */

    bool SetFastMode(uint32_t RepRate);


   /**
   * @brief 发送数据
   * @param num   序号 
   * @param data  有效数据 
   * @return 返回升级是否成功 true成功，false失败
   */
   bool sendOTAUpgradeData(const uint32_t  num,const string data);


   /**
   * @brief 发送停止升级指令
   * @return 返回升级是否成功 true成功，false失败
   */
   bool StopOTAUpgradeSign();


  /**
   * @brief 设置MIT状态机
   * @param state   参数
   */
   void StateMachine(uint16_t state);


   /**
   * @brief 获取主板连接状态
   * @param TechnologyStatistics   状态
   */
   void GetMainEditionStatus(unsigned int &TechnologyStatistics);

    /**
     * @brief 右夹手开关
     * @param State 开关状态 true开，false 关
     * @return true 设置成功 false 设置失败
     *
     */
   bool RightGrab(bool State);

     /**
     * @brief 左夹手开关
     * @param State 开关状态 true开，false 关
     * @return true 设置成功 false 设置失败
     *
     */
   bool LeftGrab(bool State);


     /**
     * @brief 左右夹手开关
     * @param State 开关状态 true开，false 关
     * @return true 设置成功 false 设置失败
     *
     */
   bool LeftAndRightGrab(bool State);



    /**
     * @brief 获取主板异常信息 1s调用一次
     * @param MainEdrionFaultErr 主板异常信息结构体
     * @return 
     *
     */

   bool GetErrorCode(MainEdrionFaultErr_t& MainEdrionFaultErr);


     /**
     * @brief 获取上报主板异常信息 
     * @param MainEdrionFaultErr 主板异常信息结构体
     * @return 
     *
     */

   void GetReportErrorCode(MainEdrionFaultErr_t& MainEdrionFaultErr);
   

    /**
     * @brief 获取BMS信息 
     * @param bmsBasicData BMS基础信息   
     * @return 
     *
     */
  //  bool GetBMSInfo(float& voltage, float&  current, float& soc, uint16_t& fault_code_0_1, uint16_t& fault_code_2_3,uint16_t& fault_code_4_5,
  //  uint16_t& fault_code_6_7,   uint16_t& fault_code_8_9, uint16_t& fault_code_10_11, uint16_t& fault_code_12_13);
   bool GetBMSInfo(BMSBasicData_t &bmsBasicData);



    /**
     * @brief 获取BMS信息 
     * @param bmsBasicData BMS基础信息
     * @return 
     *
     */
  //  bool GetReportBMSInfo(float& voltage, float&  current, float& soc, uint16_t& fault_code_0_1, uint16_t& fault_code_2_3,uint16_t& fault_code_4_5,
  //  uint16_t& fault_code_6_7,   uint16_t& fault_code_8_9, uint16_t& fault_code_10_11, uint16_t& fault_code_12_13);
   bool GetReportBMSInfo(BMSBasicData_t &bmsBasicData);



  
private:
    /**
    * @brief 读取文件
    * @param filename   升级文件名称
    * @return 以32字节为一组的字节列表
    */
    std::vector<ByteGroup> load_file(const std::string& path);

     /**
     * @brief 主板心跳处理函数
     * 
     */

    void MainEdtionHeartbeatHandle(std::string DestIp);

    /**
   * @brief 发送主板心跳
   */
    bool HeartbeatTransmission();

   

    std::thread MainEditionThread_;
    bool m_run;
    unsigned int m_TechnologyStatistics;  //记录异常数 
    
    std::shared_ptr<DataProcess> m_dataProcess;
    uint16_t Id;
    TotalParams TotalParamsData;
    FastParams FastParamsData;
    std::list<PlanningPos> m_panningPos;
    std::list<std::shared_ptr<MotorDriver>> m_MotorList;

};


#endif 