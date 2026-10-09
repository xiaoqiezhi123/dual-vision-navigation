#ifndef MOTORSCONTROL_H
#define MOTORSCONTROL_H

#include <string>
#include <map>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "MotorDrive.h"
#include "MainEdition.h"

using namespace std;

typedef struct{

    uint16_t MainBoard; 
    uint16_t Id;    
    uint16_t CanId;
    string Name;
    uint32_t ControlMode;
    uint32_t FastMode;
    float PosKp;
    float VelKp;
    float VelKi;
    float PDKp;
    float PDKd;
    float Direction;

}MotorsParam;

class MotorsControl {

public:

    MotorsControl();
    ~MotorsControl();

    /**
     * @brief 小型机器人从YAML配置文件加载参数
     *
     * @param Path 配置文件路径
     * @return true 加载成功，false 加载失败
     * @note 支持相对路径和绝对路径，推荐使用绝对路径
     * @attention 文件格式必须符合MotorConfig格式规范
     */

    bool LoadYAMLConfigFromFile(string Path1, string Path2);


     /**
     * @brief 大型机器人从YAML配置文件加载参数
     *
     * @param Path 配置文件路径
     * @return true 加载成功，false 加载失败
     * @note 支持相对路径和绝对路径，推荐使用绝对路径
     * @attention 文件格式必须符合MotorConfig格式规范
     */

    bool LoadYAMLConfigFromFile(string Path1, string Path2,string Path3);

    // void CloseMainBoardUdp();

    /**
     * @brief 根据名称查找电机驱动句柄
     *
     * @param Name 电机名称
     * @return std::shared_ptr<MotorDriver> 找到的电机驱动智能指针。若未找到，返回空指针
     * @attention 注意判断返回是否为空指针
     */

    std::shared_ptr<MotorDriver> FindMotorHandleByName(string Name);

    /**
     * @brief 根据ID查找电机驱动句柄
     *
     * @param Name 电机ID
     * @return std::shared_ptr<MotorDriver> 找到的电机驱动智能指针。若未找到，返回空指针
     * @attention 注意判断返回是否为空指针
     */

    std::shared_ptr<MotorDriver> FindMotorHandleById(uint16_t Id);

        /**
     * @brief 获取全部正常通信的电机的map指针
     *
   
     * @return map<string, std::shared_ptr<MotorDriver>> 全部正常通信的电机的map容器指针
     */

    map<string, std::shared_ptr<MotorDriver>> * GetMotorsMap();


    Eigen::VectorXd GetAllPDKpVector() const;
    Eigen::VectorXd GetAllPDKdVector() const;
    Eigen::VectorXd GetAllDirectionVector() const;

    std::shared_ptr<MainEdition> GetMotorNet1() const { return Motor_Net1; }
    std::shared_ptr<MainEdition> GetMotorNet2() const { return Motor_Net2; }
    std::shared_ptr<MainEdition> GetMotorNet3() const { return Motor_Net3; }

     /**
     * @brief 获取主板1异常信息
     * @param MainOverheat;    // Mosfet温度 
     * @param motor_bus_voltage;    // 电机母线电压错误
     * @param motor_bus_current;    // 电机母线电流错误
     * @param CanIdMismatch;    // CANID丢失 数量
     * @return 
     */
    
    void GetMainEdition1Error(MainEdrionFaultErr_t& MainEdrionFaultErr);
    

     /**
     * @brief 获取主板2异常信息
     * @param MainOverheat;    // Mosfet温度 
     * @param motor_bus_voltage;    // 电机母线电压错误
     * @param motor_bus_current;    // 电机母线电流错误
     * @param CanIdMismatch;    // CANID丢失    数量
     * @return 
     */
    void GetMainEdition2Error(MainEdrionFaultErr_t& MainEdrionFaultErr);
    


     /**
     * @brief 左夹手开关
     * @param State 开关状态 true开，false 关
     * @return true 设置成功 false 设置失败
     */

    bool  setLeftGrab(bool state);

     /**
     * @brief 右夹手开关
     * @param State 开关状态 true开，false 关
     * @return true 设置成功 false 设置失败
     */

    bool  setRightGrab(bool state);

     /**
     * @brief 左右夹手开关
     * @param State 开关状态 true开，false 关
     * @return true 设置成功 false 设置失败
     */

    bool  setLeftAndRightGrab(bool state);



private:
    std::shared_ptr<MainEdition> Motor_Net1;
    std::shared_ptr<MainEdition> Motor_Net2;
    std::shared_ptr<MainEdition> Motor_Net3;
    map<string, std::shared_ptr<MotorDriver>> MotorsNameMap;
    map<uint16_t, std::shared_ptr<MotorDriver>> MotorsIdMap;

    std::vector<float> PDKp_values_;
    std::vector<float> PDKd_values_;
    std::vector<float> direction_values_;
    std::vector<std::shared_ptr<MainEdition>> MainEditions_;
};

#endif // MOTORSCONTROL_H    