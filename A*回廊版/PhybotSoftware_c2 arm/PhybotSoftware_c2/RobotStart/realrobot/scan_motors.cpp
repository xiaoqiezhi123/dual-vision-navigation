#include <iostream>
#include <list>
#include <memory>
#include "MotorList.hpp"

int main() {
    DataPackage package;
    package.init();

    MotorList motorlist;
    std::string config_yaml1 = "../MotorList/config/phybot_mini_1.yaml";
    std::string config_yaml2 = "../MotorList/config/phybot_mini_2.yaml";

    std::cout << "========================================" << std::endl;
    std::cout << "  Initializing motor networks..." << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    motorlist.Init(config_yaml1, config_yaml2, package);

    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Scanning for online motors..." << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    // -------------------------------------------------------
    // Scan MainBoard 1 (192.168.3.10)
    // -------------------------------------------------------
    std::cout << "--- MainBoard 1 (192.168.3.10) ---" << std::endl;
    auto net1 = motorlist.MotorControl->GetMotorNet1();
    if (net1) {
        std::list<MotorCan> motorList1;
        std::cout << "  Sending scan request (timeout: 5s)..." << std::endl;
        if (net1->GetMotorOnline(motorList1)) {
            if (motorList1.empty()) {
                std::cout << "  [OK] No motors reported online." << std::endl;
            } else {
                std::cout << "  Found " << motorList1.size() << " motor(s):" << std::endl;
                for (const auto& m : motorList1) {
                    std::cout << "    [CAN ID: " << m.canId
                              << "]  [CAN Line: " << (int)m.canLind << "]"
                              << std::endl;
                }
            }
        } else {
            std::cout << "  [FAIL] Scan request timed out or failed." << std::endl;
        }
    } else {
        std::cout << "  [SKIP] Network 1 not initialized." << std::endl;
    }
    std::cout << std::endl;

    // -------------------------------------------------------
    // Scan MainBoard 2 (192.168.3.11)
    // -------------------------------------------------------
    std::cout << "--- MainBoard 2 (192.168.3.11) ---" << std::endl;
    auto net2 = motorlist.MotorControl->GetMotorNet2();
    if (net2) {
        std::list<MotorCan> motorList2;
        std::cout << "  Sending scan request (timeout: 5s)..." << std::endl;
        if (net2->GetMotorOnline(motorList2)) {
            if (motorList2.empty()) {
                std::cout << "  [OK] No motors reported online." << std::endl;
            } else {
                std::cout << "  Found " << motorList2.size() << " motor(s):" << std::endl;
                for (const auto& m : motorList2) {
                    std::cout << "    [CAN ID: " << m.canId
                              << "]  [CAN Line: " << (int)m.canLind << "]"
                              << std::endl;
                }
            }
        } else {
            std::cout << "  [FAIL] Scan request timed out or failed." << std::endl;
        }
    } else {
        std::cout << "  [SKIP] Network 2 not initialized." << std::endl;
    }
    std::cout << std::endl;

    // -------------------------------------------------------
    // Summary: expected vs actual
    // -------------------------------------------------------
    std::cout << "================ Summary ================" << std::endl;
    std::cout << std::endl;
    std::cout << "Motors expected from YAML configs:" << std::endl;
    for (const auto& [name, motor] : *motorlist.Motors_Map) {
        std::cout << "  " << name
                  << "  (ID: " << motor->Id
                  << ", CAN Line: " << motor->canLineId << ")"
                  << std::endl;
    }
    std::cout << std::endl;
    std::cout << "If a motor is expected but missing from the scan," << std::endl;
    std::cout << "check: wiring, power, CAN ID mismatch, or hardware fault." << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
