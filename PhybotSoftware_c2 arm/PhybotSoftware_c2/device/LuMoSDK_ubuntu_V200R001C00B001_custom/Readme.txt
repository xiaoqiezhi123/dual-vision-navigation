************************************************************************************
lusterinc，ubuntu_V200R001C00B001

Sample_C：C语言的demo示例及SDK
Sample_CPP：C++语言的demo示例及SDK

1.LuMoSDK文件夹中，include为SDK的需包含的头文件，lib中为SDK的动态库。其中libLuMoSDK_C.so为C接口类型的动态库，libLuMoSDK.so为C++接口的动态库
2.当前目录下的*_Sample.cpp是使用SDK的demo源码，CMakeLists.txt为工程配置，在Build文件夹下编译生成了demo可执行程序。

************************************************************************************
版本更新：2023年12月18日，lusterinc，ubuntu_V100R001C00B002
1、修复已知的编译问题；

************************************************************************************
版本更新：2024年5月28日，lusterinc，ubuntu_V100R001C00B003
1、c/c++版本LuMoSDK接收刚体数据新增欧拉角，角速度，角加速度；新增接收点集，测力台，时码信息
如有需要可以使用以下方法打印测力台数据：
// 测力台数据
            printf("ForceFlate Fx = %f.\n", MocapData.ForcePlateData.Fx); //打印测力台矢量力的分量：Fx
            printf("ForceFlate Fy = %f.\n", MocapData.ForcePlateData.Fy); //打印测力台矢量力的分量：Fy
            printf("ForceFlate Fz = %f.\n", MocapData.ForcePlateData.Fz); //打印测力台矢量力的分量：Fz
            printf("ForceFlate Mx = %f.\n", MocapData.ForcePlateData.Mx); //压心坐标：X
            printf("ForceFlate My = %f.\n", MocapData.ForcePlateData.My); //压心坐标：Y
            printf("ForceFlate Mz = %f.\n", MocapData.ForcePlateData.Mz); //压心坐标：Z
            printf("ForceFlate Lx = %f.\n", MocapData.ForcePlateData.Lx); //力矩
            printf("ForceFlate Lz = %f.\n", MocapData.ForcePlateData.Lz); //力矩


************************************************************************************
版本更新：2024年6月04日，lusterinc，ubuntu_V100R001C00B004
1、新增接收肌电数据
如有需要可以使用以下方法打印肌电数据：
for (int i = 0; i < MocapData->electromyographyData.nEmgDatas; i++)
{
       printf("ElectromyographyData EmgSN =  %s.\n", MocapData->electromyographyData.emgData[i].emgSn);
       printf("ElectromyographyData EmgData = %f.\n", MocapData->electromyographyData.emgData[i].emgData);
}

************************************************************************************
版本更新：2024年7月16日，lusterinc，ubuntu_V100R001C00B005
1、新增自定义骨骼信息接收

************************************************************************************
版本更新：2024年9月19日，lusterinc，ubuntu_V100R001C00B006
1、新增灵巧手信息接收

************************************************************************************
版本更新：2024年10月28日，lusterinc，ubuntu_V100R001C00B007
1、新增接收机器人电机角度数据

************************************************************************************
版本更新：2025年2月06日，lusterinc，ubuntu_V100R001C00B008
1、新增Ubuntu下Python版本LuMoSDK，使用方式详见./Sample_Python/README.txt

************************************************************************************
版本更新：2025年2月16日，lusterinc，ubuntu_V200R001C00B001
1、新增接收新测力台数据，删除接收原有旧测力台数据
注意：从此版本后LuMoSDK只能配合使用主线V200R006C003B003版本后的FZMotion以及生物力学项目的FZMotion

