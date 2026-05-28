# MotionSnap Fusion Core - 模块化传感器融合系统

## 概述

MotionSnap Fusion Core 是一个高度可扩展的模块化传感器融合系统，支持动态加载融合算法插件。

## 架构设计

### 核心组件

```
FusionCore/
├── fusion_core_api.h      # 融合核心API定义
├── fusion_core_manager.h  # 插件管理器头文件
├── fusion_core_manager.cpp# 插件管理器实现
└── plugins/               # 插件目录
    ├── BaseEKF/           # 基础EKF插件
    │   ├── plugin_base_ekf.h
    │   └── plugin_base_ekf.cpp
    └── RadarEKF/          # 毫米波雷达EKF插件
        ├── plugin_radar_ekf.h
        └── plugin_radar_ekf.cpp
```

### 设计原则

1. **模块化设计**：所有融合算法都作为独立插件实现
2. **可扩展性**：通过统一的接口支持添加新的融合算法
3. **UI可配置**：所有参数都可通过UI程序动态调整
4. **向后兼容**：保持现有数据结构不变

## 核心API

### FusionCoreManager

主要的插件管理类，负责：

- 加载和卸载DLL插件
- 管理已加载的插件列表
- 切换当前使用的融合算法
- 提供便捷的数据更新接口

### IFusionAlgorithm

所有融合插件必须实现的接口：

```cpp
class IFusionAlgorithm {
public:
    virtual bool Initialize() = 0;
    virtual void Shutdown() = 0;
    virtual void Reset() = 0;
    virtual void UpdateImuData(const FusionImuData& data) = 0;
    virtual void UpdateRadarData(const FusionRadarData& data) = 0;
    virtual bool GetFusionResult(FusionResult& result) = 0;
    virtual void GetPluginInfo(FusionPluginInfo& info) = 0;
    virtual uint32_t GetConfigParamCount() = 0;
    virtual bool GetConfigParam(uint32_t index, FusionConfigParam& param) = 0;
    virtual bool SetConfigParam(const char* param_name, const void* value) = 0;
    virtual bool GetConfigParam(const char* param_name, void* value) = 0;
};
```

## 数据结构

### FusionImuData

IMU数据结构（保持向后兼容）：
```cpp
struct FusionImuData {
    uint8_t node_id;
    uint8_t seq;
    uint64_t timestamp;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    float mag_x, mag_y, mag_z;
};
```

### FusionRadarData

雷达数据结构：
```cpp
struct FusionRadarData {
    uint8_t radar_type;        // 0: None, 1: LD2402, 2: Future
    uint8_t target_count;
    float distances[3];        // 最多3个目标距离（米）
    float velocities[3];       // 对应目标速度（米/秒）
    uint64_t timestamp;
};
```

### FusionResult

融合结果结构：
```cpp
struct FusionResult {
    FusionQuaternion orientation;
    float position_x, position_y, position_z;
    float velocity_x, velocity_y, velocity_z;
    uint64_t timestamp;
    uint32_t flags;
};
```

## 插件开发

### 创建新插件

1. 继承 `IFusionAlgorithm` 类
2. 实现所有纯虚函数
3. 使用 `FUSION_PLUGIN_EXPORT` 宏导出插件

### 示例：新插件模板

```cpp
#include "../FusionCore/fusion_core_api.h"

class MyNewFusion : public IFusionAlgorithm {
public:
    // 实现接口函数...
};

FUSION_PLUGIN_EXPORT(MyNewFusion)
```

## UI配置接口

### 插件信息获取

UI可以通过以下方式获取插件信息：

```cpp
// 获取插件数量
uint32_t count = FusionCore_GetPluginCount();

// 获取插件信息
FusionPluginInfo info;
FusionCore_GetPluginInfo(index, &info);
```

### 参数配置

每个插件都可以暴露可配置的参数：

```cpp
// 获取参数数量
uint32_t paramCount = algo->GetConfigParamCount();

// 获取参数信息
FusionConfigParam param;
algo->GetConfigParam(index, &param);

// 设置参数值
float value = 0.001f;
algo->SetConfigParam("qGyro", &value);
```

### 配置参数类型

- `PARAM_FLOAT`: 浮点数，可设置范围
- `PARAM_INT`: 整数
- `PARAM_BOOL`: 布尔值
- `PARAM_ENUM`: 枚举，通过字符串定义选项

## 使用示例

### C API 使用

```cpp
#include "FusionCore/fusion_core_api.h"

// 初始化
FusionCore_Init("./plugins");

// 选择算法
FusionCore_SelectAlgorithm("RadarEKF");

// 更新数据
FusionImuData imuData;
// ... 填充数据
FusionCore_UpdateImu(&imuData);

FusionRadarData radarData;
// ... 填充数据
FusionCore_UpdateRadar(&radarData);

// 获取结果
FusionResult result;
if (FusionCore_GetResult(&result)) {
    // 使用融合结果
}

// 清理
FusionCore_Shutdown();
```

### C++ API 使用

```cpp
#include "FusionCore/fusion_core_manager.h"

auto manager = FusionCoreManager::GetInstance();
manager->LoadPlugins("./plugins");
manager->SelectAlgorithm("RadarEKF");

auto algo = manager->GetCurrentAlgorithm();
algo->UpdateImuData(imuData);
algo->GetFusionResult(result);
```

## 内置插件

### BaseEKF

- **类型**: 基础IMU融合
- **功能**: 陀螺仪 + 加速度计 + 磁力计融合
- **状态数**: 10维（四元数 + 磁力计偏置 + 陀螺仪偏置）
- **特点**: 自适应噪声估计，磁力计有效性检测

### RadarEKF

- **类型**: IMU + 雷达融合
- **功能**: 增加位置和速度估计
- **状态数**: 16维（BaseEKF + 位置 + 速度）
- **特点**: 支持毫米波雷达数据融合，可动态调整雷达权重

## 改进建议

### 现有系统的问题

1. **缺少位置估计**：原有EKF只做姿态估计
2. **雷达数据未利用**：毫米波雷达数据没有参与融合
3. **参数调整不便**：算法参数不能通过UI实时调整
4. **扩展性差**：添加新的融合算法需要修改核心代码

### 模块化架构的改进

1. **支持位置/速度估计**：通过雷达数据增强
2. **即插即用**：新算法以DLL形式提供
3. **实时参数调整**：所有插件参数都可通过UI配置
4. **向后兼容**：保持现有数据结构不变

## 编译说明

### 依赖

- C++17 或更高
- Windows SDK
- CMake 3.10+

### 编译命令

```bash
cd FusionCore
mkdir build && cd build
cmake ..
cmake --build . --config Release
```

### 插件编译

每个插件需要独立编译为DLL：

```bash
cd FusionPlugins/BaseEKF
mkdir build && cd build
cmake ..
cmake --build . --config Release
```

## 目录结构

```
MotionSnapNodeMiddleware/
├── FusionCore/              # 融合核心
│   ├── fusion_core_api.h
│   ├── fusion_core_manager.h
│   ├── fusion_core_manager.cpp
│   └── CMakeLists.txt
├── FusionPlugins/           # 插件目录
│   ├── BaseEKF/
│   │   ├── plugin_base_ekf.h
│   │   ├── plugin_base_ekf.cpp
│   │   └── CMakeLists.txt
│   └── RadarEKF/
│       ├── plugin_radar_ekf.h
│       ├── plugin_radar_ekf.cpp
│       └── CMakeLists.txt
└── VRnode/                  # 原有模块保持不变
```
