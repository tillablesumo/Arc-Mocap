#include "../FusionCore/fusion_core_api.h"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    std::cout << "MotionSnap Fusion Core Example" << std::endl;
    std::cout << "==============================" << std::endl;

    if (!FusionCore_Init("./plugins")) {
        std::cout << "Failed to initialize FusionCore" << std::endl;
        return -1;
    }

    FusionCore_SetWeights(0.8f, 0.2f);
    FusionCore_SetDataSource(true, true);
    FusionCore_SetOutputRate(100.0f);

    uint32_t pluginCount = FusionCore_GetPluginCount();
    std::cout << "Loaded " << pluginCount << " plugins" << std::endl;

    for (uint32_t i = 0; i < pluginCount; ++i) {
        FusionPluginInfo info;
        if (FusionCore_GetPluginInfo(i, &info)) {
            std::cout << "  Plugin " << i << ": " << info.plugin_name 
                      << " v" << info.plugin_version << std::endl;
        }
    }

    FusionImuData imuData = {};
    FusionRadarData radarData = {};
    FusionResult result = {};

    for (int i = 0; i < 100; ++i) {
        imuData.timestamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        imuData.gyro[0] = 0.1f;
        imuData.gyro[1] = 0.05f;
        imuData.gyro[2] = 0.02f;
        imuData.accel[0] = 0.0f;
        imuData.accel[1] = 0.0f;
        imuData.accel[2] = 9.81f;
        imuData.mag[0] = 0.4f;
        imuData.mag[1] = 0.0f;
        imuData.mag[2] = 0.4f;

        radarData.timestamp = imuData.timestamp;
        radarData.distance = 2.5f + (i * 0.01f);
        radarData.velocity = 0.1f;

        FusionCore_UpdateImu(&imuData);
        FusionCore_UpdateRadar(&radarData);

        if (FusionCore_HasNewData()) {
            if (FusionCore_GetResult(&result)) {
                std::cout << "\rSample " << i << ": Position=" << result.position[0] 
                          << "m, Velocity=" << result.velocity[0] << "m/s" << std::flush;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << std::endl;
    std::cout << "Shutting down..." << std::endl;
    FusionCore_Shutdown();

    return 0;
}
