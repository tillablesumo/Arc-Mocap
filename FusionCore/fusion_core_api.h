#pragma once

#ifdef _WIN32
    #ifdef FUSION_CORE_EXPORTS
        #define FUSION_CORE_API __declspec(dllexport)
    #else
        #define FUSION_CORE_API __declspec(dllimport)
    #endif
    #define FUSION_PLUGIN_API __declspec(dllexport)
#else
    #define FUSION_CORE_API
    #define FUSION_PLUGIN_API
#endif

#include <cstdint>
#include <cstring>

struct FusionImuData {
    uint64_t timestamp;
    float gyro[3];
    float accel[3];
    float mag[3];
};

struct FusionRadarData {
    uint64_t timestamp;
    float distance;
    float velocity;
    float angle;
};

struct FusionResult {
    uint64_t timestamp;
    bool valid;
    float orientation[4];
    float angular_velocity[3];
    float linear_acceleration[3];
    float magnetic_field[3];
    float position[3];
    float velocity[3];
};

struct FusionPluginInfo {
    const char* plugin_name;
    const char* plugin_version;
    const char* plugin_description;
    const char* plugin_author;
};

enum FusionConfigParamType : uint8_t {
    FUSION_PARAM_FLOAT,
    FUSION_PARAM_INT,
    FUSION_PARAM_BOOL,
    FUSION_PARAM_STRING
};

struct FusionConfigParam {
    const char* param_name;
    FusionConfigParamType param_type;
    const char* param_description;
    union {
        float float_value;
        int int_value;
        bool bool_value;
        const char* string_value;
    };
};

class IFusionAlgorithm {
public:
    virtual ~IFusionAlgorithm() = default;

    virtual bool Initialize() noexcept = 0;
    virtual void Shutdown() noexcept = 0;
    virtual void Reset() noexcept = 0;
    virtual void UpdateImuData(uint8_t nodeId, const FusionImuData& data) = 0;
    virtual void UpdateRadarData(uint8_t nodeId, const FusionRadarData& data) = 0;
    virtual bool GetFusionResult(uint8_t nodeId, FusionResult& result) noexcept = 0;
    virtual void GetPluginInfo(FusionPluginInfo& info) noexcept = 0;
    [[nodiscard]] virtual uint32_t GetConfigParamCount() const noexcept = 0;
    [[nodiscard]] virtual bool GetConfigParam(uint32_t index, FusionConfigParam& param) const noexcept = 0;
    virtual bool SetConfigParam(const char* param_name, const void* value) noexcept = 0;
    [[nodiscard]] virtual bool GetConfigParam(const char* param_name, void* value) const noexcept = 0;
};

typedef IFusionAlgorithm* (*CreateFusionAlgorithmFunc)();
typedef void (*DestroyFusionAlgorithmFunc)(IFusionAlgorithm*);

#define FUSION_PLUGIN_EXPORT(AlgorithmClass) \
    extern "C" FUSION_CORE_API IFusionAlgorithm* CreateFusionAlgorithm() { \
        return new AlgorithmClass(); \
    } \
    extern "C" FUSION_CORE_API void DestroyFusionAlgorithm(IFusionAlgorithm* algo) { \
        delete algo; \
    }

class FUSION_CORE_API FusionCoreManager {
public:
    [[nodiscard]] static FusionCoreManager* GetInstance() noexcept;
    static void DestroyInstance() noexcept;

    virtual bool LoadPlugins(const char* plugin_dir) = 0;
    virtual void UnloadAllPlugins() noexcept = 0;
    [[nodiscard]] virtual uint32_t GetPluginCount() const noexcept = 0;
    [[nodiscard]] virtual bool GetPluginInfo(uint32_t index, FusionPluginInfo& info) const noexcept = 0;

    virtual bool SelectAlgorithm(const char* plugin_name) = 0;
    [[nodiscard]] virtual IFusionAlgorithm* GetCurrentAlgorithm() noexcept = 0;

    virtual void UpdateImu(uint8_t nodeId, const FusionImuData& data) = 0;
    virtual void UpdateRadar(uint8_t nodeId, const FusionRadarData& data) = 0;
    [[nodiscard]] virtual bool GetResult(uint8_t nodeId, FusionResult& result) noexcept = 0;

    [[nodiscard]] virtual size_t GetNodeCount() const noexcept = 0;
    [[nodiscard]] virtual bool IsNodeActive(uint8_t nodeId) const noexcept = 0;
    virtual void ResetNode(uint8_t nodeId) noexcept = 0;
    virtual void SetWorkerThreads(size_t thread_count) noexcept = 0;

protected:
    FusionCoreManager() = default;
    virtual ~FusionCoreManager() = default;
};

extern "C" {
    [[nodiscard]] FUSION_CORE_API bool FusionCore_Init(const char* plugin_dir);
    FUSION_CORE_API void FusionCore_Shutdown() noexcept;
    FUSION_CORE_API void FusionCore_Reset() noexcept;
    FUSION_CORE_API void FusionCore_ResetNode(uint8_t nodeId) noexcept;

    FUSION_CORE_API void FusionCore_UpdateImu(uint8_t nodeId, const FusionImuData* data);
    FUSION_CORE_API void FusionCore_UpdateRadar(uint8_t nodeId, const FusionRadarData* data);
    [[nodiscard]] FUSION_CORE_API bool FusionCore_GetResult(uint8_t nodeId, FusionResult* result);
    [[nodiscard]] FUSION_CORE_API bool FusionCore_HasNewData(uint8_t nodeId);
    [[nodiscard]] FUSION_CORE_API bool FusionCore_HasAnyNewData() noexcept;

    FUSION_CORE_API void FusionCore_SetWeights(float imu_weight, float radar_weight) noexcept;
    FUSION_CORE_API void FusionCore_SetDataSource(bool imu_enabled, bool radar_enabled) noexcept;
    FUSION_CORE_API void FusionCore_SetOutputRate(float max_rate_hz) noexcept;
    FUSION_CORE_API void FusionCore_SetWorkerThreads(size_t thread_count) noexcept;

    [[nodiscard]] FUSION_CORE_API bool FusionCore_SelectAlgorithm(const char* name);
    [[nodiscard]] FUSION_CORE_API uint32_t FusionCore_GetPluginCount() noexcept;
    [[nodiscard]] FUSION_CORE_API bool FusionCore_GetPluginInfo(uint32_t index, FusionPluginInfo* info);

    [[nodiscard]] FUSION_CORE_API size_t FusionCore_GetNodeCount() noexcept;
    [[nodiscard]] FUSION_CORE_API bool FusionCore_IsNodeActive(uint8_t nodeId) noexcept;
}
