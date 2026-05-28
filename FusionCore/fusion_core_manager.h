#pragma once

#include "fusion_core_api.h"
#include "fusion_mixer.h"
#include <vector>
#include <string>
#include <map>
#include <windows.h>

struct PluginHandle {
    HMODULE hModule;
    IFusionAlgorithm* algorithm;
    FusionPluginInfo info;
    CreateFusionAlgorithmFunc create_func;
    DestroyFusionAlgorithmFunc destroy_func;
    std::string plugin_path;
};

class FusionCoreManagerImpl : public FusionCoreManager {
public:
    FusionCoreManagerImpl();
    virtual ~FusionCoreManagerImpl();

    virtual bool LoadPlugins(const char* plugin_dir) override;
    virtual void UnloadAllPlugins() noexcept override;
    virtual uint32_t GetPluginCount() const noexcept override;
    virtual bool GetPluginInfo(uint32_t index, FusionPluginInfo& info) const noexcept override;

    virtual bool SelectAlgorithm(const char* plugin_name) override;
    virtual IFusionAlgorithm* GetCurrentAlgorithm() noexcept override;

    virtual void UpdateImu(uint8_t nodeId, const FusionImuData& data) override;
    virtual void UpdateRadar(uint8_t nodeId, const FusionRadarData& data) override;
    virtual bool GetResult(uint8_t nodeId, FusionResult& result) noexcept override;

    virtual size_t GetNodeCount() const noexcept override;
    virtual bool IsNodeActive(uint8_t nodeId) const noexcept override;
    virtual void ResetNode(uint8_t nodeId) noexcept override;
    virtual void SetWorkerThreads(size_t thread_count) noexcept override;

    void SetMixerWeights(float imu_weight, float radar_weight);
    void SetMixerDataSource(bool imu_enabled, bool radar_enabled);
    void SetMixerOutputRate(float max_rate_hz);
    bool HasMixerNewData(uint8_t nodeId) const;
    bool HasMixerAnyNewData() const;
    void InitializeMixer();
    void ResetMixer();

private:
    bool LoadPlugin(const std::string& plugin_path);
    void UnloadPlugin(PluginHandle& handle);

    std::vector<PluginHandle> m_plugins;
    std::map<std::string, size_t> m_plugin_name_map;
    IFusionAlgorithm* m_current_algorithm;
    bool m_initialized;

    FusionMixer m_mixer;
};
