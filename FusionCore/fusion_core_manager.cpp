#include "fusion_core_manager.h"
#include <algorithm>
#include <filesystem>

namespace fs = std::filesystem;

static FusionCoreManagerImpl* g_instance = nullptr;

FusionCoreManager* FusionCoreManager::GetInstance() noexcept {
    if (!g_instance) {
        g_instance = new FusionCoreManagerImpl();
    }
    return g_instance;
}

void FusionCoreManager::DestroyInstance() noexcept {
    if (g_instance) {
        delete g_instance;
        g_instance = nullptr;
    }
}

FusionCoreManagerImpl::FusionCoreManagerImpl()
    : m_current_algorithm(nullptr)
    , m_initialized(false)
{
    m_mixer.Initialize(4);
}

FusionCoreManagerImpl::~FusionCoreManagerImpl() {
    UnloadAllPlugins();
}

bool FusionCoreManagerImpl::LoadPlugins(const char* plugin_dir) {
    if (!plugin_dir) {
        return false;
    }

    UnloadAllPlugins();

    std::string dir_path = plugin_dir;
    if (dir_path.empty()) {
        dir_path = "./plugins";
    }

    try {
        if (!fs::exists(dir_path)) {
            fs::create_directories(dir_path);
        }

        for (const auto& entry : fs::directory_iterator(dir_path)) {
            if (entry.path().extension() == ".dll") {
                LoadPlugin(entry.path().string());
            }
        }

        m_initialized = true;
        return !m_plugins.empty();
    }
    catch (...) {
        return false;
    }
}

void FusionCoreManagerImpl::UnloadAllPlugins() noexcept {
    for (auto& handle : m_plugins) {
        UnloadPlugin(handle);
    }
    m_plugins.clear();
    m_plugin_name_map.clear();
    m_current_algorithm = nullptr;
    m_initialized = false;
}

bool FusionCoreManagerImpl::LoadPlugin(const std::string& plugin_path) {
    HMODULE hModule = LoadLibraryA(plugin_path.c_str());
    if (!hModule) {
        return false;
    }

    CreateFusionAlgorithmFunc create_func = (CreateFusionAlgorithmFunc)GetProcAddress(hModule, "CreateFusionAlgorithm");
    DestroyFusionAlgorithmFunc destroy_func = (DestroyFusionAlgorithmFunc)GetProcAddress(hModule, "DestroyFusionAlgorithm");

    if (!create_func || !destroy_func) {
        FreeLibrary(hModule);
        return false;
    }

    IFusionAlgorithm* algo = create_func();
    if (!algo) {
        FreeLibrary(hModule);
        return false;
    }

    if (!algo->Initialize()) {
        destroy_func(algo);
        FreeLibrary(hModule);
        return false;
    }

    PluginHandle handle;
    handle.hModule = hModule;
    handle.algorithm = algo;
    handle.create_func = create_func;
    handle.destroy_func = destroy_func;
    handle.plugin_path = plugin_path;
    algo->GetPluginInfo(handle.info);

    m_plugins.push_back(handle);
    m_plugin_name_map[handle.info.plugin_name] = m_plugins.size() - 1;

    if (m_plugins.size() == 1) {
        m_current_algorithm = algo;
    }

    return true;
}

void FusionCoreManagerImpl::UnloadPlugin(PluginHandle& handle) {
    if (handle.algorithm) {
        handle.algorithm->Shutdown();
        handle.destroy_func(handle.algorithm);
        handle.algorithm = nullptr;
    }
    if (handle.hModule) {
        FreeLibrary(handle.hModule);
        handle.hModule = nullptr;
    }
}

uint32_t FusionCoreManagerImpl::GetPluginCount() const noexcept {
    return (uint32_t)m_plugins.size();
}

bool FusionCoreManagerImpl::GetPluginInfo(uint32_t index, FusionPluginInfo& info) const noexcept {
    if (index >= m_plugins.size()) {
        return false;
    }
    info = m_plugins[index].info;
    return true;
}

bool FusionCoreManagerImpl::SelectAlgorithm(const char* plugin_name) {
    if (!plugin_name) {
        return false;
    }

    auto it = m_plugin_name_map.find(plugin_name);
    if (it == m_plugin_name_map.end()) {
        return false;
    }

    m_current_algorithm = m_plugins[it->second].algorithm;
    return true;
}

IFusionAlgorithm* FusionCoreManagerImpl::GetCurrentAlgorithm() noexcept {
    return m_current_algorithm;
}

void FusionCoreManagerImpl::UpdateImu(uint8_t nodeId, const FusionImuData& data) {
    m_mixer.UpdateImuData(nodeId, data);
}

void FusionCoreManagerImpl::UpdateRadar(uint8_t nodeId, const FusionRadarData& data) {
    m_mixer.UpdateRadarData(nodeId, data);
}

bool FusionCoreManagerImpl::GetResult(uint8_t nodeId, FusionResult& result) noexcept {
    return m_mixer.GetFusionResult(nodeId, result);
}

size_t FusionCoreManagerImpl::GetNodeCount() const noexcept {
    return m_mixer.GetNodeCount();
}

bool FusionCoreManagerImpl::IsNodeActive(uint8_t nodeId) const noexcept {
    return m_mixer.IsNodeActive(nodeId);
}

void FusionCoreManagerImpl::ResetNode(uint8_t nodeId) noexcept {
    m_mixer.ResetNode(nodeId);
}

void FusionCoreManagerImpl::SetWorkerThreads(size_t thread_count) noexcept {
    m_mixer.SetWorkerThreadCount(thread_count);
}

void FusionCoreManagerImpl::SetMixerWeights(float imu_weight, float radar_weight) {
    m_mixer.SetWeights(imu_weight, radar_weight);
}

void FusionCoreManagerImpl::SetMixerDataSource(bool imu_enabled, bool radar_enabled) {
    m_mixer.SetDataSourceEnabled(imu_enabled, radar_enabled);
}

void FusionCoreManagerImpl::SetMixerOutputRate(float max_rate_hz) {
    m_mixer.SetOutputRateLimit(max_rate_hz);
}

bool FusionCoreManagerImpl::HasMixerNewData(uint8_t nodeId) const {
    return m_mixer.HasNewData(nodeId);
}

bool FusionCoreManagerImpl::HasMixerAnyNewData() const {
    return m_mixer.HasAnyNewData();
}

void FusionCoreManagerImpl::InitializeMixer() {
    m_mixer.Initialize(4);
}

void FusionCoreManagerImpl::ResetMixer() {
    m_mixer.Reset();
}

extern "C" FUSION_CORE_API bool FusionCore_Init(const char* plugin_dir) {
    FusionCoreManager* manager = FusionCoreManager::GetInstance();
    bool loaded = manager->LoadPlugins(plugin_dir);
    
    FusionCoreManagerImpl* impl = dynamic_cast<FusionCoreManagerImpl*>(manager);
    if (impl) {
        impl->InitializeMixer();
    }
    
    return true;
}

extern "C" FUSION_CORE_API void FusionCore_Shutdown() noexcept {
    FusionCoreManager::DestroyInstance();
}

extern "C" FUSION_CORE_API void FusionCore_Reset() noexcept {
    FusionCoreManager* manager = FusionCoreManager::GetInstance();
    FusionCoreManagerImpl* impl = dynamic_cast<FusionCoreManagerImpl*>(manager);
    if (impl) {
        impl->ResetMixer();
    }
}

extern "C" FUSION_CORE_API void FusionCore_ResetNode(uint8_t nodeId) noexcept {
    FusionCoreManager* manager = FusionCoreManager::GetInstance();
    manager->ResetNode(nodeId);
}

extern "C" FUSION_CORE_API void FusionCore_UpdateImu(uint8_t nodeId, const FusionImuData* data) {
    if (!data) return;
    FusionCoreManager* manager = FusionCoreManager::GetInstance();
    manager->UpdateImu(nodeId, *data);
}

extern "C" FUSION_CORE_API void FusionCore_UpdateRadar(uint8_t nodeId, const FusionRadarData* data) {
    if (!data) return;
    FusionCoreManager* manager = FusionCoreManager::GetInstance();
    manager->UpdateRadar(nodeId, *data);
}

extern "C" FUSION_CORE_API bool FusionCore_GetResult(uint8_t nodeId, FusionResult* result) {
    if (!result) return false;
    FusionCoreManager* manager = FusionCoreManager::GetInstance();
    return manager->GetResult(nodeId, *result);
}

extern "C" FUSION_CORE_API bool FusionCore_HasNewData(uint8_t nodeId) {
    FusionCoreManager* manager = FusionCoreManager::GetInstance();
    FusionCoreManagerImpl* impl = dynamic_cast<FusionCoreManagerImpl*>(manager);
    if (impl) {
        return impl->HasMixerNewData(nodeId);
    }
    return false;
}

extern "C" FUSION_CORE_API bool FusionCore_HasAnyNewData() noexcept {
    FusionCoreManager* manager = FusionCoreManager::GetInstance();
    FusionCoreManagerImpl* impl = dynamic_cast<FusionCoreManagerImpl*>(manager);
    if (impl) {
        return impl->HasMixerAnyNewData();
    }
    return false;
}

extern "C" FUSION_CORE_API bool FusionCore_SelectAlgorithm(const char* name) {
    FusionCoreManager* manager = FusionCoreManager::GetInstance();
    return manager->SelectAlgorithm(name);
}

extern "C" FUSION_CORE_API uint32_t FusionCore_GetPluginCount() noexcept {
    FusionCoreManager* manager = FusionCoreManager::GetInstance();
    return manager->GetPluginCount();
}

extern "C" FUSION_CORE_API bool FusionCore_GetPluginInfo(uint32_t index, FusionPluginInfo* info) {
    if (!info) return false;
    FusionCoreManager* manager = FusionCoreManager::GetInstance();
    return manager->GetPluginInfo(index, *info);
}

extern "C" FUSION_CORE_API size_t FusionCore_GetNodeCount() noexcept {
    FusionCoreManager* manager = FusionCoreManager::GetInstance();
    return manager->GetNodeCount();
}

extern "C" FUSION_CORE_API bool FusionCore_IsNodeActive(uint8_t nodeId) noexcept {
    FusionCoreManager* manager = FusionCoreManager::GetInstance();
    return manager->IsNodeActive(nodeId);
}

extern "C" FUSION_CORE_API void FusionCore_SetWeights(float imu_weight, float radar_weight) noexcept {
    FusionCoreManager* manager = FusionCoreManager::GetInstance();
    FusionCoreManagerImpl* impl = dynamic_cast<FusionCoreManagerImpl*>(manager);
    if (impl) {
        impl->SetMixerWeights(imu_weight, radar_weight);
    }
}

extern "C" FUSION_CORE_API void FusionCore_SetDataSource(bool imu_enabled, bool radar_enabled) noexcept {
    FusionCoreManager* manager = FusionCoreManager::GetInstance();
    FusionCoreManagerImpl* impl = dynamic_cast<FusionCoreManagerImpl*>(manager);
    if (impl) {
        impl->SetMixerDataSource(imu_enabled, radar_enabled);
    }
}

extern "C" FUSION_CORE_API void FusionCore_SetOutputRate(float max_rate_hz) noexcept {
    FusionCoreManager* manager = FusionCoreManager::GetInstance();
    FusionCoreManagerImpl* impl = dynamic_cast<FusionCoreManagerImpl*>(manager);
    if (impl) {
        impl->SetMixerOutputRate(max_rate_hz);
    }
}

extern "C" FUSION_CORE_API void FusionCore_SetWorkerThreads(size_t thread_count) noexcept {
    FusionCoreManager* manager = FusionCoreManager::GetInstance();
    FusionCoreManagerImpl* impl = dynamic_cast<FusionCoreManagerImpl*>(manager);
    if (impl) {
        impl->SetWorkerThreads(thread_count);
    }
}
