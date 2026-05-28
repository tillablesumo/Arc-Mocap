#pragma once

#include "fusion_core_api.h"
#include <mutex>
#include <atomic>
#include <map>
#include <thread>
#include <condition_variable>
#include <queue>
#include <memory>

struct NodeFusionState {
    FusionImuData latest_imu_data;
    FusionRadarData latest_radar_data;
    FusionResult current_result;

    std::atomic<bool> has_new_imu;
    std::atomic<bool> has_new_radar;
    std::atomic<bool> has_new_result;

    uint64_t last_imu_timestamp;
    uint64_t last_radar_timestamp;
    uint64_t last_output_timestamp;

    std::mutex state_mutex;

    NodeFusionState() noexcept
        : has_new_imu(false), has_new_radar(false), has_new_result(false),
          last_imu_timestamp(0), last_radar_timestamp(0), last_output_timestamp(0) {
        for (int i = 0; i < 4; ++i) {
            current_result.orientation[i] = 0.0f;
        }
        current_result.orientation[3] = 1.0f;
        for (int i = 0; i < 3; ++i) {
            current_result.position[i] = 0.0f;
            current_result.velocity[i] = 0.0f;
            current_result.angular_velocity[i] = 0.0f;
            current_result.linear_acceleration[i] = 0.0f;
            current_result.magnetic_field[i] = 0.0f;
        }
        current_result.timestamp = 0;
        current_result.valid = true;
    }
};

struct FusionTask {
    uint8_t node_id;
    enum { TASK_IMU, TASK_RADAR } type;
    FusionImuData imu_data;
    FusionRadarData radar_data;

    FusionTask(uint8_t nodeId, const FusionImuData& data) noexcept
        : node_id(nodeId), type(TASK_IMU), imu_data(data) {}
    FusionTask(uint8_t nodeId, const FusionRadarData& data) noexcept
        : node_id(nodeId), type(TASK_RADAR), radar_data(data) {}
};

class FusionMixer {
public:
    FusionMixer() noexcept;
    ~FusionMixer() noexcept;

    bool Initialize(size_t worker_threads = 4);
    void Shutdown() noexcept;
    void Reset() noexcept;
    void ResetNode(uint8_t nodeId) noexcept;

    void UpdateImuData(uint8_t nodeId, const FusionImuData& data);
    void UpdateRadarData(uint8_t nodeId, const FusionRadarData& data);
    [[nodiscard]] bool GetFusionResult(uint8_t nodeId, FusionResult& result) noexcept;

    void SetWeights(float imu_weight, float radar_weight) noexcept;
    void GetWeights(float& imu_weight, float& radar_weight) const noexcept;

    void SetDataSourceEnabled(bool imu_enabled, bool radar_enabled) noexcept;
    void GetDataSourceEnabled(bool& imu_enabled, bool& radar_enabled) const noexcept;

    [[nodiscard]] bool HasNewData(uint8_t nodeId) const noexcept;
    [[nodiscard]] bool HasAnyNewData() const noexcept;

    void SetOutputRateLimit(float max_rate_hz) noexcept;

    [[nodiscard]] size_t GetNodeCount() const noexcept;
    [[nodiscard]] bool IsNodeActive(uint8_t nodeId) const noexcept;

    void SetWorkerThreadCount(size_t count);

private:
    mutable std::mutex data_mutex;
    std::mutex task_mutex;
    std::condition_variable task_cv;

    std::map<uint8_t, std::unique_ptr<NodeFusionState>> node_states;

    std::atomic<bool> imu_enabled;
    std::atomic<bool> radar_enabled;
    std::atomic<float> imu_weight;
    std::atomic<float> radar_weight;

    float output_rate_limit;
    float min_output_interval_ms;
    float complementary_alpha;

    std::atomic<bool> running;
    std::vector<std::thread> worker_threads;
    std::queue<std::unique_ptr<FusionTask>> task_queue;

    void worker_thread_func();
    void process_task(const FusionTask& task);
    void perform_fusion(NodeFusionState& state) noexcept;
    void complementary_filter(const FusionImuData& imu, const FusionRadarData& radar, FusionResult& result, uint64_t last_timestamp) noexcept;
    void weighted_fusion(const FusionImuData& imu, const FusionRadarData& radar, FusionResult& result, uint64_t last_timestamp) noexcept;
    void update_quaternion_from_gyro(FusionResult& result, const FusionImuData& imu, double dt) noexcept;
    [[nodiscard]] bool check_output_rate(uint64_t current_timestamp, uint64_t last_output_timestamp) const noexcept;
};
