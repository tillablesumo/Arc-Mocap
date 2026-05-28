#include "fusion_mixer.h"
#include <cmath>
#include <algorithm>
#include <chrono>

#define DEG_TO_RAD 0.017453292519943295

FusionMixer::FusionMixer() noexcept
    : imu_enabled(true)
    , radar_enabled(false)
    , imu_weight(0.8f)
    , radar_weight(0.2f)
    , output_rate_limit(0.0f)
    , min_output_interval_ms(0.0f)
    , complementary_alpha(0.98f)
    , running(false)
{
}

FusionMixer::~FusionMixer() noexcept {
    Shutdown();
}

bool FusionMixer::Initialize(size_t worker_threads_count) {
    running.store(true);

    for (size_t i = 0; i < worker_threads_count; ++i) {
        worker_threads.emplace_back(&FusionMixer::worker_thread_func, this);
    }

    return true;
}

void FusionMixer::Shutdown() noexcept {
    running.store(false);
    task_cv.notify_all();

    for (auto& thread : worker_threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    worker_threads.clear();

    std::lock_guard<std::mutex> lock(data_mutex);
    node_states.clear();
}

void FusionMixer::Reset() noexcept {
    std::lock_guard<std::mutex> lock(data_mutex);
    for (auto& pair : node_states) {
        pair.second = std::make_unique<NodeFusionState>();
    }
}

void FusionMixer::ResetNode(uint8_t nodeId) noexcept {
    std::lock_guard<std::mutex> lock(data_mutex);
    auto it = node_states.find(nodeId);
    if (it != node_states.end()) {
        it->second = std::make_unique<NodeFusionState>();
    }
}

void FusionMixer::SetWorkerThreadCount(size_t count) {
    if (count < 1) count = 1;
    if (count > 32) count = 32;
    
    running.store(false);
    task_cv.notify_all();
    
    for (auto& thread : worker_threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    worker_threads.clear();
    
    running.store(true);
    for (size_t i = 0; i < count; ++i) {
        worker_threads.emplace_back(&FusionMixer::worker_thread_func, this);
    }
}

void FusionMixer::UpdateImuData(uint8_t nodeId, const FusionImuData& data) {
    if (!imu_enabled.load()) {
        return;
    }
    
    {
        std::lock_guard<std::mutex> lock(task_mutex);
        task_queue.push(std::make_unique<FusionTask>(nodeId, data));
    }
    task_cv.notify_one();
}

void FusionMixer::UpdateRadarData(uint8_t nodeId, const FusionRadarData& data) {
    if (!radar_enabled.load()) {
        return;
    }
    
    {
        std::lock_guard<std::mutex> lock(task_mutex);
        task_queue.push(std::make_unique<FusionTask>(nodeId, data));
    }
    task_cv.notify_one();
}

bool FusionMixer::GetFusionResult(uint8_t nodeId, FusionResult& result) noexcept {
    NodeFusionState* state_ptr = nullptr;

    {
        std::lock_guard<std::mutex> lock(data_mutex);
        auto it = node_states.find(nodeId);
        if (it == node_states.end()) {
            return false;
        }
        state_ptr = it->second.get();
    }

    std::lock_guard<std::mutex> state_lock(state_ptr->state_mutex);

    if (!state_ptr->has_new_result.load()) {
        return false;
    }

    result = state_ptr->current_result;
    state_ptr->has_new_result.store(false);

    return true;
}

void FusionMixer::SetWeights(float imu_w, float radar_w) noexcept {
    imu_weight.store(std::clamp(imu_w, 0.0f, 1.0f));
    radar_weight.store(std::clamp(radar_w, 0.0f, 1.0f));
}

void FusionMixer::GetWeights(float& imu_w, float& radar_w) const noexcept {
    imu_w = imu_weight.load();
    radar_w = radar_weight.load();
}

void FusionMixer::SetDataSourceEnabled(bool imu_en, bool radar_en) noexcept {
    imu_enabled.store(imu_en);
    radar_enabled.store(radar_en);
}

void FusionMixer::GetDataSourceEnabled(bool& imu_en, bool& radar_en) const noexcept {
    imu_en = imu_enabled.load();
    radar_en = radar_enabled.load();
}

bool FusionMixer::HasNewData(uint8_t nodeId) const noexcept {
    std::lock_guard<std::mutex> lock(data_mutex);

    auto it = node_states.find(nodeId);
    if (it == node_states.end()) {
        return false;
    }

    return it->second->has_new_result.load();
}

bool FusionMixer::HasAnyNewData() const noexcept {
    std::lock_guard<std::mutex> lock(data_mutex);

    for (const auto& pair : node_states) {
        if (pair.second->has_new_result.load()) {
            return true;
        }
    }

    return false;
}

void FusionMixer::SetOutputRateLimit(float max_rate_hz) noexcept {
    output_rate_limit = max_rate_hz;
    if (max_rate_hz > 0.0f) {
        min_output_interval_ms = 1000.0f / max_rate_hz;
    } else {
        min_output_interval_ms = 0.0f;
    }
}

size_t FusionMixer::GetNodeCount() const noexcept {
    std::lock_guard<std::mutex> lock(data_mutex);
    return node_states.size();
}

bool FusionMixer::IsNodeActive(uint8_t nodeId) const noexcept {
    std::lock_guard<std::mutex> lock(data_mutex);
    return node_states.find(nodeId) != node_states.end();
}

void FusionMixer::worker_thread_func() {
    while (running.load()) {
        std::unique_lock<std::mutex> lock(task_mutex);
        
        task_cv.wait(lock, [this] { 
            return !running.load() || !task_queue.empty(); 
        });
        
        if (!running.load()) {
            break;
        }
        
        if (task_queue.empty()) {
            continue;
        }
        
        auto task = std::move(task_queue.front());
        task_queue.pop();
        lock.unlock();
        
        process_task(*task);
    }
}

void FusionMixer::process_task(const FusionTask& task) {
    NodeFusionState* state_ptr = nullptr;
    
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        auto it = node_states.find(task.node_id);
        if (it == node_states.end()) {
            node_states[task.node_id] = std::make_unique<NodeFusionState>();
            it = node_states.find(task.node_id);
        }
        state_ptr = it->second.get();
    }
    
    std::lock_guard<std::mutex> state_lock(state_ptr->state_mutex);
    
    if (task.type == FusionTask::TASK_IMU) {
        state_ptr->latest_imu_data = task.imu_data;
        state_ptr->has_new_imu.store(true);
    } else if (task.type == FusionTask::TASK_RADAR) {
        state_ptr->latest_radar_data = task.radar_data;
        state_ptr->has_new_radar.store(true);
    }
    
    perform_fusion(*state_ptr);
}

void FusionMixer::perform_fusion(NodeFusionState& state) noexcept {
    bool new_imu = state.has_new_imu.load();
    bool new_radar = state.has_new_radar.load();
    
    if (!new_imu && !new_radar) {
        return;
    }
    
    uint64_t current_timestamp = 0;
    if (new_imu) {
        current_timestamp = state.latest_imu_data.timestamp;
    } else if (new_radar) {
        current_timestamp = state.latest_radar_data.timestamp;
    }
    
    if (!check_output_rate(current_timestamp, state.last_output_timestamp)) {
        return;
    }
    
    if (radar_enabled.load() && new_radar) {
        weighted_fusion(state.latest_imu_data, state.latest_radar_data, state.current_result, state.last_imu_timestamp);
    } else if (new_imu) {
        double dt = 0.01;
        if (state.last_imu_timestamp > 0 && state.latest_imu_data.timestamp > state.last_imu_timestamp) {
            dt = (state.latest_imu_data.timestamp - state.last_imu_timestamp) / 1000000.0;
            dt = std::clamp(dt, 0.0001, 0.1);
        }
        
        update_quaternion_from_gyro(state.current_result, state.latest_imu_data, dt);
        state.current_result.timestamp = state.latest_imu_data.timestamp;
    }
    
    if (new_imu) {
        state.last_imu_timestamp = state.latest_imu_data.timestamp;
        state.has_new_imu.store(false);
    }
    if (new_radar) {
        state.last_radar_timestamp = state.latest_radar_data.timestamp;
        state.has_new_radar.store(false);
    }
    
    state.last_output_timestamp = current_timestamp;
    state.has_new_result.store(true);
}

void FusionMixer::complementary_filter(const FusionImuData& imu, const FusionRadarData& radar, FusionResult& result, uint64_t last_timestamp) noexcept {
    [[maybe_unused]] auto unused = radar;
    
    double dt = 0.01;
    if (last_timestamp > 0) {
        dt = (imu.timestamp - last_timestamp) / 1000000.0;
        dt = std::clamp(dt, 0.0001, 0.1);
    }
    
    double gx = imu.gyro[0] * DEG_TO_RAD;
    double gy = imu.gyro[1] * DEG_TO_RAD;
    double gz = imu.gyro[2] * DEG_TO_RAD;
    
    double q0 = result.orientation[3];
    double q1 = result.orientation[0];
    double q2 = result.orientation[1];
    double q3 = result.orientation[2];
    
    double dq0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    double dq1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
    double dq2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
    double dq3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);
    
    q0 += dq0 * dt;
    q1 += dq1 * dt;
    q2 += dq2 * dt;
    q3 += dq3 * dt;
    
    double norm = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm > 1e-10) {
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }
    
    double ax = imu.accel[0];
    double ay = imu.accel[1];
    double az = imu.accel[2];
    
    double acc_norm = std::sqrt(ax*ax + ay*ay + az*az);
    if (acc_norm > 1e-10) {
        ax /= acc_norm;
        ay /= acc_norm;
        az /= acc_norm;
        
        double roll = std::atan2(ay, az);
        double pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));
        
        double q0_acc = std::cos(roll/2) * std::cos(pitch/2);
        double q1_acc = std::sin(roll/2) * std::cos(pitch/2);
        double q2_acc = std::cos(roll/2) * std::sin(pitch/2);
        double q3_acc = -std::sin(roll/2) * std::sin(pitch/2);
        
        q0 = complementary_alpha * q0 + (1 - complementary_alpha) * q0_acc;
        q1 = complementary_alpha * q1 + (1 - complementary_alpha) * q1_acc;
        q2 = complementary_alpha * q2 + (1 - complementary_alpha) * q2_acc;
        q3 = complementary_alpha * q3 + (1 - complementary_alpha) * q3_acc;
        
        norm = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        if (norm > 1e-10) {
            q0 /= norm;
            q1 /= norm;
            q2 /= norm;
            q3 /= norm;
        }
    }
    
    result.orientation[3] = (float)q0;
    result.orientation[0] = (float)q1;
    result.orientation[1] = (float)q2;
    result.orientation[2] = (float)q3;
}

void FusionMixer::weighted_fusion(const FusionImuData& imu, const FusionRadarData& radar, FusionResult& result, uint64_t last_timestamp) noexcept {
    double dt = 0.01;
    if (last_timestamp > 0) {
        dt = (imu.timestamp - last_timestamp) / 1000000.0;
        dt = std::clamp(dt, 0.0001, 0.1);
    }
    
    update_quaternion_from_gyro(result, imu, dt);
    
    if (radar.distance > 0.0f) {
        float w_imu = imu_weight.load();
        float w_radar = radar_weight.load();
        
        result.position[2] = w_imu * result.position[2] + w_radar * radar.distance;
        
        if (radar.velocity != 0.0f) {
            result.velocity[2] = w_imu * result.velocity[2] + w_radar * radar.velocity;
        }
    }
    
    result.timestamp = imu.timestamp;
}

void FusionMixer::update_quaternion_from_gyro(FusionResult& result, const FusionImuData& imu, double dt) noexcept {
    double gx = imu.gyro[0] * DEG_TO_RAD;
    double gy = imu.gyro[1] * DEG_TO_RAD;
    double gz = imu.gyro[2] * DEG_TO_RAD;
    
    double q0 = result.orientation[3];
    double q1 = result.orientation[0];
    double q2 = result.orientation[1];
    double q3 = result.orientation[2];
    
    double dq0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    double dq1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
    double dq2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
    double dq3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);
    
    q0 += dq0 * dt;
    q1 += dq1 * dt;
    q2 += dq2 * dt;
    q3 += dq3 * dt;
    
    double norm = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm > 1e-10) {
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }
    
    result.orientation[3] = (float)q0;
    result.orientation[0] = (float)q1;
    result.orientation[1] = (float)q2;
    result.orientation[2] = (float)q3;
}

bool FusionMixer::check_output_rate(uint64_t current_timestamp, uint64_t last_output_timestamp) const noexcept {
    if (min_output_interval_ms <= 0.0f) {
        return true;
    }
    
    if (last_output_timestamp == 0) {
        return true;
    }
    
    double elapsed_ms = (current_timestamp - last_output_timestamp) / 1000.0;
    return elapsed_ms >= min_output_interval_ms;
}
