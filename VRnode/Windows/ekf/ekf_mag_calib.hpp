#pragma once

#include <array>
#include <span>
#include <ranges>
#include <cmath>

// Quaternion structure
struct Quaternion {
    double x, y, z, w;
    Quaternion() : x(0), y(0), z(0), w(1) {}
    Quaternion(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {}
    
    // Normalize quaternion
    void normalize() {
        double norm = std::sqrt(x * x + y * y + z * z + w * w);
        if (norm > 0.0) {
            double inv_norm = 1.0 / norm;
            x *= inv_norm;
            y *= inv_norm;
            z *= inv_norm;
            w *= inv_norm;
        }
    }
};

// EKF anti-magnetic interference algorithm class - 10-state extended Kalman filter
class EKFMagCalib {
private:
    // State variables: quaternion(q0,q1,q2,q3) + magnetometer bias(bx,by,bz) + gyroscope bias(gx_b, gy_b, gz_b)
    std::array<double, 10> state; // [q0, q1, q2, q3, bx, by, bz, gx_b, gy_b, gz_b]
    
    // State covariance matrix
    std::array<double, 100> P; // 10x10 matrix
    
    // Process noise covariance
    std::array<double, 10> Q; // 10x10 diagonal matrix
    
    // Observation noise covariance
    std::array<double, 6> R; // 6x6 diagonal matrix
    
    // Gravity acceleration
    static constexpr double GRAVITY = 9.80665;
    
    // Magnetometer reference values (Earth's magnetic field)
    static constexpr double MAG_REF_X = 0.4;
    static constexpr double MAG_REF_Y = 0.0;
    static constexpr double MAG_REF_Z = 0.4;
    
    // Adaptive noise estimation
    bool adaptive_noise_enabled; // Whether to enable adaptive noise estimation
    double noise_adaptation_gain; // Noise adaptation gain
    std::array<double, 6> residual_history; // Residual history for adaptive noise estimation
    size_t residual_history_index; // Residual history index
    
    // Normalize quaternion
    void normalize_quaternion();
    
    // Covariance stabilization
    void stabilize_covariance(std::span<double, 100> P);
    
    // State prediction
    void predict(double gx, double gy, double gz, double dt);
    
    // Observation update
    void update_observation(double ax, double ay, double az, double mx, double my, double mz);
    
    // Compute Jacobian matrices
    void compute_jacobian_F(std::span<double, 100> F, double gx, double gy, double gz, double dt);
    void compute_jacobian_H(std::span<double, 60> H, double ax, double ay, double az, double mx, double my, double mz);
    
    // Matrix operation helper functions
    void matrix_multiply(std::span<const double, 100> A, std::span<const double, 100> B, std::span<double, 100> C);
    void matrix_transpose(std::span<const double, 100> A, std::span<double, 100> At);
    void matrix_add(std::span<const double, 100> A, std::span<const double, 100> B, std::span<double, 100> C);
    void matrix_multiply_6x10_10x10(std::span<const double, 60> H, std::span<const double, 100> P, std::span<double, 60> HP);
    void matrix_multiply_6x10_10x1(std::span<const double, 60> H, std::span<const double, 10> x, std::span<double, 6> Hx);
    void matrix_inverse_6x6(std::span<const double, 36> A, std::span<double, 36> A_inv);
    void matrix_multiply_10x6_6x6(std::span<const double, 60> Ht, std::span<const double, 36> S_inv, std::span<double, 60> HtS_inv);
    void matrix_multiply_10x6_6x1(std::span<const double, 60> K, std::span<const double, 6> y, std::span<double, 10> Ky);
    
public:
    // Constructor
    EKFMagCalib();
    
    // Update function
    Quaternion update(double gx, double gy, double gz, double ax, double ay, double az, 
                     double mx, double my, double mz, double dt);
    
    // Reset EKF state
    void reset();
    
    // Get current state
    Quaternion get_quaternion() const;
    std::array<double, 3> get_mag_bias() const;
    std::array<double, 3> get_gyro_bias() const;
    
    // Configuration parameters (adjustable)
    void set_process_noise(double q_gyro, double q_mag, double q_gyro_bias);
    void set_observation_noise(double r_accel, double r_mag);
    
    // Adaptive noise estimation
    void enable_adaptive_noise(bool enable);
};

// Adjustable parameter explanation:
// 1. Process noise: affects the filter's trust in input data
//    - q_gyro: gyroscope process noise, increases trust in gyroscope data when increased
//    - q_mag: magnetometer bias process noise, bias changes faster when increased
// 2. Observation noise: affects the filter's trust in observation data
//    - r_accel: accelerometer observation noise, less trust in accelerometer data when increased
//    - r_mag: magnetometer observation noise, less trust in magnetometer data when increased
// 3. Parameter adjustment suggestions:
//    - Initial values: q_gyro=0.001, q_mag=0.0001, r_accel=0.1, r_mag=0.1
//    - Severe magnetic interference: increase r_mag, decrease q_mag
//    - Dynamic environment: increase q_gyro, decrease r_accel
