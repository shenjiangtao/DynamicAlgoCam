// Copyright (c) shenjiangtao. All Rights Reserved.
// Licensed under the MIT License.
//
// dynalgo_kalman_tracker.cpp — Single-target 6D constant-velocity Kalman filter.
//
// [文件说明 / File Description]
// 中文：单目标6D恒定速度卡尔曼滤波器实现，所有矩阵使用固定大小数组避免依赖Eigen
// English: Single-target 6D constant-velocity Kalman filter implementation, all matrices use fixed-size arrays to avoid Eigen dependency
//
// All matrices are fixed-size std::array<double, N*N>. Kept inline / explicit
// to avoid pulling Eigen (dynalgo_core stays dependency-free). See header for
// the contract and intended usage.

#include "dynalgo_kalman_tracker.hpp"

#include <cmath>

namespace dynalgo {

namespace {

// [内部辅助函数 / Internal Helper Functions]
// 中文：小型固定大小线性代数辅助函数，操作行主序数组
// English: Small fixed-size linear-algebra helpers over row-major arrays

// [矩阵乘法 / Matrix Multiplication]
// 中文：矩阵乘法 C = A (m×n) × B (n×p) → C (m×p)
// English: Matrix multiplication C = A (m×n) times B (n×p) → C (m×p)
template <size_t M, size_t N, size_t P>
std::array<double, M*P> matMul(const std::array<double, M*N>& A,
                               const std::array<double, N*P>& B) {
    std::array<double, M*P> C{};
    for (size_t i = 0; i < M; ++i)
        for (size_t k = 0; k < N; ++k) {
            double a = A[i * N + k];
            if (a == 0.0) continue;
            for (size_t j = 0; j < P; ++j)
                C[i * P + j] += a * B[k * P + j];
        }
    return C;
}

// [矩阵向量乘法 / Matrix-Vector Multiplication]
// 中文：矩阵向量乘法 Y (m) = A (m×n) × x (n)
// English: Matrix-vector multiplication Y (m) = A (m×n) times x (n)
template <size_t M, size_t N>
std::array<double, M> matVec(const std::array<double, M*N>& A,
                             const std::array<double, N>& x) {
    std::array<double, M> y{};
    for (size_t i = 0; i < M; ++i) {
        double s = 0.0;
        for (size_t k = 0; k < N; ++k)
            s += A[i * N + k] * x[k];
        y[i] = s;
    }
    return y;
}

// [对角线加法 / Diagonal Addition]
// 中文：添加单位缩放噪声 P = P + qI (n×n)
// English: Add identity-scaled noise: P = P + qI (size n×n)
template <size_t N>
void addToDiagonal(std::array<double, N*N>& P, double q) {
    for (size_t i = 0; i < N; ++i)
        P[i * N + i] += q;
}

// [卡尔曼增益求解 / Kalman Gain Solution]
// 中文：通过高斯消元法求解卡尔曼增益 K = P H^T S^{-1}，S是4×4对称正定矩阵
// English: Solve Kalman gain K = P H^T S^{-1} via Gaussian elimination for 4×4 symmetric positive-definite S
std::array<double, 24> solveKalmanGain(const std::array<double, 16>& S,
                                       const std::array<double, 24>& HPt) {
    // Augmented 4×(4+6) for Gaussian elimination.
    std::array<double, 40> aug{}; // 4 rows × 10 cols
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j)
            aug[i * 10 + j] = S[i * 4 + j];
        for (size_t j = 0; j < 6; ++j)
            aug[i * 10 + 4 + j] = HPt[i * 6 + j];
    }
    // Forward elimination with partial pivoting.
    for (size_t i = 0; i < 4; ++i) {
        // pivot
        size_t pivot = i;
        double maxv = std::fabs(aug[i * 10 + i]);
        for (size_t r = i + 1; r < 4; ++r) {
            double v = std::fabs(aug[r * 10 + i]);
            if (v > maxv) { maxv = v; pivot = r; }
        }
        if (pivot != i) {
            for (size_t c = 0; c < 10; ++c)
                std::swap(aug[i * 10 + c], aug[pivot * 10 + c]);
        }
        double piv = aug[i * 10 + i];
        if (piv == 0.0) continue; // singular — leave row alone
        // normalise pivot row
        for (size_t c = i; c < 10; ++c)
            aug[i * 10 + c] /= piv;
        // eliminate from other rows
        for (size_t r = 0; r < 4; ++r) {
            if (r == i) continue;
            double factor = aug[r * 10 + i];
            if (factor == 0.0) continue;
            for (size_t c = i; c < 10; ++c)
                aug[r * 10 + c] -= factor * aug[i * 10 + c];
        }
    }
    // Extract the 4×6 solution.
    std::array<double, 24> K{};
    for (size_t i = 0; i < 4; ++i)
        for (size_t j = 0; j < 6; ++j)
            K[i * 6 + j] = aug[i * 10 + 4 + j];
    return K;
}

} // namespace

// [构造函数 / Constructor]
// 中文：初始化状态向量和协方差矩阵为零
// English: Initialize state vector and covariance matrix to zero
DynalgoKalmanTracker::DynalgoKalmanTracker() {
    x_.fill(0.0);
    P_.fill(0.0);
}

// [方法说明 / Method Description]
// 中文：用检测结果初始化状态，设置初始位置、大小和速度
// English: Initialize state with detection result, set initial position, size and velocity
void DynalgoKalmanTracker::init(const DynalgoDetectionResult& det) {
    x_[0] = det.x + 0.5 * det.w;   // cx
    x_[1] = det.y + 0.5 * det.h;   // cy
    x_[2] = det.w;                 // w
    x_[3] = det.h;                 // h
    x_[4] = 0.0;                   // vx
    x_[5] = 0.0;                   // vy
    P_.fill(0.0);
    // [初始不确定性 / Initial Uncertainty]
    // 中文：设置初始不确定性，速度不确定性较高因为未知
    // English: Set initial uncertainty, velocity uncertainty high since unknown
    P_[0 * 6 + 0] = P_[1 * 6 + 1] = 10.0;  // position
    P_[2 * 6 + 2] = P_[3 * 6 + 3] = 10.0;  // size
    P_[4 * 6 + 4] = P_[5 * 6 + 5] = 100.0; // velocity — high, since unknown
    initialised_ = true;
}

// [方法说明 / Method Description]
// 中文：合并测量值，执行预测和更新步骤，返回平滑后的边界框
// English: Incorporate measurement, perform predict and update steps, return smoothed bounding box
DynalgoDetectionResult DynalgoKalmanTracker::update(const DynalgoDetectionResult& det) {
    if (!initialised_)
        init(det);

    // [预测步骤 / Predict Step]
    // 中文：恒定速度模型预测，状态转移矩阵F是6×6单位矩阵加上dt在速度位置槽
    // English: Constant-velocity model predict, state transition matrix F is 6×6 identity with dt in velocity-position slots
    {
        // x_ = F x_  (F has 1 on diagonal and dt_ at [0][4] and [1][5]).
        double newCx = x_[0] + dt_ * x_[4];
        double newCy = x_[1] + dt_ * x_[5];
        x_[0] = newCx;
        x_[1] = newCy;
        // x_[2..5] unchanged.

        // F is identity with two extra off-diagonals. We compute P = F P F^T
        // explicitly by row/col transforms for a 6×6 — concise hand-written form.
        // Let A = P (symmetric).
        // Row0 and row1 of F mix in row4 and row5 respectively. Equivalent:
        //   P_ij' = P_ij + dt * (P_{i,4} if j==0 else P_{i,5} if j==1 else 0) ...
        // but it is simpler to just build F, F^T explicitly and use matMul.
        std::array<double, 36> F{}; // 6×6
        for (size_t i = 0; i < 6; ++i) F[i * 6 + i] = 1.0;
        F[0 * 6 + 4] = dt_;
        F[1 * 6 + 5] = dt_;
        std::array<double, 36> Ft{}; // F^T
        for (size_t i = 0; i < 6; ++i)
            for (size_t j = 0; j < 6; ++j)
                Ft[i * 6 + j] = F[j * 6 + i];
        std::array<double, 36> PFt = matMul<6, 6, 6>(P_, Ft);
        P_ = matMul<6, 6, 6>(F, PFt);
        // Q = processNoise_ * I (rough; keeps things stable).
        addToDiagonal<6>(P_, processNoise_);
    }

    // [更新步骤 / Update Step]
    // 中文：使用测量值z = [cx, cy, w, h]更新状态
    // English: Update step with measurement z = [cx, cy, w, h]
    std::array<double, 4> z{
        det.x + 0.5 * det.w,
        det.y + 0.5 * det.h,
        det.w,
        det.h
    };

    // [观测矩阵 / Observation Matrix]
    // 中文：H是4×6矩阵，选择cx, cy, w, h状态分量
    // English: H is 4×6 matrix selecting cx, cy, w, h state components
    std::array<double, 24> H{}; // 4×6
    H[0 * 6 + 0] = 1.0;
    H[1 * 6 + 1] = 1.0;
    H[2 * 6 + 2] = 1.0;
    H[3 * 6 + 3] = 1.0;

    // [新息 / Innovation]
    // 中文：新息 y = z − H x，测量值与预测值的差异
    // English: Innovation y = z − H x, difference between measurement and prediction
    std::array<double, 4> Hx = matVec<4, 6>(H, x_);
    std::array<double, 4> innov{};
    for (size_t i = 0; i < 4; ++i)
        innov[i] = z[i] - Hx[i];

    // [新息协方差 / Innovation Covariance]
    // 中文：新息协方差 S = H P H^T + R
    // English: Innovation covariance S = H P H^T + R
    std::array<double, 24> Htrans{}; // store transpose-of-H is unnecessary; compute H P^T below.
    // HP (4×6) = H * P.
    std::array<double, 24> HP = matMul<4, 6, 6>(H, P_);
    // H P H^T = HP * H^T. H^T is 6×4.
    std::array<double, 24> Ht{}; // 6×4
    for (size_t i = 0; i < 6; ++i)
        for (size_t j = 0; j < 4; ++j)
            Ht[i * 4 + j] = H[j * 6 + i];
    std::array<double, 16> S = matMul<4, 6, 4>(HP, Ht);
    // R = measNoise_ * I (4×4).
    for (size_t i = 0; i < 4; ++i)
        S[i * 4 + i] += measNoise_;

    // [卡尔曼增益 / Kalman Gain]
    // 中文：计算卡尔曼增益 K = P H^T S^{-1}，通过求解线性方程组得到
    // English: Compute Kalman gain K = P H^T S^{-1} by solving linear system
    std::array<double, 24> PHt = matMul<6, 6, 4>(P_, Ht);
    // Solve S * X = PHt^T  ⇒ X (4×6) = S^{-1} * (PHt)^T = K^T.
    // PHt is 6×4; (PHt)^T is 4×6 — call it Rt.
    std::array<double, 24> Rt{}; // 4×6 = transpose of PHt
    for (size_t i = 0; i < 6; ++i)
        for (size_t j = 0; j < 4; ++j)
            Rt[j * 6 + i] = PHt[i * 4 + j];
    std::array<double, 24> Kt = solveKalmanGain(S, Rt); // Kt is 4×6 = (K)^T
    // Reassemble K as 6×4 by transposing Kt.
    std::array<double, 24> K{}; // 6×4
    for (size_t i = 0; i < 4; ++i)
        for (size_t j = 0; j < 6; ++j)
            K[j * 4 + i] = Kt[i * 6 + j];

    // [状态更新 / State Update]
    // 中文：状态更新 x = x + K * innov
    // English: State update: x = x + K * innov
    std::array<double, 6> Kinnov = matVec<6, 4>(K, innov);
    for (size_t i = 0; i < 6; ++i)
        x_[i] += Kinnov[i];

    // [协方差更新 / Covariance Update]
    // 中文：协方差更新 P = (I − K H) P
    // English: Covariance update: P = (I − K H) P
    std::array<double, 36> KH = matMul<6, 4, 6>(K, H);   // 6×6
    std::array<double, 36> eye{}; // 6×6 identity
    for (size_t i = 0; i < 6; ++i) eye[i * 6 + i] = 1.0;
    std::array<double, 36> IKH{};
    for (size_t i = 0; i < 36; ++i)
        IKH[i] = eye[i] - KH[i];
    P_ = matMul<6, 6, 6>(IKH, P_);

    // [输出平滑框 / Emit Smoothed Box]
    // 中文：输出平滑后的边界框
    // English: Emit smoothed box
    DynalgoDetectionResult out = det; // copies label/stream metadata
    out.x = static_cast<float>(x_[0] - 0.5 * x_[2]);
    out.y = static_cast<float>(x_[1] - 0.5 * x_[3]);
    out.w = static_cast<float>(x_[2]);
    out.h = static_cast<float>(x_[3]);
    return out;
}

// [方法说明 / Method Description]
// 中文：在没有测量值的情况下时间传播状态，返回预测的边界框
// English: Time-propagate state without measurement, return predicted bounding box
DynalgoDetectionResult DynalgoKalmanTracker::predict() {
    if (!initialised_) {
        // [未初始化 / Not Initialized]
        // 中文：没有可传播的状态，返回零大小框
        // English: Nothing to propagate — return a zero-size box
        DynalgoDetectionResult r;
        return r;
    }
    // [时间传播 / Time Propagation]
    // 中文：无测量值的时间传播，跳过测量修正返回先验框
    // English: Time-propagate state without measurement, skip measurement correction and return a-priori box
    double cx = x_[0] + dt_ * x_[4];
    double cy = x_[1] + dt_ * x_[5];
    DynalgoDetectionResult r;
    r.x = static_cast<float>(cx - 0.5 * x_[2]);
    r.y = static_cast<float>(cy - 0.5 * x_[3]);
    r.w = static_cast<float>(x_[2]);
    r.h = static_cast<float>(x_[3]);
    return r;
}

} // namespace dynalgo
