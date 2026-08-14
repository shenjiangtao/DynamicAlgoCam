// Copyright (c) NIO Inc. All Rights Reserved.
// Licensed under the MIT License.
//
// nio_kalman_tracker.cpp — Single-target 6D constant-velocity Kalman filter.
//
// All matrices are fixed-size std::array<double, N*N>. Kept inline / explicit
// to avoid pulling Eigen (nio_core stays dependency-free). See header for
// the contract and intended usage.

#include "nio_kalman_tracker.hpp"

#include <cmath>

namespace nio {

namespace {

// Small fixed-size linear-algebra helpers over row-major arrays.

// C = A (m×n) times B (n×p) → C (m×p).
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

// Y (m) = A (m×n) times x (n).
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

// Add identity-scaled noise: P = P + qI  (size n×n).
template <size_t N>
void addToDiagonal(std::array<double, N*N>& P, double q) {
    for (size_t i = 0; i < N; ++i)
        P[i * N + i] += q;
}

// Solve (S) K = H P^T  via Gaussian elimination for 4×4; S is symmetric.
// Returns K (4×6). We solve S (4×4) K_row = H P^T (4×6) — i.e. 6 right-hand
// columns. Implemented as one Gaussian elimination with 6 RHS.
// S is supposed to be symmetric positive-definite.
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

NioKalmanTracker::NioKalmanTracker() {
    x_.fill(0.0);
    P_.fill(0.0);
}

void NioKalmanTracker::init(const NioDetectionResult& det) {
    x_[0] = det.x + 0.5 * det.w;   // cx
    x_[1] = det.y + 0.5 * det.h;   // cy
    x_[2] = det.w;                 // w
    x_[3] = det.h;                 // h
    x_[4] = 0.0;                   // vx
    x_[5] = 0.0;                   // vy
    P_.fill(0.0);
    // Modest initial uncertainty.
    P_[0 * 6 + 0] = P_[1 * 6 + 1] = 10.0;  // position
    P_[2 * 6 + 2] = P_[3 * 6 + 3] = 10.0;  // size
    P_[4 * 6 + 4] = P_[5 * 6 + 5] = 100.0; // velocity — high, since unknown
    initialised_ = true;
}

NioDetectionResult NioKalmanTracker::update(const NioDetectionResult& det) {
    if (!initialised_)
        init(det);

    // 1) Predict step (constant-velocity, dt = dt_).
    // F is 6×6: identity with dt in (vx→cx, vy→cy) slots.
    // Only need x_ = F x_ and P_ = F P_ F^T + Q.
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

    // 2) Update step with measurement z = [cx, cy, w, h].
    std::array<double, 4> z{
        det.x + 0.5 * det.w,
        det.y + 0.5 * det.h,
        det.w,
        det.h
    };

    // H is 4×6 selecting cx, cy, w, h.
    std::array<double, 24> H{}; // 4×6
    H[0 * 6 + 0] = 1.0;
    H[1 * 6 + 1] = 1.0;
    H[2 * 6 + 2] = 1.0;
    H[3 * 6 + 3] = 1.0;

    // Innovation y = z − H x.
    std::array<double, 4> Hx = matVec<4, 6>(H, x_);
    std::array<double, 4> innov{};
    for (size_t i = 0; i < 4; ++i)
        innov[i] = z[i] - Hx[i];

    // S = H P H^T + R.
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

    // Kalman gain K = P H^T S^{-1}. Equivalently solve S K^T = (H P)^T, then transpose.
    // K is 6×4. (H P)^T is 6×4, equal to Ht*P^T... but we already have HP (4×6); its
    // transpose is 6×4 and is the RHS we need: solve S (4×4) * K_row(4×6) = HP (4×6)
    // would give K as 4×6 = P H^T S^{-1} — but K must be 6×4 = P H^T S^{-1}. We actually
    // have K^T = S^{-1} H P, so K = (S^{-1} H P)^T = (HP)^T * S^{-1}?
    //
    // Cleaner direct route: K (6×4) = P H^T (6×4) * S^{-1} (4×4).
    // P H^T = Ht's mirror — we need P (6×6) * H^T (6×4) ⇒ 6×4.
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

    // State update: x = x + K * innov.
    std::array<double, 6> Kinnov = matVec<6, 4>(K, innov);
    for (size_t i = 0; i < 6; ++i)
        x_[i] += Kinnov[i];

    // Covariance update: P = (I − K H) P.
    std::array<double, 36> KH = matMul<6, 4, 6>(K, H);   // 6×6
    std::array<double, 36> eye{}; // 6×6 identity
    for (size_t i = 0; i < 6; ++i) eye[i * 6 + i] = 1.0;
    std::array<double, 36> IKH{};
    for (size_t i = 0; i < 36; ++i)
        IKH[i] = eye[i] - KH[i];
    P_ = matMul<6, 6, 6>(IKH, P_);

    // Emit smoothed box.
    NioDetectionResult out = det; // copies label/stream metadata
    out.x = static_cast<float>(x_[0] - 0.5 * x_[2]);
    out.y = static_cast<float>(x_[1] - 0.5 * x_[3]);
    out.w = static_cast<float>(x_[2]);
    out.h = static_cast<float>(x_[3]);
    return out;
}

NioDetectionResult NioKalmanTracker::predict() {
    if (!initialised_) {
        // Nothing to propagate — return a zero-size box.
        NioDetectionResult r;
        return r;
    }
    // Time-propagate the state without a measurement. Same F as update(),
    // but skip the measurement correction and return the a-priori box.
    double cx = x_[0] + dt_ * x_[4];
    double cy = x_[1] + dt_ * x_[5];
    NioDetectionResult r;
    r.x = static_cast<float>(cx - 0.5 * x_[2]);
    r.y = static_cast<float>(cy - 0.5 * x_[3]);
    r.w = static_cast<float>(x_[2]);
    r.h = static_cast<float>(x_[3]);
    return r;
}

} // namespace nio
