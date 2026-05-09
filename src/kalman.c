#include "kalman.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#ifndef _QNX_
#include "qnx_compat.h"
#endif

// Matrix helpers (operate on fixed 6x6 or 3x3 blocks)

static void mat6_zero(double A[6][6]) {
    memset(A, 0, sizeof(double) * 36);
}

static void mat6_identity(double A[6][6]) {
    mat6_zero(A);
    for (int i = 0; i < 6; i++) A[i][i] = 1.0;
}

// C = A * B  (6x6)
static void mat6_mul(const double A[6][6], const double B[6][6],
                     double C[6][6]) {
    double tmp[6][6];
    memset(tmp, 0, sizeof(tmp));
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            for (int k = 0; k < 6; k++)
                tmp[i][j] += A[i][k] * B[k][j];
    memcpy(C, tmp, sizeof(tmp));
}

// C = A + B  (6x6)
static void mat6_add(const double A[6][6], const double B[6][6],
                     double C[6][6]) {
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            C[i][j] = A[i][j] + B[i][j];
}

// C = A - B  (6x6)
static void mat6_sub(const double A[6][6], const double B[6][6],
                     double C[6][6]) {
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            C[i][j] = A[i][j] - B[i][j];
}

// Transpose of 6x6 into B
static void mat6_transpose(const double A[6][6], double B[6][6]) {
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            B[j][i] = A[i][j];
}

// y = A * x  (6x6 * 6x1)
static void mat6_vec_mul(const double A[6][6], const double x[6],
                         double y[6]) {
    for (int i = 0; i < 6; i++) {
        y[i] = 0.0;
        for (int k = 0; k < 6; k++) y[i] += A[i][k] * x[k];
    }
}

// 3x3 inverse (used for innovation covariance S)
static int mat3_inverse(const double A[3][3], double B[3][3]) {
    double det = A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1])
               - A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])
               + A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);
    if (fabs(det) < 1e-12) return -1;
    double inv = 1.0 / det;
    B[0][0] =  (A[1][1]*A[2][2]-A[1][2]*A[2][1]) * inv;
    B[0][1] = -(A[0][1]*A[2][2]-A[0][2]*A[2][1]) * inv;
    B[0][2] =  (A[0][1]*A[1][2]-A[0][2]*A[1][1]) * inv;
    B[1][0] = -(A[1][0]*A[2][2]-A[1][2]*A[2][0]) * inv;
    B[1][1] =  (A[0][0]*A[2][2]-A[0][2]*A[2][0]) * inv;
    B[1][2] = -(A[0][0]*A[1][2]-A[0][2]*A[1][0]) * inv;
    B[2][0] =  (A[1][0]*A[2][1]-A[1][1]*A[2][0]) * inv;
    B[2][1] = -(A[0][0]*A[2][1]-A[0][1]*A[2][0]) * inv;
    B[2][2] =  (A[0][0]*A[1][1]-A[0][1]*A[1][0]) * inv;
    return 0;
}

// Process noise — tuned for hypersonic manoeuvring targets
#define SIGMA_ACC_NORMAL    0.5     // km/s² std dev in cruise
#define SIGMA_ACC_MANOEUVER 5.0     // km/s² std dev when evading
#define MANOEUVRE_THRESHOLD 8.0     // magnitude (km)

static void build_Q(KalmanFilter *kf, double dt) {
    // Singer model process noise for manoeuvring target
    double sigma = kf->manoeuvre_detected ?
                   SIGMA_ACC_MANOEUVER : SIGMA_ACC_NORMAL;
    double q = sigma * sigma;
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    mat6_zero(kf->Q);
    // Position noise
    kf->Q[0][0] = q * dt4 / 4.0;
    kf->Q[1][1] = q * dt4 / 4.0;
    kf->Q[2][2] = q * dt4 / 4.0;
    // Velocity noise
    kf->Q[3][3] = q * dt2;
    kf->Q[4][4] = q * dt2;
    kf->Q[5][5] = q * dt2;
    // Cross terms
    kf->Q[0][3] = kf->Q[3][0] = q * dt3 / 2.0;
    kf->Q[1][4] = kf->Q[4][1] = q * dt3 / 2.0;
    kf->Q[2][5] = kf->Q[5][2] = q * dt3 / 2.0;
}

static void build_F(KalmanFilter *kf, double dt) {
    mat6_identity(kf->F);
    // Position += velocity * dt
    kf->F[0][3] = dt;
    kf->F[1][4] = dt;
    kf->F[2][5] = dt;
}

// Public API

void kf_init(KalmanFilter *kf,
             double x0, double y0, double z0,
             double vx0, double vy0, double vz0) {
    memset(kf, 0, sizeof(KalmanFilter));

    kf->x[0] = x0;  kf->x[1] = y0;  kf->x[2] = z0;
    kf->x[3] = vx0; kf->x[4] = vy0; kf->x[5] = vz0;

    // Initial covariance — high uncertainty in velocity
    mat6_zero(kf->P);
    kf->P[0][0] = 1.0;   kf->P[1][1] = 1.0;   kf->P[2][2] = 1.0;
    kf->P[3][3] = 25.0;  kf->P[4][4] = 25.0;  kf->P[5][5] = 25.0;

    // Measurement noise R (radar position accuracy ~0.5km std dev)
    memset(kf->R, 0, sizeof(kf->R));
    kf->R[0][0] = 0.25; kf->R[1][1] = 0.25; kf->R[2][2] = 0.25;

    // H: observe x,y,z from state
    memset(kf->H, 0, sizeof(kf->H));
    kf->H[0][0] = 1.0;
    kf->H[1][1] = 1.0;
    kf->H[2][2] = 1.0;

    kf->initialized = 1;
    kf->update_count = 0;
}

void kf_predict(KalmanFilter *kf, double dt_s) {
    if (!kf->initialized) return;
    kf->dt_s = dt_s;

    build_F(kf, dt_s);
    build_Q(kf, dt_s);

    // x = F * x
    double new_x[6];
    mat6_vec_mul(kf->F, kf->x, new_x);
    memcpy(kf->x, new_x, sizeof(new_x));

    // P = F * P * F^T + Q
    double Ft[6][6], FP[6][6], FPFt[6][6];
    mat6_transpose(kf->F, Ft);
    mat6_mul(kf->F, kf->P, FP);
    mat6_mul(FP, Ft, FPFt);
    mat6_add(FPFt, kf->Q, kf->P);
}

void kf_update(KalmanFilter *kf,
               double meas_x, double meas_y, double meas_z) {
    if (!kf->initialized) return;

    double z[3] = { meas_x, meas_y, meas_z };

    // Innovation: y = z - H*x
    double Hx[3] = { kf->x[0], kf->x[1], kf->x[2] };
    double innov[3] = {
        z[0] - Hx[0],
        z[1] - Hx[1],
        z[2] - Hx[2]
    };
    kf->innovation[0] = innov[0];
    kf->innovation[1] = innov[1];
    kf->innovation[2] = innov[2];
    kf->innovation_magnitude = sqrt(innov[0]*innov[0] +
                                    innov[1]*innov[1] +
                                    innov[2]*innov[2]);

    // Detect manoeuvre if innovation is large
    kf->manoeuvre_detected =
        (kf->innovation_magnitude > MANOEUVRE_THRESHOLD) ? 1 : 0;

    // S = H * P * H^T + R  (3x3)
    double S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = kf->P[i][j] + kf->R[i][j];

    double S_inv[3][3];
    if (mat3_inverse(S, S_inv) != 0) return;

    // K = P * H^T * S^-1  (6x3)
    double K[6][3];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++) {
            K[i][j] = 0.0;
            for (int k = 0; k < 3; k++)
                K[i][j] += kf->P[i][k] * S_inv[k][j];
        }

    // x = x + K * innov
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++)
            kf->x[i] += K[i][j] * innov[j];

    // P = (I - K*H) * P
    double KH[6][6];
    memset(KH, 0, sizeof(KH));
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++)
            KH[i][j] += K[i][j];   /* H extracts first 3 cols */

    double I_KH[6][6];
    mat6_identity(I_KH);
    mat6_sub(I_KH, KH, I_KH);

    double new_P[6][6];
    mat6_mul(I_KH, kf->P, new_P);
    memcpy(kf->P, new_P, sizeof(new_P));

    kf->update_count++;
}

void kf_get_position(const KalmanFilter *kf,
                     double *x, double *y, double *z) {
    *x = kf->x[0]; *y = kf->x[1]; *z = kf->x[2];
}

void kf_get_velocity(const KalmanFilter *kf,
                     double *vx, double *vy, double *vz) {
    *vx = kf->x[3]; *vy = kf->x[4]; *vz = kf->x[5];
}

void kf_predict_future(const KalmanFilter *kf, double future_s,
                       double *px, double *py, double *pz) {
    *px = kf->x[0] + kf->x[3] * future_s;
    *py = kf->x[1] + kf->x[4] * future_s;
    *pz = kf->x[2] + kf->x[5] * future_s;
}

int kf_manoeuvre_detected(const KalmanFilter *kf) {
    return kf->manoeuvre_detected;
}

void kf_set_manoeuvre_mode(KalmanFilter *kf, int active) {
    kf->manoeuvre_detected = active;
}
