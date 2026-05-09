#ifndef KALMAN_H
#define KALMAN_H

#define KF_STATE_DIM    6
#define KF_MEAS_DIM     3

typedef struct {
    double x[KF_STATE_DIM];
    double P[KF_STATE_DIM][KF_STATE_DIM];
    double Q[KF_STATE_DIM][KF_STATE_DIM];
    double R[KF_MEAS_DIM][KF_MEAS_DIM];
    double F[KF_STATE_DIM][KF_STATE_DIM];
    double H[KF_MEAS_DIM][KF_STATE_DIM];
    double innovation[KF_MEAS_DIM];
    double innovation_magnitude;
    int    initialized;
    int    manoeuvre_detected;
    double dt_s;
    int    update_count;
} KalmanFilter;

void kf_init(KalmanFilter *kf,
             double x0, double y0, double z0,
             double vx0, double vy0, double vz0);
void kf_predict(KalmanFilter *kf, double dt_s);
void kf_update(KalmanFilter *kf,
               double meas_x, double meas_y, double meas_z);
void kf_get_position(const KalmanFilter *kf,
                     double *x, double *y, double *z);
void kf_get_velocity(const KalmanFilter *kf,
                     double *vx, double *vy, double *vz);
void kf_predict_future(const KalmanFilter *kf, double future_s,
                       double *px, double *py, double *pz);
int  kf_manoeuvre_detected(const KalmanFilter *kf);
void kf_set_manoeuvre_mode(KalmanFilter *kf, int active);

#endif /* KALMAN_H */
