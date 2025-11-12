package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode.robot;
//
//public class HeadingKalmanFilter {
//    private double theta;   // Estimated heading
//    private double omega;   // Estimated angular velocity
//
//    private double[][] P = {{1, 0}, {0, 1}};  // 2x2 covariance matrix
//
//    // Process noise (how much we trust our motion model)
//    private final double[][] Q = {{0.001, 0}, {0, 0.01}};
//
//    // Measurement noise
//    private final double R_imu  = 0.03;
//    private final double R_odo = 0.05;
//
//    public HeadingKalmanFilter(double initialHeading) {
//        theta = initialHeading;
//        omega = 0;
//    }
//
//    /** Predict step: advance heading based on angular velocity */
//    public void predict(double dt) {
//        // State prediction
//        theta = wrapAngle(theta + omega * dt);
//
//        // Covariance prediction: P = F P F^T + Q
//        double[][] F = {{1, dt}, {0, 1}};
//        double[][] Pnew = new double[2][2];
//        Pnew[0][0] = F[0][0]*P[0][0]*F[0][0] + F[0][0]*P[0][1]*F[0][1] +
//                F[0][1]*P[1][0]*F[0][0] + F[0][1]*P[1][1]*F[0][1] + Q[0][0];
//        Pnew[0][1] = F[0][0]*P[0][0]*F[0][1] + F[0][0]*P[0][1]*F[0][1] +
//                F[0][1]*P[1][0]*F[0][1] + F[0][1]*P[1][1]*F[0][1]; // simplified
//        Pnew[1][0] = Pnew[0][1];
//        Pnew[1][1] = P[1][1] + Q[1][1];
//        P = Pnew;
//    }
//
//    /** Update with a single sensor */
//    private void updateMath(double measurement, double R) {
//        // Innovation
//        double y = angleDiff(measurement, theta);
//
//        // Innovation covariance: S = H P H^T + R
//        double S = P[0][0] + R;
//
//        // Kalman gain
//        double K0 = P[0][0] / S;
//        double K1 = P[1][0] / S;
//
//        // Correct state
//        theta = wrapAngle(theta + K0 * y);
//        omega += K1 * y;
//
//        // Update covariance
//        double[][] Pnew = new double[2][2];
//        Pnew[0][0] = (1 - K0) * P[0][0];
//        Pnew[0][1] = (1 - K0) * P[0][1];
//        Pnew[1][0] = -K1 * P[0][0] + P[1][0];
//        Pnew[1][1] = -K1 * P[0][1] + P[1][1];
//        P = Pnew;
//    }
//
//    /** Update with both sensors */
//    public void update(double imuHeading, double odoHeading) {
//        updateMath(imuHeading, R_imu);
//        updateMath(odoHeading, R_odo);
//    }
//
//    public double getHeading() {
//        return theta;
//    }
//
//    // --- Helper functions ---
//    private double wrapAngle(double angle) {
//        while (angle > Math.PI) angle -= 2*Math.PI;
//        while (angle < -Math.PI) angle += 2*Math.PI;
//        return angle;
//    }
//
//    private double angleDiff(double a, double b) {
//        double d = a - b;
//        while (d > Math.PI) d -= 2*Math.PI;
//        while (d < -Math.PI) d += 2*Math.PI;
//        return d;
//    }
//}
