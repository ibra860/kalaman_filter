#include <iostream>
#include "kalman_filter.hpp"
//can you see my hpp file?


int main() {
        // Define the system parameters
        Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
        Eigen::MatrixXd B(2, 1); // Not used in this example
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(2, 2);
        Eigen::MatrixXd Q = 1e-3 * Eigen::MatrixXd::Identity(2, 2);
        Eigen::MatrixXd R = 0.05 * 0.05 * Eigen::MatrixXd::Identity(2, 2);

        // Initialize the state using the first measurement
        Eigen::VectorXd x(2);
        x << 0, 0; // Initial state will be set later

        Eigen::MatrixXd sigma = 2 * Eigen::MatrixXd::Identity(2, 2);

        // Create KalmanFilter object
        KalmanFilter kf(A, B, H, Q, R, x, sigma);

        // Generate x_axis and y_axis
        Eigen::VectorXd x_axis = kf.x_axis(-5, 5, 0.1);
        Eigen::VectorXd y_axis = kf.y_axis(x_axis); // Assuming y = x for ground truth

        // Generate noisy measurements
        Eigen::MatrixXd z(2, x_axis.size());
        Eigen::MatrixXd L = R.llt().matrixL();
        for (int i = 0; i < x_axis.size(); ++i) {
            Eigen::VectorXd noise = L * Eigen::VectorXd::Random(2);
            z.col(i) << x_axis[i], y_axis[i] + noise[1];
        }

        //class "KalmanFilter" has no member "set_x"

        // Run the Kalman filter
        for (int i = 1; i < x_axis.size(); ++i) {
            kf.predection();
            kf.correction(z.col(i));

            // std::cout << "original axis " << i << ": " << x_axis <<" " << y_axis << std::endl;
            std::cout << "State estimate at step " << i << ": " << kf.get_x().transpose() << std::endl;
        }

        return 0;
    }