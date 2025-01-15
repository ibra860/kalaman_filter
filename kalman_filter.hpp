#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <iostream>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Dense>
#include <cmath>




class KalmanFilter {

    private :
    //system params
        Eigen::MatrixXd A; //System Matrix
        Eigen::MatrixXd B; //Input matrix
        Eigen::MatrixXd H; //measurment model (sensor) matrix 
        Eigen::MatrixXd Q; //input noise covariance matrix
        Eigen::MatrixXd R; // Measurment Noise

        //Initial State (State Estimation)
        Eigen::VectorXd x; //initial state vector 
        Eigen::MatrixXd sigma; //initial state covariance

        //Predection Variables
        Eigen::VectorXd x_pred;
        Eigen::MatrixXd sigma_pred;
        Eigen::MatrixXd z_hat;

        //Correction variabbles
        Eigen::VectorXd v; //innovation
        Eigen::MatrixXd s; //innivation covariance matrix
        Eigen::MatrixXd k; //kalman Gain
    public :
        //
        KalmanFilter(const Eigen::MatrixXd& A_, const Eigen::MatrixXd& B_, 
                    const Eigen::MatrixXd& H_, const Eigen::MatrixXd& Q_, 
                    const Eigen::MatrixXd& R_, const Eigen::VectorXd& x_,
                    const Eigen::MatrixXd& sigma_)
                    : A(A_), B(B_), H(H_), Q(Q_), R(R_), x(x_), sigma(sigma_){}
                    
        void set_x(const Eigen::VectorXd& x);


        void predection(){
            x_pred = A * x; 

            // Prediction step for the covariance
            sigma_pred = A * sigma * A.transpose() + Q;

            // Predicted measurement
            z_hat = H * x_pred; 
        }

        void correction(const Eigen::VectorXd& z) {
            v = z - z_hat; // Innovation
            s = H * sigma_pred * H.transpose() + R; // Innovation covariance
            k = sigma_pred * H.transpose() * s.inverse(); // Kalman gain
            x = x_pred + k * v; // State update
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x.size(), x.size());
            sigma = (I - k * H) * sigma_pred; // Covariance update
        }

// x_axis and y_axis sizes do not match because the y_axis function is not returning the correct size, so put them all in
        Eigen::VectorXd x_axis(double start, double stop, double step) {

            int size = std::abs(((stop - start) / step) + 1);
            Eigen::VectorXd range(size);

            // Fill the vector with values
            for (int i = 0; i < size; ++i) {
                range[i] = start + i * step;
            }

            return range;
        }

        Eigen::VectorXd y_axis(Eigen::VectorXd& x_vector){

            int size = x_vector.size();
            Eigen::VectorXd y(size); // Initialize y with the same size as x_vector
            for(int i = 0; i < size ; i++){
                y[i] = sin(x_vector[i]);
            }
            return y;
        }

        //Getter function for x
        Eigen::VectorXd get_x() {
            return x;
        }


        //Create funntions for cholesky

};

#endif // KALMAN_FILTER_HPP

//Kalman_filter: /usr/include/eigen3/Eigen/src/Core/DenseCoeffsBase.h:410: Eigen::DenseCoeffsBase<Derived, 1>::Scalar& Eigen::DenseCoeffsBase<Derived, 1>::operator[](Eigen::Index) [with Derived = Eigen::Matrix<double, -1, 1>; Eigen::DenseCoeffsBase<Derived, 1>::Scalar = double; Eigen::Index = long int]: Assertion `index >= 0 && index < size()' failed. Aborted (core dumped)