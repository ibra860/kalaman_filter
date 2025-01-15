# Kalman Filter

This repository contains a simple implementation of the Kalman Filter, designed to estimate the position of inputs such as sine or cosine waves.

## Applied Mathematical Equations

Below are the mathematical equations that drive the Kalman Filter:

![Alt text](https://github.com/ibra860/kalaman_filter/blob/main/image.png)

## System Matrices

The following matrices are used in this implementation:

```cpp
Eigen::MatrixXd A; // System Matrix
Eigen::MatrixXd B; // Input matrix
Eigen::MatrixXd H; // Measurement model (sensor) matrix
Eigen::MatrixXd Q; // Input noise covariance matrix
Eigen::MatrixXd R; // Measurement Noise

// Initial State (State Estimation)
Eigen::VectorXd x;       // Initial state vector
Eigen::MatrixXd sigma;   // Initial state covariance

// Prediction Variables
Eigen::VectorXd x_pred;  // Predicted state
Eigen::MatrixXd sigma_pred; // Predicted state covariance
Eigen::MatrixXd z_hat;   // Estimated measurement

// Correction Variables
Eigen::VectorXd v;       // Innovation
Eigen::MatrixXd s;       // Innovation covariance matrix
Eigen::MatrixXd k;       // Kalman Gain
```

## Features

- Estimation of dynamic states for systems with noisy inputs.
- Configurable matrices for system dynamics and noise models.
- Integration with Eigen library for efficient matrix operations.

## Prerequisites

Ensure you have the following tools installed:

- C++ compiler (supporting C++17 or later)
- Eigen library (a C++ template library for linear algebra)

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/kalman_filter.git
   ```
2. Navigate to the project directory:
   ```bash
   cd kalman_filter
   ```
3. Compile the code using your preferred C++ compiler.

## Usage

- Modify the system matrices (`A`, `B`, `H`, `Q`, `R`) and initial conditions (`x`, `sigma`) in the code to match your system's requirements.
- Run the compiled executable to observe the filter's performance on your input data.

## Contributing

Contributions are welcome! Feel free to open issues or submit pull requests to enhance this implementation.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

## Acknowledgments

- The Eigen library for providing efficient matrix operations.
- The open-source community for guidance and inspiration.
-
