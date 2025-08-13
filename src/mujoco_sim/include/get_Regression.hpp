#ifndef GET_REGRESSION_HPP
#define GET_REGRESSION_HPP
#include <rclcpp/rclcpp.hpp>
#include <math.h>

void get_Regression(double* H, const std::array<double, 6>& q, const std::array<double, 6>& dq, const std::array<double, 6>& ddq);

#endif