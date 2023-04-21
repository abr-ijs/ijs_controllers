#pragma once

#include <Eigen/Dense>
#include <vector>


namespace ijs_controllers {

// make a diagonal matrix from a vector
inline Eigen::MatrixXd make_diag(Eigen::VectorXd v) {
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(v.size(), v.size());
  for (size_t i = 0; i < v.size(); ++i) {
    M(i, i) = v(i);
  }
  return M;
}


/* function M_x_sqrt = get_M_x(M_x_inv, min_singular_value)
  * Inverts the input matrix using SVD to obtain the square root of Cartesian mass
  % matrix
  *
  * Inputs:
  *   M_x_inv: inverse of the Cartesian mass matrix (6x6)
  *   min_singular_value: the minimum admissible value for the singular value of M_x_inv
  *
  * Outputs:
  *   M_x_sqrt: the square root of the matrix M_x_inv (6x6)
  */
inline Eigen::MatrixXd get_M_x_sqrt(const Eigen::MatrixXd &M_x_inv,
                       const double min_singular_value) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_x_inv, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::VectorXd singular_values = svd.singularValues();
  const Eigen::MatrixXd U = svd.matrixU();
  const Eigen::MatrixXd V = svd.matrixV();

  // If one of the singular values is below the threshold, 
  // For each of the singular values:
  // If it is below the threshold, set it to the minimum singular value
  if (singular_values.minCoeff() < min_singular_value) {

    for (size_t i = 0; i < singular_values.size(); ++i) {
      if (singular_values(i) < min_singular_value) {
        singular_values(i) = min_singular_value;
      }
    }
  }
    
  // Compute the reciprocal of each singular value
  Eigen::VectorXd singular_values_reciprocal = singular_values;
  singular_values_reciprocal = singular_values_reciprocal.cwiseInverse();

  // Compute the square root of the singular values' reciprocals
  Eigen::VectorXd singular_values_sqrt = singular_values_reciprocal;
  singular_values_sqrt = singular_values_sqrt.cwiseSqrt();

  // Compute the Cartesian mass matrix and its square root
  // note: usage of u instead of v is due to the matrix being p.s.d.
  //Eigen::MatrixXd M_x = U * make_diag(singular_values_reciprocal) * U.transpose(); 
  Eigen::MatrixXd M_x_sqrt = U * make_diag(singular_values_sqrt) * U.transpose(); 

  return M_x_sqrt;
}

/*

Eigen::MatrixXd make_diag(Eigen::VectorXd coeff) {
  const int size = coeff.rows();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> mat(size,size);
  mat.setZero();
  for (size_t i = 0; i < coeff.rows(); i++)
  {
    mat(i,i) = coeff(i);
  }
  return mat;
}


*/



inline Eigen::MatrixXd make_diag(std::vector<double> coeff) {
  const int size = coeff.size();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> mat(size,size);
  for (size_t i = 0; i < size; i++)
  {
    mat(i,i) = coeff[i];
  }
  return mat;

}

}