#include <stdio.h>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
using namespace std;

/*!
//  计算伪逆矩阵
 * Compute the pseudo inverse of a matrix
 * @param matrix : input matrix
 * @param sigmaThreshold : threshold for singular values being zero
 * @param invMatrix : output matrix
 */
template <typename T>         //通用的伪逆函数，用于计算矩阵的伪逆
void pseudoInverse(DMat<T> const& matrix, double sigmaThreshold,   
                   DMat<T>& invMatrix) {
  if ((1 == matrix.rows()) && (1 == matrix.cols())) {              //特殊情况处理，1*1矩阵。
    invMatrix.resize(1, 1); 
    if (matrix.coeff(0, 0) > sigmaThreshold) {
      invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
    } else {
      invMatrix.coeffRef(0, 0) = 0.0;
    }
    return;
  }
  //计算伪逆函数
  Eigen::JacobiSVD<DMat<T>> svd(matrix,Eigen::ComputeThinU | Eigen::ComputeThinV);        //奇异值分解，计算矩阵的奇异值分解。   指定计算U和V矩阵，有助于提高计算效率
  // not sure if we need to svd.sort()... probably not
  int const nrows(svd.singularValues().rows());                     //初始化逆奇异值矩阵，返回奇异值矩阵的长度，即奇异值的数量。
  DMat<T> invS;
  invS = DMat<T>::Zero(nrows, nrows);                           //用于存储奇异值的倒数。
  for (int ii(0); ii < nrows; ++ii) {               
    if (svd.singularValues().coeff(ii) > sigmaThreshold) {              //计算逆奇异值
      invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
    } else {
      // invS.coeffRef(ii, ii) = 1.0/ sigmaThreshold;
      // printf("sigular value is too small: %f\n",
      // svd.singularValues().coeff(ii));
    }
  }
  invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();       //计算伪逆矩阵。J+ = V*S+*U^T
}
/*伪逆是一个广义逆，用于解决线性方程组无法通过常规逆矩阵求解的情况。它在最小二乘问题和优化问题中广泛应用。

    定义：对于任意矩阵 J，其伪逆 J+ 满足以下四个条件：
        JJ+J=J
        J+JJ+=J+
        (JJ+)^T=JJ+
        (J+J)^T=J+J

    计算方法：通过 SVD 分解，将矩阵分解为 J=UΣV^T，然后定义 J+=VΣ+U^T，其中 Σ+ 是对角矩阵，包含非零奇异值的倒数。

奇异值分解（SVD）

奇异值分解是将任何 m×n矩阵 J 分解为三个矩阵的乘积：
J=UΣVT


    U: m×m 正交矩阵。
    Σ: m×n 对角矩阵，包含奇异值。
    V: n×n 正交矩阵。

SVD 在计算伪逆时提供了一个稳健的方法，尤其在处理奇异矩阵或接近奇异矩阵时。 */