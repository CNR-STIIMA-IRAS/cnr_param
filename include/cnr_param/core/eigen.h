#ifndef CNR_PARAM_INCLUDE_CNR_PARAM_IMPL_EIGEN
#define CNR_PARAM_INCLUDE_CNR_PARAM_IMPL_EIGEN


#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <type_traits>

namespace cnr
{
namespace param
{
namespace core
{


bool copy(double& lhs, const double& rhs);

bool copy(double& lhs, const std::vector<double>& rhs);

bool copy(std::vector<double>& lhs, const double& rhs);

template<typename D>
bool copy(double& lhs, const Eigen::MatrixBase<D>& rhs);

template<typename D, typename E> 
bool copy(Eigen::MatrixBase<D> & lhs,const Eigen::MatrixBase<E>& rhs);

template<typename D>
bool copy(Eigen::MatrixBase<D>& lhs, const double& rhs);

template<typename D>
bool copy(Eigen::MatrixBase<D>& lhs, const std::vector<double>& rhs);

template<typename D>
bool copy(std::vector<double>& lhs, const Eigen::MatrixBase<D>& rhs);

//! at
template<typename T>
T& at(T& v, const int& i, const int& j=0) ;

template<typename T>
const T& at(const T& v, const int& i, const int& j=0);

template<typename T, typename D>
T& at(Eigen::MatrixBase<D>& m, const int& i, const int& j=0);

template<typename T, typename D>
const T& at(const Eigen::MatrixBase<D>& m, const int& i, const int& j=0);

//! size
template<typename T>
int size(const T& m);

template<typename T>
int size(const Eigen::MatrixBase<T>& m);

template<typename T>
int size(const std::vector<T>& m);

template<typename T>
int size(const std::vector<std::vector<T>>& m);

//!rows
template<typename T>
int rows(const T& m);

template<typename T>
int rows(const std::vector<T>& m);

template<typename T>
int rows(const std::vector<std::vector<T>>& m);

template<typename D> 
int rows(const Eigen::MatrixBase<D>& m);


//! cols
template<typename T>
int cols(const T& m);

template<typename T> 
int cols(const std::vector<T>& m);

template<typename T> 
int cols(const std::vector<std::vector<T>>& m);

template<typename D>
int cols(const Eigen::MatrixBase<D>& m);

//! resize
template<typename D,
          typename std::enable_if<
            (Eigen::MatrixBase<D>::RowsAtCompileTime == Eigen::Dynamic) ||
              (Eigen::MatrixBase<D>::ColsAtCompileTime == Eigen::Dynamic), 
                int >::type = 0 >
bool resize(Eigen::MatrixBase<D> const & m, int rows, int cols = 1);

template<typename D,
          typename std::enable_if<
            (Eigen::MatrixBase<D>::RowsAtCompileTime != Eigen::Dynamic) &&
              (Eigen::MatrixBase<D>::ColsAtCompileTime != Eigen::Dynamic),
                  int>::type = 0>
bool resize(Eigen::MatrixBase<D> const & m, int rows, int cols = 1);

bool resize(const double& m, int rows, int cols = 1);

template<typename T>
bool resize(std::vector<T>& m, int rows, int cols = 1);

template<typename T>
bool resize(std::vector<T>& m1, const std::vector<T>& m2);

template<typename T>
bool resize(T& /*m1*/, const T& /*m2*/);



//============
template<typename T>
std::string to_string(const T& m, bool transpose = true);

template<typename Derived>
std::string to_string(const Eigen::MatrixBase<Derived>& m, bool transpose = true);
  
}  // namespace core
}  // namespace param
}  // namespace core


#include <cnr_param/core/eigen_impl.hpp>

#endif  /* INCLUDE_CNR_PARAM_UTILS_EIGEN */
