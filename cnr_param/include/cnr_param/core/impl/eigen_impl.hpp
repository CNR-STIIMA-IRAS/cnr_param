#ifndef CNR_PARAM_INCLUDE_CNR_PARAM_IMPL_EIGEN_IMPL
#define CNR_PARAM_INCLUDE_CNR_PARAM_IMPL_EIGEN_IMPL


#if !defined(__PRETTY_FUNCTION__) && !defined(__GNUC__)
#define __PRETTY_FUNCTION__ __FUNCSIG__
#endif

#if !defined(UNUSED)
#define UNUSED(expr) do { (void)(expr); } while (0)
#endif

#include <cnr_param/core/eigen.h>


namespace cnr 
{
namespace param
{
namespace core
{

/**
 * RESIZE - SAFE FUNCTION CALLED ONLY IF THE MATRIX IS DYNAMICALLY CREATED AT RUNTIME
 */
template<typename Derived,
         typename std::enable_if< (Eigen::MatrixBase<Derived>::RowsAtCompileTime == Eigen::Dynamic) 
                        || (Eigen::MatrixBase<Derived>::ColsAtCompileTime == Eigen::Dynamic) 
                        , int>::type >
inline bool resize(Eigen::MatrixBase<Derived> const & m, int rows, int cols)
{
  Eigen::MatrixBase<Derived>& mat = const_cast< Eigen::MatrixBase<Derived>& >(m);
  if((Eigen::MatrixBase<Derived>::RowsAtCompileTime ==Eigen::Dynamic) 
  && (Eigen::MatrixBase<Derived>::ColsAtCompileTime ==Eigen::Dynamic))
  {
    mat.derived().resize(rows,cols);
  }
  else if(Eigen::MatrixBase<Derived>::RowsAtCompileTime ==Eigen::Dynamic) 
  {
    mat.derived().resize(rows,Eigen::NoChange);
  }
  else if(Eigen::MatrixBase<Derived>::ColsAtCompileTime ==Eigen::Dynamic) 
  {
    mat.derived().resize(Eigen::NoChange, cols);
  }
  return (mat.derived().rows() == rows) && (mat.derived().cols() == cols);
}


/**
 * RESIZE - SAFE FUNCTION CALLED ONLY IF THE MATRIX IS DYNAMICALLY CREATED AT RUNTIME
 */
template<typename Derived,
         typename std::enable_if< (Eigen::MatrixBase<Derived>::RowsAtCompileTime != Eigen::Dynamic)  &&
            (Eigen::MatrixBase<Derived>::ColsAtCompileTime != Eigen::Dynamic),
              int>::type >
inline bool resize(Eigen::MatrixBase<Derived> const & /*m*/, int rows, int cols)
{
  return Eigen::MatrixBase<Derived>::RowsAtCompileTime == rows 
      && Eigen::MatrixBase<Derived>::ColsAtCompileTime == cols;
}

/**
 * RESIZE - SAFE FUNCTION CALLED ONLY IF THE MATRIX IS DYNAMICALLY CREATED AT RUNTIME
 */
inline bool resize(const double& m, int rows, int cols)
{
  UNUSED(m);
  return rows==1 && cols ==1;
}

template<typename T>
inline bool resize(std::vector<T>& m, int rows, int cols)
{
  if(!rows || (cols!= 1))
  {
    return false;
  }
  m.resize(rows);
  return true;
}

template<typename T>
inline bool resize(std::vector<T>& m1, const std::vector<T>& m2)
{
  m1.resize(m2.size());
  return true;
}

template<typename T>
inline bool resize(T& /*m1*/, const T& /*m2*/)
{
  return true;
}


/**
 * RESIZE - SAFE FUNCTION CALLED ONLY IF THE MATRIX IS DYNAMICALLY CREATED AT RUNTIME
 */
template<typename Derived,
        typename std::enable_if< (Eigen::MatrixBase<Derived>::RowsAtCompileTime == Eigen::Dynamic) 
                        ||(Eigen::MatrixBase<Derived>::ColsAtCompileTime == Eigen::Dynamic) 
                        , int>::type >
inline bool checkInputDim(const std::string& id, const Eigen::MatrixBase<Derived>& m, int rows, int cols, std::string& error)
{
  if((m.rows()!= rows)||(m.cols()!= cols))
  {
    error += std::string(__PRETTY_FUNCTION__) + ":" + std::to_string(__LINE__) + ":\n";
    error +=    id + " dimension mismatch."
        " Object size: " + std::to_string(m.rows()) + "x" + std::to_string(m.cols()) + ","
        " Expected size: " + std::to_string(rows) + "x" + std::to_string(cols) + "";
    return false;
  }
  return true;
}


/**
 * CHECK - SKIP, SINCE THE DIMENSION IS CHECKED AT COMPILE TIME
 */
template<typename Derived,
        typename std::enable_if< (Eigen::MatrixBase<Derived>::RowsAtCompileTime != Eigen::Dynamic) 
                        && (Eigen::MatrixBase<Derived>::ColsAtCompileTime != Eigen::Dynamic) 
                        , int>::type >
inline bool checkInputDim(const std::string& id, const Eigen::MatrixBase<Derived>& m, int rows, int cols, std::string& error)
{
  UNUSED(id);
  UNUSED(m);
  UNUSED(error);
  return Eigen::MatrixBase<Derived>::RowsAtCompileTime == rows 
      && Eigen::MatrixBase<Derived>::ColsAtCompileTime == cols;
}

inline bool checkInputDim(const std::string& id, const double& m, int rows, int cols, std::string& error)
{
  UNUSED(error); UNUSED(m); UNUSED(id);
  return rows == 1 && cols == 1;
}

template<typename Derived>
inline void checkInputDimAndThrowEx(const std::string& id, const Eigen::MatrixBase<Derived>& m, int rows, int cols)
{
  std::string error = std::string(__PRETTY_FUNCTION__) + ":" + std::to_string(__LINE__) + ":\n";
  if(!checkInputDim(id,m,rows,cols,error))
  {
    throw std::runtime_error(error.c_str());
  }
}

inline void checkInputDimAndThrowEx(const std::string& id, const double& m, int rows, int cols)
{
  UNUSED(m);
  if(rows !=1 && cols !=1)
  {
    throw std::runtime_error((id + ": expected a 1x1 (double) while a matrix has been assigned").c_str());
  }
}

// double, double
inline bool copy(double& lhs, const double& rhs)
{
  lhs = rhs;
  return true;
}


// double, double
inline bool copy(double& lhs, const std::vector<double>& rhs)
{
  if(rhs.size()!=1)
    return false;
  lhs = rhs.front();
  return true;
}

// double, double
inline bool copy(std::vector<double>& lhs, const double& rhs)
{
  if(lhs.size()!=1)
    return false;
  lhs.front() = rhs;
  return true;
}

// double, matrix
template<typename Derived> 
inline bool copy(double& lhs, const Eigen::MatrixBase<Derived>& rhs)
{
  if(rhs.rows()!=1 || rhs.cols()!=1)
  {
    return false;
  }
  lhs = rhs(0,0);
  return true;
}

// matrix, matrix
template<typename Derived, typename OtherDerived>
inline bool copy(Eigen::MatrixBase<Derived> & lhs, 
          const Eigen::MatrixBase<OtherDerived>& rhs)
{
  if(!cnr::param::core::resize(lhs, rhs.rows(),  rhs.cols()))
    return false;
  lhs = rhs;
  return true;
}

// matrix, double
template<typename Derived> 
inline bool copy(Eigen::MatrixBase<Derived>& lhs, const double& rhs)
{
  lhs.setConstant(rhs);
  return true;
}

// matrix, double
template<typename Derived>
inline bool copy(Eigen::MatrixBase<Derived>& lhs, const std::vector<double>& rhs)
{
  if(!cnr::param::core::resize(lhs, int(rhs.size()),  1))
    return false;
  for(int i=0;i<lhs.rows();i++)
    lhs(i) = rhs.at(size_t(i));
  return true;
}

// matrix, double
template<typename Derived>
inline bool copy(std::vector<double>& lhs, const Eigen::MatrixBase<Derived>& rhs)
{
  if(int(lhs.size())!=rhs.rows())
  {
    lhs.resize(rhs.rows());
  }
  for(int i=0;i<rhs.rows();i++)
    lhs.at(size_t(i)) = rhs(i);
  return true;
}




inline double* data(double& v)
{
  return &v;
}

template<typename Derived>
inline typename Derived::Scalar* data(Eigen::MatrixBase<Derived>& m)
{
  Eigen::Ref<Eigen::Matrix<
    typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> 
            > _m(m);
  return _m.data();
}

inline const double* data(const double& v)
{
  return &v;
}

template<typename Derived>
inline const typename Derived::Scalar* data(const Eigen::MatrixBase<Derived>& m)
{
  return m.derived().data();
}

template<typename T>
inline const double& at(const T& v, const int& i, const int& j)
{
  if((i!=0)||(j!=0))
    throw std::invalid_argument("Input is double, while the index is greater than 0");
  return v;
}

template<typename T, typename Derived>
inline const T& at(const Eigen::MatrixBase<Derived>& m, const int& i, const int& j)
{
  return m(i,j);
}

template<typename T>
inline T& at(T& v, const int& i, const int& j)
{
  if((i!=0)||(j!=0))
    throw std::invalid_argument("Input is double, while the index is greater than 0");
  return v;
}

template<typename T, typename Derived>
inline T& at(Eigen::MatrixBase<Derived>& m, const int& i, const int& j)
{
  return m(i,j);
}

inline double norm(const double& m)
{
  return std::fabs(m);
}

template<typename Derived>
inline double norm(const Eigen::MatrixBase<Derived>& m)
{
  return m.norm();
}

inline double normalized(const double& m)
{
  UNUSED(m);
  return 1.0;
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
  normalized(const Eigen::MatrixBase<Derived>& m)
{
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> ret;
  ret = m.normalized();
  return ret;
}

template<typename T>
inline int size(const T& m)
{
  UNUSED(m);
  return 1;
}

template<typename Derived>
inline int size(const Eigen::MatrixBase<Derived>& m)
{
  return m.size();
}

template<typename Derived>
inline int size(const std::vector<Derived>& m)
{
  return m.size();
}

template<typename Derived>
inline int size(const std::vector<std::vector<Derived>>& m)
{
  return m.size() * m.front().size();
}


template<typename T>
inline int rows(const T& m)
{
  UNUSED(m);
  return 1;
}

template<typename T>
inline int rows(const std::vector<T>& m)
{
  return static_cast<int>(m.size());
}

template<typename T>
inline int rows(const std::vector<std::vector<T>>& m)
{
  return static_cast<int>(m.size());
}

template<typename Derived>
inline int rows(const Eigen::MatrixBase<Derived>& m)
{
  return m.rows();
}


inline int rank(const double& m)
{
  UNUSED(m);
  return 1;
}

template<typename Derived>
inline int rank(const Eigen::MatrixBase<Derived>& m)
{
  Eigen::FullPivLU<Derived> lu_decomp(m);
  return lu_decomp.rank();
}

template<typename T>
inline int cols(const T& m)
{
  UNUSED(m);
  return 1;
}


template<typename T>
inline int cols(const std::vector<T>& m)
{
  return static_cast<int>(m.size());
}

template<typename T>
inline int cols(const std::vector<std::vector<T>>& m)
{
  return static_cast<int>(m.front().size());
}

template<typename Derived>
inline int cols(const Eigen::MatrixBase<Derived>& m)
{
  return m.cols();
}

inline double& block(double& m, Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  UNUSED(startRow); UNUSED(startCol); UNUSED(blockRows); UNUSED(blockCols); 
  return m;
}

inline const double& block(const double& m, Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  UNUSED(startRow); UNUSED(startCol); UNUSED(blockRows); UNUSED(blockCols); 
  return m;
}

template<typename Derived>
inline Eigen::Block<Derived> block(Eigen::MatrixBase<Derived>& m, 
                          Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  return Eigen::Block<Derived>(m.derived(),startRow, startCol, blockRows, blockCols);
}
 
template<typename Derived>
inline const Eigen::Block<Derived> block(const Eigen::MatrixBase<Derived>& m, 
                          Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  return Eigen::Block<Derived>(m.derived(),startRow, startCol, blockRows, blockCols);
}
//================================================================
//
//================================================================
inline bool copy_to_block(double& lhs, const double& rhs, 
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  lhs = rhs;
  bool ret = startRow==0 && startCol==0 && blockRows==1 && blockCols==1;
  if(!ret)
  {
    std::cerr << "Error: " << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": arguments " 
              <<  startRow <<","<< startCol <<","<< blockRows <<","<< blockCols << std::endl;
  }
  return ret;
}

template<typename LHS, typename RHS>
inline bool copy_to_block(Eigen::MatrixBase<LHS>& lhs, const Eigen::MatrixBase<RHS>& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  if((blockRows == rhs.rows() && blockCols == rhs.cols() )
  && (startRow + blockRows <= lhs.rows() && startCol + blockCols <= lhs.cols() ))
  {
    lhs.block(startRow, startCol, blockRows, blockCols) = rhs;
    return true;
  }
  std::cerr << "Error: " << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": arguments" 
              <<  startRow <<","<< startCol <<","<< blockRows <<","<< blockCols << std::endl;
  return false;
}

template<typename Derived>
inline bool copy_to_block(Eigen::MatrixBase<Derived>& lhs, const double& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  if(startRow + blockRows <= lhs.rows() && startCol + blockCols <= lhs.cols() )
  {
    lhs.block(startRow, startCol, blockRows, blockCols).setConstant(rhs);
    return true;
  }
  std::cerr << "Error: " << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": arguments" 
              <<  startRow <<","<< startCol <<","<< blockRows <<","<< blockCols << std::endl;
  return false;
}

template<typename Derived>
inline bool copy_to_block(double& lhs, const Eigen::MatrixBase<Derived>& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  if((blockRows==1 && blockCols==1)
  && (startRow + blockRows <= rhs.rows() && startCol + blockCols <= rhs.cols()))
  {
    lhs = rhs(startRow,startCol);
    return true;
  }
  return false;
}

inline bool copy_to_block(double& lhs, const double& rhs,
              Eigen::Index startRow, Eigen::Index blockRows)
{
  return copy_to_block(lhs, rhs, startRow, 0, blockRows, 1);
}

template<typename LHS, typename RHS>
inline bool copy_to_block(Eigen::MatrixBase<LHS>& lhs, const Eigen::MatrixBase<RHS>& rhs,
              Eigen::Index startRow, Eigen::Index blockRows)
{
  return copy_to_block(lhs, rhs, startRow, 0, blockRows, 1);
}

template<typename Derived>
inline bool copy_to_block(Eigen::MatrixBase<Derived>& lhs, const double& rhs,
              Eigen::Index startRow, Eigen::Index blockRows)
{
  return copy_to_block(lhs, rhs, startRow, 0, blockRows, 1);
}

template<typename Derived>
inline bool copy_to_block(double& lhs, const Eigen::MatrixBase<Derived>& rhs,
              Eigen::Index startRow, Eigen::Index blockRows)
{
  return copy_to_block(lhs, rhs, startRow, 0, blockRows, 1);
}


//================================================================
//
//================================================================
inline bool copy_from_block(double& lhs, const double& rhs, 
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  lhs = rhs;
  bool ret = startRow==0 && startCol==0 && blockRows==1 && blockCols==1;
  if(!ret)
  {
    std::cerr << "Error: " << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": arguments" 
              <<  startRow <<","<< startCol <<","<< blockRows <<","<< blockCols << std::endl;
  }
  return ret;
}

template<typename LHS, typename RHS>
inline bool copy_from_block(Eigen::MatrixBase<LHS>& lhs, const Eigen::MatrixBase<RHS>& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  if((blockRows == lhs.rows() && blockCols == lhs.cols() )
  && (startRow + blockRows <= rhs.rows() && startCol + blockCols <= rhs.cols() ))
  {
    lhs = rhs.block(startRow, startCol, blockRows, blockCols);
    return true;
  }
  std::cerr << "Error: " << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": arguments" 
              <<  startRow <<","<< startCol <<","<< blockRows <<","<< blockCols << std::endl;
  return false;
}

template<typename Derived>
inline bool copy_from_block(Eigen::MatrixBase<Derived>& lhs, const double& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  if(startRow==0 && startCol==0 && blockRows==1 && blockCols==1)
  {
    lhs.setConstant(rhs);
    return true;
  }
  std::cerr << "Error: " << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": arguments" 
              <<  startRow <<","<< startCol <<","<< blockRows <<","<< blockCols << std::endl;
  return false;
}

template<typename Derived>
inline bool copy_from_block(double& lhs, const Eigen::MatrixBase<Derived>& rhs,
              Eigen::Index startRow, Eigen::Index startCol, Eigen::Index blockRows, Eigen::Index blockCols)
{
  if((blockRows==1 && blockCols==1)
  && (startRow + blockRows <= rhs.rows() && startCol + blockCols <= rhs.cols()))
  {
    lhs = rhs(startRow,startCol);
    return true;
  }
  return false;
}




//! A x = b --> x = b /A
inline bool solve(double& x, const double& A, const double& b )
{
  x = b / A;
  return true;
}

//! A x = b --> x = b /A
template<typename Derived>
inline bool solve(Eigen::MatrixBase<Derived>& x, const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
  Eigen::VectorXd _x;
  if(A.rows()==A.cols() && A.rows()==1 && b.rows()==1)
  {
    double __x;
    if(solve(__x, A(0,0), b(0)))
    {
      _x.resize(1);
      _x(0) = __x;
      return true;
    }
    else
    {
      return false;
    }
  }
  else   if((A.rows()==A.cols()) && (cnr::param::core::rank(A) == A.rows()))
  {
    _x = A.fullPivLu().solve(b);
  }
  else if(cnr::param::core::rank(A) == std::min(A.rows(),A.cols()))
  {
    _x = A.colPivHouseholderQr().solve(b);
  }
  else
  {
    _x = A.jacobiSvd().solve(b);
  }

  return (cnr::param::core::copy_to_block(x, _x, 0, 0, _x.rows(),1) );
}


template<typename Derived>
inline bool solve(Eigen::MatrixBase<Derived>& x, const Eigen::MatrixXd& A, const double& b)
{
  Eigen::VectorXd _b(1); _b << b;
  Eigen::VectorXd _x;
  if(A.rows()==A.cols() && A.rows()==1)
  {
    double __x;
    if(solve(__x, A(0,0), _b(0)))
    {
      _x.resize(1);
      _x(0) = __x;
      return true;
    }
    else
    {
      return false;
    }
  }
  else if((A.rows()==A.cols()) && (cnr::param::core::rank(A) == A.rows()))
  {
    _x = A.fullPivLu().solve(_b);
  }
  else if(cnr::param::core::rank(A) == std::min(A.rows(),A.cols()))
  {
    _x = A.colPivHouseholderQr().solve(_b);
  }
  else
  {
    _x = A.jacobiSvd().solve(_b);
  }

  return (cnr::param::core::copy_to_block(x, _x, 0, 0, _x.rows(),1) );
}


inline void setConstant(double& m, const double& v)
{
  m = v;
}

template<typename Derived>
inline void setConstant(Eigen::MatrixBase<Derived>& m, const double& v)
{
  m.setConstant(v);
}

inline void setDiagonal(double& m, const double& v)
{
  m = v;
}

inline void setDiagonal(double& m, const std::vector<double>& v)
{
  if(v.size()!=1) throw std::runtime_error("setDiagonal failed! The input vector has the size greater than 1!");
  m = v.front();
}

template<typename D>
inline void setDiagonal(double& m, const Eigen::MatrixBase<D>& v)
{
  if(v.size()!=1) throw std::runtime_error("setDiagonal failed! The input vector has the size greater than 1!");
  m = v(0,0);
}

template<typename Derived>
inline void setDiagonal(Eigen::MatrixBase<Derived>& m, const double& v)
{
  if(rows(m) != cols(m)) throw std::runtime_error("The input matrix is not squared!");
  for(int i=0;i<rows(m);i++)
      m(i,i)=v;
}

template<typename D>
inline void setDiagonal(Eigen::MatrixBase<D>& m, const std::vector<double>& v)
{
  if(static_cast<int>(v.size())!=std::min(m.rows(), m.cols()))
    throw std::runtime_error("setDiagonal failed! The input vector has the size greater than the matrix!");
  for(size_t i=0;i<v.size();i++)
  {
    m(static_cast<int>(i),static_cast<int>(i)) = v.at(i);
  }
}


inline void saturate(double& v, const double& min, const double& max)
{
  v = std::max(std::min(v, max), min);
}

template<typename Derived>
inline void saturate(Eigen::MatrixBase<Derived>& m, const double& min, const double& max)
{
  for(int i=0;i<m.rows();i++)
  {
    for(int j=0;j<m.cols();j++)
    {
      m(i,j) = std::max(std::min(m(i,j), max), min);
    }
  }
}

template<typename Derived>
inline void saturate(Eigen::MatrixBase<Derived>& m, 
                     const Eigen::MatrixBase<Derived>& min,
                     const Eigen::MatrixBase<Derived>& max)
{
  assert(m.rows() == min.rows());
  assert(m.rows() == max.rows());
  assert(m.cols() == min.cols());
  assert(m.cols() == max.cols());

  for(int i=0;i<m.rows();i++)
  {
    for(int j=0;j<m.cols();j++)
    {
      m(i,j) = std::max(std::min(m(i,j), max(i,j)), min(i,j));
    }
  }
}

inline double dot(const double& m1, const double& m2)
{
  return m1 * m2;
}

template<typename Derived, typename OtherDerived>
inline double dot(const Eigen::MatrixBase<Derived>& m1, const Eigen::MatrixBase<OtherDerived>& m2)
{
  return m1.dot(m2);
}



//============
template<typename MatType>
using PseudoInverseType = Eigen::Matrix<typename MatType::Scalar, MatType::ColsAtCompileTime, MatType::RowsAtCompileTime>;

template<typename MatType>
PseudoInverseType<MatType> pseudoInverse(const MatType &a, double epsilon)
{
	using WorkingMatType = Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, Eigen::Dynamic, 0,
																			 MatType::MaxRowsAtCompileTime, MatType::MaxColsAtCompileTime>;
	Eigen::BDCSVD<WorkingMatType> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
	svd.setThreshold(epsilon*std::max(a.cols(), a.rows()));
	Eigen::Index rank = svd.rank();
	Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, MatType::RowsAtCompileTime,
								0, Eigen::BDCSVD<WorkingMatType>::MaxDiagSizeAtCompileTime, MatType::MaxRowsAtCompileTime>
		tmp = svd.matrixU().leftCols(rank).adjoint();
	tmp = svd.singularValues().head(rank).asDiagonal().inverse() * tmp;
	return svd.matrixV().leftCols(rank) * tmp;
}

inline double div(const double& a, const double b)
{
  return a/b;
}

template<typename AType, typename BType> //  C = B^-1 x A  (bc x ac ) = (bc x br)  x (ar x ac )
using DivType = Eigen::Matrix<typename AType::Scalar, BType::ColsAtCompileTime, AType::ColsAtCompileTime>;

template<typename AType, typename BType>
inline DivType<AType, BType> div(const AType& a, const BType& b)
{
  DivType<AType, BType> ret;
  PseudoInverseType<BType> binv = pseudoInverse(b);
  ret = binv * a;
  return ret;
}


}
}
}

#endif  /* INCLUDE_CNR_PARAM_IMPL_EIGEN_IMPL */
