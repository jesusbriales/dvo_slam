#ifndef UTILITY_H
#define UTILITY_H

#include <vector>

#include <dvo/core/rgbd_image.h> /* for iterator type */

#include <Eigen/Core>
#include <Eigen/Dense> /* for inverse() */

namespace dvo
{
namespace selection
{

typedef float Utility;
typedef std::vector<Utility> UtilityVector;
typedef UtilityVector::iterator UtilityIterator;

typedef std::vector<core::PointWithIntensityAndDepth, Eigen::aligned_allocator<core::PointWithIntensityAndDepth> >::iterator PointIt;

class UtilityCalculator
{
public:
  virtual void compute(
      const PointIt& first_point,
      const PointIt& last_point,
      const Eigen::Matrix<double, 6, 6>& information /* necessary for some cases */
      ) = 0;

  std::vector<UtilityVector> utilities;
};

// TODO: Implement with traits instead to change only the behaviour
// inside the for loop

class JacMagCalculator : public UtilityCalculator
{
  void compute(
      const PointIt& first_point,
      const PointIt& last_point,
      const Eigen::Matrix<double, 6, 6>& information /* necessary for some cases */
      )
  {
    // Set dimension of the utilities in the storage
    utilities.resize(1);
    utilities[0].resize( last_point - first_point );

    UtilityIterator util_it = utilities[0].begin();
    for(PointIt point_it = first_point;
        point_it != last_point; ++point_it, ++util_it)
    {
      // compute utility as the squared norm of the photometric jacobian
      *util_it = point_it->getIntensityJacobianVec6f().squaredNorm();
    }
  }
};

class GradMagCalculator : public UtilityCalculator
{
  void compute(
      const PointIt& first_point,
      const PointIt& last_point,
      const Eigen::Matrix<double, 6, 6>& information /* necessary for some cases */
      )
  {
    // Set dimension of the utilities in the storage
    utilities.resize(1);
    utilities[0].resize( last_point - first_point );

    UtilityIterator util_it = utilities[0].begin();
    for(PointIt point_it = first_point;
        point_it != last_point; ++point_it, ++util_it)
    {
      // compute utility as the squared norm of the photometric gradient
      *util_it = point_it->getIntensityDerivativeVec2f().squaredNorm();
    }
  }
};

// TODO: Create vector of vector6
class JacCalculator : public UtilityCalculator
{
  void compute(
      const PointIt& first_point,
      const PointIt& last_point,
      const Eigen::Matrix<double, 6, 6>& information /* necessary for some cases */
      )
  {
    // Set dimension of the utilities in the storage
    utilities.resize(6);
    for(size_t i=0; i<6; i++)
      utilities[i].resize( last_point - first_point );

    size_t utilIdx = 0;
    Eigen::Matrix<float, 6, 1> jac;
    for(PointIt point_it = first_point;
        point_it != last_point; ++point_it, ++utilIdx)
    {
      // store jacobian images
      jac = point_it->getIntensityJacobianVec6f();
      utilities[0][utilIdx] = std::abs(jac[0]);
      utilities[1][utilIdx] = std::abs(jac[1]);
      utilities[2][utilIdx] = std::abs(jac[2]);
      utilities[3][utilIdx] = std::abs(jac[3]);
      utilities[4][utilIdx] = std::abs(jac[4]);
      utilities[5][utilIdx] = std::abs(jac[5]);
    }
  }
};

class TrVarCalculator : public UtilityCalculator
{
  void compute(
      const PointIt& first_point,
      const PointIt& last_point,
      const Eigen::Matrix<double, 6, 6>& information /* necessary for some cases */
      )
  {
    // Compute covariance
    Eigen::Matrix<float, 6, 6> cov = information.inverse().cast<float>();
    Eigen::Matrix<float, 6, 6> cov2 = cov * cov;
    Eigen::Matrix<float, 6, 1> jac;

    // TODO: Cov values are very low, o(-13).
    // - May this induce numeric instability?
    // - Could be somehow normalized?

    // Set dimension of the utilities in the storage
    utilities.resize(1);
    utilities[0].resize( last_point - first_point );

    UtilityIterator util_it = utilities[0].begin();
    for(PointIt point_it = first_point;
        point_it != last_point; ++point_it, ++util_it)
    {
      // compute utility as the decrement in the trace of the covariance
      jac = point_it->getIntensityJacobianVec6f(); // Necessary explicit cast in Eigen from float to double
      *util_it = jac.dot( cov2 * jac ) / (1.0 + jac.dot(cov * jac));
    }
  }
};

struct Utilities {
  typedef enum {
  JacMag,
  GradMag,
  Jac,
  TrVar
  // don't forget to add to dynamic reconfigure!
  } enum_t;

  static const char* str(enum_t type);

  static UtilityCalculator* get(enum_t type);
  static size_t dim(enum_t type);
};

} /* namespace selection */
} /* namespace dvo */

#endif // UTILITY_H
