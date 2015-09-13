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
      const Eigen::Matrix<double, 6, 6>& information, /* necessary for some cases */
      UtilityVector& utilities) = 0;
};

// TODO: Implement with traits instead to change only the behaviour
// inside the for loop

class JacMagCalculator : public UtilityCalculator
{
  void compute(
      const PointIt& first_point,
      const PointIt& last_point,
      const Eigen::Matrix<double, 6, 6>& information, /* necessary for some cases */
      UtilityVector& utilities)
  {
    UtilityIterator util_it = utilities.begin();
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
      const Eigen::Matrix<double, 6, 6>& information, /* necessary for some cases */
      UtilityVector& utilities)
  {
    UtilityIterator util_it = utilities.begin();
    for(PointIt point_it = first_point;
        point_it != last_point; ++point_it, ++util_it)
    {
      // compute utility as the squared norm of the photometric gradient
      *util_it = point_it->getIntensityDerivativeVec2f().squaredNorm();
    }
  }
};

class TrVarCalculator : public UtilityCalculator
{
  void compute(
      const PointIt& first_point,
      const PointIt& last_point,
      const Eigen::Matrix<double, 6, 6>& information, /* necessary for some cases */
      UtilityVector& utilities)
  {
    // Compute covariance
    Eigen::Matrix<double, 6, 6> cov = information.inverse();
    Eigen::Matrix<double, 6, 1> jac;

    UtilityIterator util_it = utilities.begin();
    for(PointIt point_it = first_point;
        point_it != last_point; ++point_it, ++util_it)
    {
      // compute utility as the decrement in the trace of the covariance
      jac = point_it->getIntensityJacobianVec6f().cast<double>(); // Necessary explicit cast in Eigen from float to double
      *util_it = jac.dot( cov * cov * jac ) / (1.0 + jac.dot(cov * jac));
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
};

} /* namespace selection */
} /* namespace dvo */

#endif // UTILITY_H
