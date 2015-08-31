#include <dvo/selection/utility_map.h>

#include <cmath> // For std::abs
#include <boost/range/numeric.hpp> // For boost::accumulate
#include <boost/range/algorithm.hpp> // For boost::max_element
#include <boost/accumulators/accumulators.hpp> // For incremental sum
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/sum.hpp>

namespace dvo
{
namespace selection
{

float bisect( UtilityMapPSPFFunctor fun, float a, float b, float tolerance );

UtilityMap::UtilityMap(const UtilityVector& utilities, float samplingRatio ) :
  utilities_(utilities),
  samplingRatio_(samplingRatio)
{
  numOfSamples_ = samplingRatio * (float)utilities.size();
}

float UtilityMapPSPF::operator ()( Utility point_utility ) const
{
  if( point_utility > lowerThres_ )
  {
    return std::min( probMax_, slope_ * (point_utility - lowerThres_) );
  }
  else
    return 0;
}
void UtilityMapPSPF::solveParameters()
{
  // Set lower threshold (user value for now)
  lowerThres_ = 0;

  // Set maximum selection probability
  // Set heuristically, see Oreshkin's paper
  probMax_ = std::min( 1.0f, 10.0f * samplingRatio_ );

  // Solve slope to fulfill the sampling ratio constraint
  float alpha = numOfSamples_ / boost::accumulate(utilities_,0);
  float minValue = 0.5 * alpha;
  float maxValue = 2 * alpha; // Set a non-too-high value to reduce number of bisection steps
  float toleranceNumOfSamples = 1.0f;
  slope_ = bisect( UtilityMapPSPFFunctor(*this),
                        minValue, maxValue, toleranceNumOfSamples );
}

// Numerical solvers
float bisect( UtilityMapPSPFFunctor fun, float a, float b, float tolerance )
{
  float fa,fb,fc;
  fa = fun(a); fb = fun(b);

  // check that a solution exists
  if( fa * fb > 0 )
  {
    std::cout << "There is no solution for lt" << std::endl;
  }

  float distanceThres = 2*tolerance;
  float c;
  while( std::abs(fb-fa) > distanceThres )
  {
    c = 0.5f * (a + b);
    fc = fun(c);

    if( fc * fb < 0 )
    {
        a = c;
        fa = fc;
    }
    else
    {
      b = c;
      fb = fc;
    }
  }
  return 0.5f * (a+b);
}

} /* namespace selection */
} /* namespace dvo */
