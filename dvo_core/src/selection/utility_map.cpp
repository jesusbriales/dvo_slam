#include <dvo/selection/utility_map.h>

#include <dvo/selection/solvers.h>

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

// Constructor for the base class UtilityMap
UtilityMap::UtilityMap(const UtilityVector& utilities, float samplingRatio ) :
  utilities_(utilities),
  samplingRatio_(samplingRatio)
{
  numOfSamples_ = samplingRatio * (float)utilities.size();
}

// Operator for the PSFP map, a piecewise function with a lower threshold,
// then a ramp and then an upper threshold
float UtilityMapPSPF::operator ()( Utility point_utility ) const
{
  if( point_utility > lowerThres )
  {
    return std::min( probMax, slope * (point_utility - lowerThres) );
  }
  else
    return 0;
}

// Functor for solveParameters in UtilityMapPSPF
struct PSPFFunctor : public Functor
{
public:
  PSPFFunctor ( UtilityMapPSPF& map ) : map( map ) {}
  float operator() ( float paramValue )
  {
//    map.slope(paramValue);
    map.slope = map.probMax / paramValue;
    return map.samplingRatioConstraint();
  }
private:
  UtilityMapPSPF map;
};

void UtilityMapPSPF::solveParameters()
{
  // Set lower threshold (user value for now)
  lowerThres = 0;

  // Set maximum selection probability
  // Set heuristically, see Oreshkin's paper
  probMax = std::min( 1.0f, 10.0f * samplingRatio_ );

  // Solve slope to fulfill the sampling ratio constraint
  // Parameterize slope with horizontal interval of the ramp (for fixed height)
  // which adjusts better to the non-linearity of the problem
  float alpha = numOfSamples_ / boost::accumulate(utilities_,0);
  float minValue = 0;
  float maxValue = 2.0f * (probMax / alpha); // Set a non-too-high value to reduce number of bisection steps
  float toleranceNumOfSamples = 1.0f;
  PSPFFunctor fun( *this );
  // Check if alpha is close enough:
  if( std::abs(fun(probMax / alpha)) < toleranceNumOfSamples )
    slope = alpha;
  else
    slope = probMax / bisect( fun, minValue, maxValue, toleranceNumOfSamples );
}

} /* namespace selection */
} /* namespace dvo */
