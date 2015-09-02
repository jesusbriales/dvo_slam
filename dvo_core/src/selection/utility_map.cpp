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

// Operator for the PSFP map, a piecewise function with a lower threshold,
// then a ramp and then an upper threshold
float ProbRampMap::operator ()( Utility point_utility ) const
{
  if( point_utility > lowerThres )
  {
    return std::min( probMax, slope * (point_utility - lowerThres) );
  }
  else
    return 0;
}

// Functor for setup() in ProbRampMap
struct ProbRampFunctor : public Functor
{
public:
  ProbRampFunctor ( ProbRampMap& map, const UtilityVector& utilities ) :
    map( map ), utilities( utilities ) {}
  float operator() ( float paramValue )
  {
    map.slope = map.probMax / paramValue;
    return map.samplingRatioConstraint( utilities );
  }
private:
  ProbRampMap map;
  UtilityVector utilities;
};

void ProbRampMap::setup(const UtilityVector& utilities, float ratio)
{
  samplingRatio = ratio;

  // Set lower threshold (user value for now)
  lowerThres = 0;

  // Set maximum selection probability
  // Set heuristically, see Oreshkin's paper
  probMax = std::min( 1.0f, 10.0f * samplingRatio );

  // Solve slope to fulfill the sampling ratio constraint
  // Parameterize slope with horizontal interval of the ramp (for fixed height)
  // which adjusts better to the non-linearity of the problem
  float numOfSamples = samplingRatio * utilities.size();
  float alpha = numOfSamples / boost::accumulate(utilities,0);
  float minValue = 0;
  float maxValue = 2.0f * (probMax / alpha); // Set a non-too-high value to reduce number of bisection steps
  float toleranceNumOfSamples = 1.0f;
  ProbRampFunctor fun( *this, utilities );
  // Check if alpha is close enough:
  if( std::abs(fun(probMax / alpha)) < toleranceNumOfSamples )
    slope = alpha;
  else
    slope = probMax / bisect( fun, minValue, maxValue, toleranceNumOfSamples );
}

const char* UtilityMaps::str(enum_t type)
{
  switch(type)
  {
    case UtilityMaps::Id:
      return "Id";
    case UtilityMaps::Ramp:
      return "Probabilistic ramp";
    default:
      break;
  }
  assert(false && "Unknown utility map type!");

  return "";
}

UtilityMap* UtilityMaps::get(UtilityMaps::enum_t type)
{
  static IdMap id;
  static ProbRampMap probRamp;

  switch(type)
  {
    case UtilityMaps::Id:
      return (UtilityMap*)&id;
    case UtilityMaps::Ramp:
      return (UtilityMap*)&probRamp;
    default:
      break;
  }
  assert(false && "Unknown utility map type!");

  return 0;
}

} /* namespace selection */
} /* namespace dvo */
