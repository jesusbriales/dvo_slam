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
    return 0.0f;
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
  lowerThres = 0.0f;

  // Set maximum selection probability
  // Set heuristically, see Oreshkin's paper
  probMax = std::min( 1.0f, 10.0f * samplingRatio );

  // Solve slope to fulfill the sampling ratio constraint
  // Parameterize slope with horizontal interval of the ramp (for fixed height)
  // which adjusts better to the non-linearity of the problem
  float numOfSamples = samplingRatio * utilities.size();
  float alpha = numOfSamples / boost::accumulate(utilities,0.0f);
  float minValue = 0.0f;
  float maxValue = 2.0f * (probMax / alpha); // Set a non-too-high value to reduce number of bisection steps
  float toleranceNumOfSamples = 1.0f;
  ProbRampFunctor fun( *this, utilities );
  // Check if alpha is close enough:
  if( std::abs(fun(probMax / alpha)) < toleranceNumOfSamples )
    slope = alpha;
  else
    slope = probMax / bisect( fun, minValue, maxValue, toleranceNumOfSamples );
}

// Operator for the step map,
// given by a threshold and a maximum value
float StepMap::operator ()( Utility point_utility ) const
{
  if( point_utility > lowerThres )
  {
    return probMax;
  }
  else
    return 0.0f;
}

void StepMap::setup(const UtilityVector& utilities, float ratio)
{
  samplingRatio = ratio;

  // Set lower threshold with a 10% margin wrt the desired ratio
  float prefilterRatio = std::min( 1.0f, ratio + 0.1f );
  // Set the threshold no lower that at a 20% of prefiltering (original value by Dellaert)
  if(prefilterRatio<0.2)
    prefilterRatio = 0.2f;

  // Set map threshold in the utility value for the desired prefilter ratio
  lowerThres = nth_orderStatistic( utilities, std::floor( prefilterRatio * utilities.size() ) );

  // Compute the expected num of samples with the current configuration, probMax of 1
  // Use a simple proportional rule (works for this simple profile)
  probMax = 1.0f;
  probMax = ratio * float(utilities.size()) / expectedNumOfSamples( utilities );

  // Debug:
  // Check that the expected num of samples constraint is fulfilled with the new probMax
//  float checkConstraint = samplingRatioConstraint( utilities );
//  std::cout << "Constraint value (should be 0): " << checkConstraint << std::endl;
}

const char* UtilityMaps::str(enum_t type)
{
  switch(type)
  {
    case UtilityMaps::Id:
      return "Id";
    case UtilityMaps::Ramp:
      return "Probabilistic ramp";
    case UtilityMaps::Step:
      return "Step function";
    default:
      break;
  }
  assert(false && "Unknown utility map type!");

  return "";
}

pUtilityMap* UtilityMaps::get(UtilityMaps::enum_t type, size_t num)
{
  static std::vector<IdMap> id;
  static std::vector<ProbRampMap> probRamp;
  static std::vector<StepMap> probStep;

  static std::vector<pUtilityMap> pointers(num);

  switch(type)
  {
  case UtilityMaps::Id:
  {
    id.resize(num);
    for(size_t i=0; i<num; ++i)
      pointers[i] = (UtilityMap*)(&id[i]);
    return pointers.data();
  }
  case UtilityMaps::Ramp:
  {
    probRamp.resize(num);
    for(size_t i=0; i<num; ++i)
      pointers[i] = (UtilityMap*)(&probRamp[i]);
    return pointers.data();
  }
  case UtilityMaps::Step:
  {
    probStep.resize(num);
    for(size_t i=0; i<num; ++i)
      pointers[i] = (UtilityMap*)(&probStep[i]);
    return pointers.data();
  }
  default:
    break;
  }
  assert(false && "Unknown utility map type!");

  return 0;
}

} /* namespace selection */
} /* namespace dvo */
