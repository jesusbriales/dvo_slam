#ifndef UTILITYMAP_H
#define UTILITYMAP_H

#include <vector>

#include <boost/accumulators/accumulators.hpp> // For incremental sum
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/sum.hpp>

namespace dvo
{
namespace selection
{

typedef float Utility;
typedef std::vector<Utility> UtilityVector;
typedef UtilityVector::iterator UtilityIterator;

// UtilityMap is a functor which receives a utility value
// and returns a related value (could be the corresponding probability,
// a thresholded version of the input, or some other magnitude)
class UtilityMap
{
public:
  UtilityMap( const UtilityVector&, float samplingRatio );
  virtual float operator() ( Utility ) const = 0; // Function to apply by the functor
  virtual void solveParameters() {} // Method that sets parameters in the map

protected:
  UtilityVector utilities_;
  float samplingRatio_;
  float numOfSamples_;
};

class ProbabilityMap : public UtilityMap
{
public:
  ProbabilityMap( const UtilityVector& utilities, float samplingRatio ):
    UtilityMap(utilities,samplingRatio) {}

  inline float expectedNumOfSamples() const
  {
    using namespace boost::accumulators;

    accumulator_set<float, stats<tag::sum> > acc;
    for(UtilityVector::const_iterator util_it = utilities_.begin(); util_it!=utilities_.end(); ++util_it)
    {
      acc( this->operator ()(*util_it) );
    }
    return sum( acc );
  }

  inline float samplingRatioConstraint()
  {
    return expectedNumOfSamples() - numOfSamples_;
  }
};

class UtilityMapPSPF : public ProbabilityMap
{
public:
  UtilityMapPSPF( const UtilityVector& utilities, float samplingRatio ):
    ProbabilityMap(utilities,samplingRatio) {}
  virtual float operator() ( Utility ) const;
  virtual void solveParameters();

public:
  float lowerThres, slope, probMax; // Map parameters
};

} /* namespace selection */
} /* namespace dvo */

#endif // UTILITYMAP_H
