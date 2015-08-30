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

// Define generic functor
struct Functor
{
  virtual float operator() ( float paramValue ) const = 0;
};
struct MapFunctor : Functor
{
public:
  MapFunctor ( UtilityMap* mapPtr ) : mapPtr( mapPtr ) {}
public:
  UtilityMap* mapPtr;
};

class ProbabilityMap : public UtilityMap
{
public:
  ProbabilityMap( const UtilityVector& utilities, float samplingRatio ):
    UtilityMap(utilities,samplingRatio) {}
  inline float expectedNumOfSamples() const;
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

protected:
  float lowerThres_, slope_, probMax_; // Map parameters
public:
  inline void slope( float in )
  {
    slope_ = in;
  }

};

struct UtilityMapPSPFFunctor
{
public:
  UtilityMapPSPFFunctor (UtilityMapPSPF& map): map(map) {}
  float operator() ( float paramValue )
  {
    map.slope(paramValue);
    return map.samplingRatioConstraint();
  }
private:
  UtilityMapPSPF map;
};

// Implementation of inline functions in header for visibility
inline float ProbabilityMap::expectedNumOfSamples() const
{
  using namespace boost::accumulators;

  accumulator_set<float, stats<tag::sum> > acc;
  for(UtilityVector::const_iterator util_it = utilities_.begin(); util_it!=utilities_.end(); ++util_it)
  {
    acc( this->operator ()(*util_it) );
  }
  return sum( acc );
}

} /* namespace selection */
} /* namespace dvo */

#endif // UTILITYMAP_H
