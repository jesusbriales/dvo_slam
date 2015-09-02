#ifndef UTILITYMAP_H
#define UTILITYMAP_H

#include <vector>

#include <boost/accumulators/accumulators.hpp> // For incremental sum
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/sum.hpp>

#include <time.h> /* time, for srand */

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
  // Function associated to the map profile
  virtual float operator() ( Utility ) const = 0;

  // Method that sets parameters in the map, if any
  virtual void setup(const UtilityVector&, float ratio) = 0;
};

class IdMap : public UtilityMap
{
  inline float operator() ( Utility input ) const
  {
	return input;
  }
  inline void setup(const UtilityVector&, float) { }
};

class ProbabilityMap : public UtilityMap
{
public:
  ProbabilityMap( )
  {
    // set the rand seed depending on the current time
    // this is necessary for the experiments to evaluate trends
    srand( time(NULL) );
  }

  inline float expectedNumOfSamples( const UtilityVector& utilities ) const
  {
    using namespace boost::accumulators;

    accumulator_set<float, stats<tag::sum> > acc;
	for(UtilityVector::const_iterator util_it = utilities.begin(); util_it!=utilities.end(); ++util_it)
    {
      acc( this->operator ()(*util_it) );
    }
    return sum( acc );
  }

  inline float samplingRatioConstraint( const UtilityVector& utilities )
  {
	return expectedNumOfSamples( utilities ) - samplingRatio * utilities.size();
  }

public:
  float samplingRatio;
};

class ProbRampMap : public ProbabilityMap
{
public:
  virtual float operator() ( Utility ) const;
  void setup(const UtilityVector&, float ratio);

public:
  float lowerThres, slope, probMax; // Map parameters
};

struct UtilityMaps {
  typedef enum {
	Id,
	Ramp,
	Step
	// don't forget to add to dynamic reconfigure!
  } enum_t;

  static const char* str(enum_t type);

  static UtilityMap* get(enum_t type);
};

} /* namespace selection */
} /* namespace dvo */

#endif // UTILITYMAP_H
