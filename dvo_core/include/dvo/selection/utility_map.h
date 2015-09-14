#ifndef UTILITYMAP_H
#define UTILITYMAP_H

#include <vector>

#include <boost/accumulators/accumulators.hpp> // For incremental sum
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/sum.hpp>

#include <time.h> /* time, for srand */
#include <algorithm> /* nth_element, for QuickSelect */

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

  // Gives the expected difference (in number of samples, NOT ratio!)
  // between the desired number of samples and the expected number of samples
  // with the current configuration map
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

class StepMap : public ProbabilityMap
{
public:
  virtual float operator() ( Utility ) const;
  void setup(const UtilityVector&, float ratio);

public:
  float lowerThres, probMax; // Map parameters
};

typedef UtilityMap* pUtilityMap;

struct UtilityMaps {
  typedef enum {
	Id,
	Ramp,
	Step
	// don't forget to add to dynamic reconfigure!
  } enum_t;

  static const char* str(enum_t type);

  static pUtilityMap* get(enum_t type, size_t num = 1);
};



// Define some util functions

// Compute the value in a list (given as a vector)
// which would have the nth position in the ordered list
// Implemented using std::nth_element, equivalent to QuickSelect
template <class T>
float nth_orderStatistic( const std::vector<T>& list, size_t n )
{
  // Create a copy of the list to keep the original unchanged
  std::vector<T> copyOfList;
  copyOfList.resize( list.size() );
  std::copy( list.begin(), list.end(), copyOfList.begin() );

  typename std::vector<T>::iterator it_nth;
  it_nth = copyOfList.begin() + n;
  std::nth_element( copyOfList.begin(), it_nth, copyOfList.end() );
  return *it_nth;
}

} /* namespace selection */
} /* namespace dvo */

#endif // UTILITYMAP_H
