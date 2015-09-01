#include <dvo/selection/sampler.h>

#include <cmath>
#include <algorithm>

namespace dvo
{
namespace selection
{

void WrsExponentialQuickselectSampler::setup(const UtilityMap &map, const UtilityVector &utilities)
{
  // Compute exponential samples from mapped utilities and store them as temporary values
  temporaries.resize(utilities.size());
  ProbabilityIterator t_it = temporaries.begin();
  float divMaxInt = 1.0f/(float)(RAND_MAX/1);
  for(UtilityVector::const_iterator util_it = utilities.begin();
      util_it != utilities.end(); ++util_it, ++t_it)
  {
    // Explanation here:
    // https://en.wikipedia.org/wiki/Exponential_distribution#Generating_exponential_variates
    *t_it = -std::log( (float)std::rand() * divMaxInt ) / (*util_it) ;
  }

  // Compute the threshold to obtain the desired number of samples
  // For that, partially sort the vector around the n_th element
  // and finally read the new value for the n_th element
  // Copy the vector of temporaries to a copy vector, to keep original one ordered
  ProbabilityVector temporariesOrdered;
  temporariesOrdered.resize( temporaries.size() );
  std::copy( temporaries.begin(), temporaries.end(), temporariesOrdered.begin() );
  ProbabilityIterator t_nth = temporariesOrdered.begin() + std::floor(map.numOfSamples_);
  std::nth_element( temporariesOrdered.begin(), t_nth, temporariesOrdered.end() );
  threshold_ = *t_nth;
}

} /* namespace selection */
} /* namespace dvo */
