#ifndef SAMPLER_H
#define SAMPLER_H

#include <dvo/selection/utility_map.h>

namespace dvo
{
namespace selection
{

typedef float Probability;
typedef std::vector<Probability> ProbabilityVector;
typedef ProbabilityVector::iterator ProbabilityIterator;

class Sampler
{
public:
  virtual bool operator() ( float ) const = 0; // Function to apply by the functor
};

// Generic class for samplers which treats probabilities one-at-a-time
// This may avoid the storage of all mapped values at the same time
class SamplerWithoutStorage : public Sampler {};
// Generic class for samplers which need to take all probabilities into account
// These require the storage of all mapped values
class SamplerWithStorage : public Sampler
{
public:
  // Make necessary steps to compute temporary values
  virtual void setup( const UtilityMap& map, const UtilityVector& utilities ) = 0;
  ProbabilityVector temporaries;
};

// BernoulliSampler
// Treats the selection of a pixel as an stochastic process
// defined by a Bernoulli distribution with a certain probability
class BernoulliSampler : public SamplerWithoutStorage
{
public:
  virtual inline bool operator() (float probability) const
  {
    return probability > ((float)std::rand()/(float)(RAND_MAX/1));
  }
};

// Weighted Random Selection (WRS) without replacement
// using the exponential distribution to generate probabilities
// and Quickselect to choose the samples
// See http://stackoverflow.com/a/30226926
class WrsExponentialQuickselectSampler : public SamplerWithStorage
{
public:
  virtual void setup( const UtilityMap& map, const UtilityVector& utilities );

  virtual inline bool operator() (float temporary) const
  {
    return temporary < threshold_;
  }
protected:
  float threshold_;
};

} /* namespace selection */
} /* namespace dvo */

#endif // SAMPLER_H

