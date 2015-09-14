#ifndef SAMPLER_H
#define SAMPLER_H

#include <dvo/selection/utility_map.h>

namespace dvo
{
namespace selection
{

typedef float NumType;
typedef std::vector<NumType> NumVector;
typedef NumVector::iterator NumIterator;

class Sampler
{
public:
  // Set parameters in the sampler, if any
  virtual void setup(const UtilityMap&, const UtilityVector&, float ratio) = 0;

  // Make decision for the currently pointed point
  virtual bool makeDecision() const = 0;

  // Increment iterator to point to the next sampled point
  void pointToNext()
  {
	it_++;
  }

protected:
  NumVector parameters_;
  NumIterator it_; // Internal iterator to traverse the sampled population
};

// ProbExpectedSampler, really a Bernoulli sampler
// Treats the selection of a pixel as an stochastic process
// defined by a Bernoulli distribution with a certain probability
class ProbExpectedSampler : public Sampler
{
public:
  void setup(const UtilityMap&, const UtilityVector&, float ratio);

  inline bool makeDecision() const
  {
	return (*map)(*it_) > ((float)std::rand()/(float)(RAND_MAX/1));
  }
public:
  const UtilityMap* map;
};

// Weighted Random Selection (WRS) without replacement
// using the exponential distribution to generate probabilities
// and Quickselect to choose the samples
// See http://stackoverflow.com/a/30226926
class ProbExactSampler : public Sampler
{
public:
  void setup( const UtilityMap& map, const UtilityVector& utilities, float ratio );

  inline bool makeDecision() const
  {
	return *it_ < threshold_;
  }
protected:
  float threshold_;
};

// Deterministic sampler
// which takes points with highest (+) utility
class DeterministicSampler : public Sampler
{
public:
  void setup( const UtilityMap& map, const UtilityVector& utilities, float ratio );

  inline bool makeDecision() const
  {
	return *it_ > threshold_;
  }
protected:
  float threshold_;
};

// Struct of available implemented samplers
// and methods that take the type as input
// to produce a generic pointer that control them all
typedef Sampler* pSampler;

struct Samplers {
  typedef enum {
  ProbExpected,
  ProbExact,
  Deterministic,
  Saliency
  // don't forget to add to dynamic reconfigure!
  } enum_t;

  static const char* str(enum_t type);

  static pSampler* get(enum_t type, size_t num = 1);
};

} /* namespace selection */
} /* namespace dvo */

#endif // SAMPLER_H

