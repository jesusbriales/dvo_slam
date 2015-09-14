#include <dvo/selection/sampler.h>

#include <cmath>
#include <algorithm>

namespace dvo
{
namespace selection
{

void ProbExpectedSampler::setup(
    const UtilityMap &inputMap,
    const UtilityVector &utilities,
    float ratio)
{
  // Set internal parameters of the sampler
  map = &inputMap; // Copy direction into the pointer
  parameters_ = utilities; // Check copy is assignment of pointer (same direction)
  it_ = parameters_.begin();
}

void ProbExactSampler::setup(
    const UtilityMap &map,
    const UtilityVector &utilities,
    float ratio)
{
  // Set internal parameters of the sampler
//  parameters_.resize(utilities.size());
  parameters_ = utilities; // Is this copying data into a new variable?
  it_ = parameters_.begin();

  // Compute exponential samples from mapped utilities and store them as temporary values
  NumIterator t_it = parameters_.begin();
  float divMaxInt = 1.0f/(float)(RAND_MAX/1);
  for(UtilityVector::const_iterator util_it = utilities.begin();
      util_it != utilities.end(); ++util_it, ++t_it)
  {
    // Explanation here:
    // https://en.wikipedia.org/wiki/Exponential_distribution#Generating_exponential_variates
    *t_it = -std::log( (float)std::rand() * divMaxInt ) / map(*util_it) ;
  }

  // Notice the nth point is taken from the beginning of the vector
  // so that the threshold is ABOVE the lowest scores (a matter of sign)
  float numOfSamples = ratio * utilities.size();
  threshold_ = nth_orderStatistic( parameters_, std::floor(numOfSamples) );
}

void DeterministicSampler::setup(const UtilityMap&,
    const UtilityVector &utilities,
    float ratio)
{	
  // Set internal parameters of the sampler
  parameters_ = utilities; // Is this copying data into a new variable?
  it_ = parameters_.begin();

  // Notice the nth point is taken from the end of the vector
  // so that the threshold is BELOW the highest utilities (a matter of sign)
  float numOfSamples = ratio * utilities.size();
  threshold_ = nth_orderStatistic( utilities, utilities.size() - std::floor(numOfSamples) );
}

const char* Samplers::str(enum_t type)
{
  switch(type)
  {
    case Samplers::ProbExpected:
      return "Probabilistic Expected";
    case Samplers::ProbExact:
      return "Probabilistic Exact";
    case Samplers::Deterministic:
      return "Deterministic";
    case Samplers::Saliency:
      return "Saliency";
    default:
      break;
  }
  assert(false && "Unknown sampler type!");

  return "";
}

pSampler* Samplers::get(Samplers::enum_t type, size_t num)
{
  static std::vector<ProbExpectedSampler> probExpected;
  static std::vector<ProbExactSampler> probExact;
  static std::vector<DeterministicSampler> deterministic;

  static std::vector<pSampler> pointers(num);

  switch(type)
  {
  case Samplers::ProbExpected:
  {
    probExpected.resize(num);
    for(size_t i=0; i<num; ++i)
      pointers[i] = (Sampler*)(&probExpected[i]);
    return pointers.data();
  }
  case Samplers::ProbExact:
  {
    probExact.resize(num);
    for(size_t i=0; i<num; ++i)
      pointers[i] = (Sampler*)(&probExact[i]);
    return pointers.data();
  }
  case Samplers::Deterministic:
  {
    deterministic.resize(num);
    for(size_t i=0; i<num; ++i)
      pointers[i] = (Sampler*)(&deterministic[i]);
    return pointers.data();
  }
  default:
    break;
  }
  assert(false && "Unknown sampler type!");

  return 0;
}


} /* namespace selection */
} /* namespace dvo */
