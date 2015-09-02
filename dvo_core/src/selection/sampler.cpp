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
    float numOfSamples)
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

  // Compute the threshold to obtain the desired number of samples
  // For that, partially sort the vector around the n_th element
  // and finally read the new value for the n_th element
  // Copy the vector of parameters_ to a copy vector, to keep original one ordered
  NumVector parameters_Ordered;
  parameters_Ordered.resize( parameters_.size() );
  std::copy( parameters_.begin(), parameters_.end(), parameters_Ordered.begin() );
  NumIterator t_nth = parameters_Ordered.begin() + std::floor(numOfSamples);
  std::nth_element( parameters_Ordered.begin(), t_nth, parameters_Ordered.end() );
  threshold_ = *t_nth;
}

void DeterministicSampler::setup(
    const UtilityMap&,
    const UtilityVector &utilities,
    float numOfSamples)
{	
  // Set internal parameters of the sampler
  parameters_ = utilities; // Is this copying data into a new variable?
  it_ = parameters_.begin();

  // Compute the threshold to obtain the desired number of samples
  // For that, partially sort the vector around the n_th element
  // and finally read the new value for the n_th element
  // Copy the vector of parameters_ to a copy vector, to keep original one ordered
  NumVector utilitiesOrdered;
  utilitiesOrdered.resize( utilities.size() );
  std::copy( utilities.begin(), utilities.end(), utilitiesOrdered.begin() );
  // Notice the nth point is taken from the end of the vector
  // so that the threshold is BELOW the highest utilities (a matter of sign)
  NumIterator t_nth = utilitiesOrdered.end() - std::floor(numOfSamples);
  std::nth_element( utilitiesOrdered.begin(), t_nth, utilitiesOrdered.end() );
  threshold_ = *t_nth;
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
    default:
      break;
  }
  assert(false && "Unknown sampler type!");

  return "";
}

Sampler* Samplers::get(Samplers::enum_t type)
{
  static ProbExpectedSampler probExpected;
  static ProbExactSampler probExact;
  static DeterministicSampler deterministic;

  switch(type)
  {
    case Samplers::ProbExpected:
      return (Sampler*)&probExpected;
    case Samplers::ProbExact:
      return (Sampler*)&probExact;
    case Samplers::Deterministic:
      return (Sampler*)&deterministic;
    default:
      break;
  }
  assert(false && "Unknown sampler type!");

  return 0;
}


} /* namespace selection */
} /* namespace dvo */
