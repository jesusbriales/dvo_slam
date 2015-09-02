#include <dvo/selection/selector.h>

namespace dvo
{
namespace selection
{

Selector::Selector( UtilityVector& utilities, float samplingRatio ) :
  utilities(utilities),
  samplingRatio(samplingRatio)
{
  numOfSamples = samplingRatio * (float)utilities.size();

  map = new UtilityMapPSPF( utilities, numOfSamples );
  sampler = new BernoulliSampler();
}

SelectorNonDirect::SelectorNonDirect( UtilityVector& utilities, float samplingRatio ) :
  utilities(utilities),
  samplingRatio(samplingRatio)
{
  numOfSamples = samplingRatio * (float)utilities.size();

  map = new UtilityMapPSPF( utilities, samplingRatio );
  sampler = new WrsExponentialQuickselectSampler();
}

} /* namespace selection */
} /* namespace dvo */
