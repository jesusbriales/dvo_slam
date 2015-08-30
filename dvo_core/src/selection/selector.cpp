#include <dvo/selection/selector.h>

namespace dvo
{
namespace selection
{

Selector::Selector( UtilityVector& utilities, float samplingRatio ) :
  utilities(utilities)
{
  map = new UtilityMapPSPF( utilities, samplingRatio );
  sampler = new BernoulliSampler();
}

} /* namespace selection */
} /* namespace dvo */
