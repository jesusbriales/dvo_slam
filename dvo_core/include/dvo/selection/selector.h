#ifndef SELECTOR_H
#define SELECTOR_H

#include <dvo/selection/utility.h>
#include <dvo/selection/utility_map.h>
#include <dvo/selection/sampler.h>

namespace dvo
{
namespace selection
{

class Selector
{
public:
  template <typename it_type>
  void selectPoints( it_type&, it_type& );

public:
  UtilityCalculator* utilCalc;
  UtilityMap* map;
  Sampler* sampler;

  float samplingRatio;
};

template <typename it_type>
void Selector::selectPoints(
	it_type& first_point, it_type& last_point )
{
  // TODO: Caution with accessing unwritten memory
  it_type selected_point_it = first_point;
  for(it_type point_it = first_point;
	  point_it != last_point; ++point_it)
  {
	if( sampler->makeDecision() )
    {
      *selected_point_it = *point_it;
      ++selected_point_it;
    }
	// Increment internal operator in the sampler
	sampler->pointToNext();
  }

  last_point = selected_point_it - 1;
}

} /* namespace selection */
} /* namespace dvo */

#endif // SELECTOR_H

