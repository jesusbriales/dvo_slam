#ifndef SELECTOR_H
#define SELECTOR_H

#include <dvo/selection/utility_map.h>
#include <dvo/selection/sampler.h>

namespace dvo
{
namespace selection
{

class Selector
{
public:
  Selector(UtilityVector& utilities, float samplingRatio );

  template <typename iterator_type>
  void selectPoints( iterator_type&, iterator_type& );

public:
  UtilityVector utilities;
  UtilityMap* map;
  Sampler* sampler;
};

template <typename iterator_type>
void Selector::selectPoints(
    iterator_type& first_point, iterator_type& last_point )
{
  // TODO: Caution with accessing unwritten memory
  UtilityVector::const_iterator util_it = utilities.begin();
  iterator_type selected_point_it = first_point;
  for(iterator_type point_it = first_point;
      point_it != last_point; ++point_it, ++util_it)
  {
    if( (*sampler)( (*map)(*util_it) ) )
    {
      *selected_point_it = *point_it;
      ++selected_point_it;
    }
  }

  last_point = selected_point_it - 1;
}


// A selector class which needs temporary storage for the sampler
class SelectorNonDirect
{
public:
  SelectorNonDirect(UtilityVector& utilities, float samplingRatio );

  template <typename iterator_type>
  void selectPoints( iterator_type&, iterator_type& );

public:
  UtilityVector utilities;
  UtilityMap* map;
  SamplerWithStorage* sampler;
};

template <typename iterator_type>
void SelectorNonDirect::selectPoints(
    iterator_type& first_point, iterator_type& last_point )
{
  // Make preliminary step necessary for the *global* sampler
  sampler->setup( *map, utilities );

  // Make point-wise sampling
  ProbabilityIterator t_it = sampler->temporaries.begin();
  // TODO: Caution with accessing unwritten memory
  iterator_type selected_point_it = first_point;
  for(iterator_type point_it = first_point;
      point_it != last_point; ++point_it, ++t_it)
  {
    if( (*sampler)( *t_it ) )
    {
      *selected_point_it = *point_it;
      ++selected_point_it;
    }
  }

  last_point = selected_point_it - 1;
}

} /* namespace selection */
} /* namespace dvo */

#endif // SELECTOR_H

