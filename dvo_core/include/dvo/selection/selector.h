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
  void selectPoints( float samplingRatio, it_type&, it_type& );

public:
  UtilityCalculator* utilCalc;
  pUtilityMap* map;
  pSampler* sampler;

  float samplingRatio;
};

template <typename it_type>
void Selector::selectPoints( float samplingRatio,
	it_type& first_point, it_type& last_point )
{
  // Dimension of the selection problem (num of elements in a utility vector)
  size_t dim = utilCalc->utilities.size();
  // Divide the sampling ratio between the number of dimensions
  // to approximately sum up to the desired total samplingRatio
  samplingRatio /= dim;

  for(size_t idx=0; idx < dim; idx++)
  {
    // THIS IS FAILING:
    // Return a vector of objects in the get method, instead of a pointer?

    // Setup map
    UtilityMap* currMap = *(map + idx);
    currMap->setup( utilCalc->utilities[idx], samplingRatio );

    // Setup sampler
    Sampler* currSampler = *(sampler + idx);
    currSampler->setup( *currMap, utilCalc->utilities[idx], samplingRatio );
  }

  // TODO: Caution with accessing unwritten memory
  // TODO: Fix types for pSampler*
  it_type selected_point_it = first_point;
  for(it_type point_it = first_point;
	  point_it != last_point; ++point_it)
  {
    for( pSampler* p = sampler; p < sampler + dim; ++p )
    {
      if( (*p)->makeDecision() )
      {
        *selected_point_it = *point_it;
        ++selected_point_it;
        break;
      }
    }

    // Increment internally pointed point in the samplers
    for( pSampler* p = sampler; p < sampler + dim; ++p )
      (*p)->pointToNext();
  }

  last_point = selected_point_it - 1; // Need to remove 1? Check size
}

} /* namespace selection */
} /* namespace dvo */

#endif // SELECTOR_H

