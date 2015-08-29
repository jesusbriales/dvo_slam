/**
 *  This file is part of dvo.
 *
 *  Copyright 2013 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <dvo/core/information_selection.h>
#include <cmath> // For std::abs
#include <boost/range/numeric.hpp> // For boost::accumulate
#include <boost/range/algorithm.hpp> // For boost::max_element
#include <boost/accumulators/accumulators.hpp> // For incremental sum
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/sum.hpp>

namespace dvo
{
namespace core
{

// Auxiliar functions declaration
float bisect(const ExpectedErrorInNumOfSamples &fun,
              float a, float b, float tolerance );


void ProbabilityProfile::computeParameters( const UtilityVector& utilities, float sampling_ratio )
{
  // Compute an initial estimate of the slope that yields the desired number of samples
  // considering a simplified linear model for the probability profile:
  // alpha = desiredNumOfSamples / sumOfUtilities
  float desiredNumOfSamples = sampling_ratio * utilities.size();
  float alpha = desiredNumOfSamples / boost::accumulate(utilities,0);

  // Set slope as a proportion of the computed alpha (>1.0)
  // TODO: Set heuristically
  slope_ = 2.0 * alpha;

  // Set maximum selection probability
  // TODO: Set heuristically, see Oreshkin's
  probMax_ = std::min( 1.0, 10.0 * sampling_ratio );

  // Solve lower threshold for the given sampling ratio constraint
  float minValue = 0.0;
  float maxValue = *(boost::max_element(utilities));
  float initialInterval[] = {minValue, maxValue};
  // TODO: Set as parameter
  float tolerance = 1.0;
  lowerThres_ = bisect( ExpectedErrorInNumOfSamples(*this, utilities, desiredNumOfSamples),
                       initialInterval[0], initialInterval[1], tolerance );
}

ExpectedErrorInNumOfSamples::ExpectedErrorInNumOfSamples(
    const ProbabilityProfile& probProfile,
    const UtilityVector& utilities,
    float numSamples ) :
  probProfile_(probProfile),
  utilities_(utilities),
  numSamples_(numSamples)
{
}

float ExpectedErrorInNumOfSamples::operator() (float lowerThres) const
{
  using namespace boost::accumulators;

  accumulator_set<float, stats<tag::sum> > acc;
  for(UtilityVector::const_iterator util_it = utilities_.begin(); util_it!=utilities_.end(); ++util_it)
  {
    acc( probProfile_(*util_it, lowerThres) );
  }
  float sumValue = sum( acc );
  return ( sumValue - numSamples_ );
}


float bisect( const ExpectedErrorInNumOfSamples& fun, float a, float b, float tolerance )
{
  float fa,fb,fc;
  fa = fun(a); fb = fun(b);

  // check that a solution exists
  if( fa * fb > 0 )
  {
    std::cout << "There is no solution for lt" << std::endl;
  }

  float distanceThres = 2*tolerance;
  float c;
  while( std::abs(fb-fa) > distanceThres )
  {
    c = 0.5f * (a + b);
    fc = fun(c);

    if( fc * fb < 0 )
    {
        a = c;
        fa = fc;
    }
    else
    {
      b = c;
      fb = fc;
    }
  }
  return 0.5f * (a+b);
}

} /* namespace core */
} /* namespace dvo */
