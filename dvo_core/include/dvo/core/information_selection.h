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

#ifndef INFORMATION_SELECTION_H_
#define INFORMATION_SELECTION_H_

#include <vector> // For std::vector

#include <dvo/core/point_selection.h> // For point iterators

namespace dvo
{
namespace core
{

typedef float Utility;
typedef std::vector<Utility> UtilityVector;
typedef UtilityVector::iterator UtilityIterator;
typedef float Probability;

class ProbabilityProfile
{
public:
//  ProbabilityProfile();

  inline Probability operator() (Utility point_utility) const;

  void computeParameters( const UtilityVector& utilities, float sampling_ratio );

  template <typename iterator_type>
  void samplePoints(
      const UtilityVector& utilities,
      iterator_type& first_point, iterator_type& last_point );

private:
  float lowerThres, upperThres;
  float slope;
  float probMax; // Maximum selection probability
};

class ExpectedErrorInNumOfSamples
{
public:
  ExpectedErrorInNumOfSamples(
      const ProbabilityProfile& probProfile,
      const UtilityVector& utilities,
      float numSamples );
  inline float operator() (float lowerThres) const;

protected:
  ProbabilityProfile probProfile_;
  UtilityVector utilities_;
  float numSamples_;
};

// Template implementations
template <typename iterator_type>
void ProbabilityProfile::samplePoints(
    const UtilityVector& utilities,
    iterator_type& first_point, iterator_type& last_point )
{

  UtilityVector::const_iterator util_it = utilities.begin();
  iterator_type selected_point_it = first_point;
  double probability;
  for(iterator_type point_it = first_point;
      point_it != last_point; ++point_it, ++util_it)
  {
    // compute probability from the profile
    probability = this->operator ()( *util_it ); // operator()
    // generate random sampling for given probability
    float x = (float)std::rand()/(float)(RAND_MAX/1);
    bool doSample = false;
    if(probability > x) doSample = true;
    if(doSample)
    {
      *selected_point_it = *point_it;
      ++selected_point_it;
    }
  }

  last_point = selected_point_it - 1;
}

} /* namespace core */
} /* namespace dvo */

#endif /* INFORMATION_SELECTION_H_ */
