#include <dvo/selection/solvers.h>

#include <cmath> // For std::abs
#include <iostream>

namespace dvo
{
namespace selection
{

// Numerical solvers
float bisect( Functor& fun, float a, float b, float tolerance )
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
  size_t counter = 0;
  while( std::abs(fb-fa) > distanceThres && counter < 100 )
    // Repeat while:
    // distance between values in a and b is above threshold
    // the number of iterations is lower than 100
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
    // Increment the iteration counter
    ++counter;
  }
  return 0.5f * (a+b);
}

} /* namespace selection */
} /* namespace dvo */
