#include <dvo/selection/utility_map.h>

#include <cmath> // For std::abs

namespace dvo
{
namespace selection
{

// Numerical solvers
float bisect( const Functor& fun, float a, float b, float tolerance )
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

} /* namespace selection */
} /* namespace dvo */
