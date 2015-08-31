#ifndef SOLVERS_H
#define SOLVERS_H

namespace dvo
{
namespace selection
{

// Define generic functor
// All specific functors to be used with the solvers below
// should inherit from this abstract class
struct Functor
{
  virtual float operator() ( float paramValue ) = 0;
};

float bisect( Functor& fun, float a, float b, float tolerance );

} /* namespace selection */
} /* namespace dvo */

#endif // SOLVERS_H

