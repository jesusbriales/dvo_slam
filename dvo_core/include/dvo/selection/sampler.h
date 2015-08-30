#ifndef SAMPLER_H
#define SAMPLER_H

namespace dvo
{
namespace selection
{

class Sampler
{
public:
  virtual bool operator() ( float ) const = 0; // Function to apply by the functor
};

class BernoulliSampler : public Sampler
{
public:
  virtual inline bool operator() (float probability) const
  {
    return probability > ((float)std::rand()/(float)(RAND_MAX/1));
  }
};

} /* namespace selection */
} /* namespace dvo */

#endif // SAMPLER_H

