#include <dvo/selection/utility.h>

namespace dvo
{
namespace selection
{

const char* Utilities::str(enum_t type)
{
  switch(type)
  {
    case Utilities::JacMag:
      return "Jacobian Magnitude";
    case Utilities::GradMag:
      return "Gradient Magnitude";
    case Utilities::Jac:
      return "Complete Jacobian";
    case Utilities::TrVar:
      return "Variance Trace";
    default:
      break;
  }
  assert(false && "Unknown utility calculator type!");

  return "";
}

UtilityCalculator* Utilities::get(Utilities::enum_t type)
{
  // It may be necessary to build a vector if multidimensional criteria
  // (as complete Jacobian) is used
  static JacMagCalculator jacMagCalc;
  static GradMagCalculator gradMagCalc;
  static TrVarCalculator trVarCalc;

  switch(type)
  {
    case Utilities::JacMag:
      return (UtilityCalculator*)&jacMagCalc;
    case Utilities::GradMag:
      return (UtilityCalculator*)&gradMagCalc;
    case Utilities::TrVar:
      return (UtilityCalculator*)&trVarCalc;
    default:
      break;
  }
  assert(false && "Unknown utility calculator type!");

  return 0;
}


} /* namespace selection */
} /* namespace dvo */
