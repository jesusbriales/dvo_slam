#ifndef H5FILEBENCHMARK_H
#define H5FILEBENCHMARK_H

#include <H5Wrap.h>
#include <dvo/dense_tracking.h>

namespace H5
{

// Declare set of proper types useful for the benchmark
class H5_DLLCPP MyPredType : public DataType
{
public:
  // Compound types (equivalent to structures)
  static const CompType IterationStats;
  static const CompType LevelStats;
  static const CompType TimeStats;
  static const CompType Config;

  // Array types (equivalent to matrices)
//  static const ArrayType Matrix4d;
};

// Declare method to create 2D arrays
ArrayType createArrayType2D(const DataType&, hsize_t, hsize_t);

// Declare the necessary equivalent types
DECLARE_EQUIVALENT_TYPE(dvo::DenseTracker::LevelStats)

// Define equivalent struct for LevelStats
// Needs special treatment of Variable Length vectors
template <>
struct hEquivalent<dvo::DenseTracker::LevelStats>
{
  size_t Id, MaxValidPixels, ValidPixels, SelectedPixels;

  hvlconst_t Iterations;

  // Default constructor
  hEquivalent() {}

  // Copy constructor from original LevelStats
  hEquivalent( const dvo::DenseTracker::LevelStats& in ) :
    Id(in.Id),
    MaxValidPixels(in.MaxValidPixels),
    ValidPixels(in.ValidPixels),
    SelectedPixels(in.SelectedPixels)
  {
    Iterations.len = in.Iterations.size();
    Iterations.p = in.Iterations.data();
  }
};

// Declare all H5 custom compound types
// Must inherit from the struct template BaseCompType
// instantiated in the corresponding original type
struct CompTypeIterationStats
    : BaseCompType<dvo::DenseTracker::IterationStats>
{
  CompTypeIterationStats();
};
struct CompTypeLevelStats
    : BaseCompType<hEquivalent<dvo::DenseTracker::LevelStats>>
{
  CompTypeLevelStats();
};
struct CompTypeTimeStats
    : BaseCompType<dvo::DenseTracker::TimeStats>
{
  CompTypeTimeStats();
};
struct CompTypeConfig
    : BaseCompType<dvo::DenseTracker::Config>
{
  CompTypeConfig();
};

} // end H5 namespace

#endif // H5FILEBENCHMARK_H

