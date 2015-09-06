#ifndef H5FILEBENCHMARK_H
#define H5FILEBENCHMARK_H

#include <H5Cpp.h>

#include <dvo/dense_tracking.h>

namespace H5
{

class H5_DLLCPP PredCompType : public CompType
{
public:
  static const CompType IterationStats;
  static const CompType LevelStats;
  static const CompType TimeStats;
  static const CompType Config;
};

// Define equivalent H5 structs as a template
// for better generalization
template <typename T>
struct hEquivalent;

// Define equivalent struct for LevelStats
template <>
struct hEquivalent<dvo::DenseTracker::LevelStats>
{
  size_t Id, MaxValidPixels, ValidPixels, SelectedPixels;

  hvl_t Iterations;

  // Default constructor
  hEquivalent() {}

  // Copy constructor from original LevelStats
  hEquivalent( dvo::DenseTracker::LevelStats& in ) :
    Id(in.Id),
    MaxValidPixels(in.MaxValidPixels),
    ValidPixels(in.ValidPixels),
    SelectedPixels(in.SelectedPixels)
  {
    Iterations.len = in.Iterations.size();
    Iterations.p = in.Iterations.data();
  }
};

template <>
struct hEquivalent<dvo::DenseTracker::TimeStats>
    : public dvo::DenseTracker::TimeStats
{
  // Default constructor
  hEquivalent() {}

  // Copy constructor from original LevelStats
  hEquivalent( dvo::DenseTracker::TimeStats& in ) :
    dvo::DenseTracker::TimeStats(in) {}
};

// Useful macro to reduce extension of new types
#define ADD_MEMBER(name,type) \
  this->insertMember( #name, HOFFSET(THIS_TYPE,name),type)

template <class T>
class BaseCompType : public CompType
{
public:
  BaseCompType() :
    // Set base object CompType to the desired object type
    CompType( sizeof(T) ) { }
};

// Define new type for IterationStats struct
#define THIS_TYPE dvo::DenseTracker::IterationStats
class CompTypeIterationStats : public BaseCompType<THIS_TYPE>
{
public:
  CompTypeIterationStats()
  {
    ADD_MEMBER(Id,PredType::NATIVE_INT);
    ADD_MEMBER(ValidConstraints,PredType::NATIVE_INT);
    ADD_MEMBER(TDistributionLogLikelihood,PredType::NATIVE_DOUBLE);

    {
      hsize_t arrSize[1] = {2};
      H5::ArrayType arrType(H5::PredType::NATIVE_DOUBLE, 1, arrSize);
      ADD_MEMBER(TDistributionMean,arrType);
    }

    {
      hsize_t arrSize[2] = {6,6};
      H5::ArrayType arrType(H5::PredType::NATIVE_DOUBLE, 2, arrSize);
      ADD_MEMBER(EstimateInformation,arrType);
    }
  }
};
#undef THIS_TYPE

// Define equivalent class for LevelStats
struct hLevelStats
{
  size_t Id, MaxValidPixels, ValidPixels, SelectedPixels;

  hvl_t Iterations;

  // Default constructor
  hLevelStats() {}

  // Copy constructor from original LevelStats
  hLevelStats( dvo::DenseTracker::LevelStats& in ) :
    Id(in.Id),
    MaxValidPixels(in.MaxValidPixels),
    ValidPixels(in.ValidPixels),
    SelectedPixels(in.SelectedPixels)
  {
    Iterations.len = in.Iterations.size();
    Iterations.p = in.Iterations.data();
  }
};

// Define new type for LevelStats struct
#define THIS_TYPE hLevelStats
class CompTypeLevelStats : public BaseCompType<THIS_TYPE>
{
public:
  CompTypeLevelStats()
  {
    ADD_MEMBER(Id,PredType::NATIVE_UINT);
    ADD_MEMBER(MaxValidPixels,PredType::NATIVE_UINT);
    ADD_MEMBER(ValidPixels,PredType::NATIVE_UINT);
    ADD_MEMBER(SelectedPixels,PredType::NATIVE_UINT);
    ADD_MEMBER(Iterations,VarLenType(&PredCompType::IterationStats));
  }
};
#undef THIS_TYPE

// Define new type for Times struct
#define THIS_TYPE dvo::DenseTracker::TimeStats
class CompTypeTimeStats : public BaseCompType<THIS_TYPE>
{
public:
  CompTypeTimeStats()
  {
    ADD_MEMBER(level,PredType::NATIVE_DOUBLE);
    ADD_MEMBER(it,PredType::NATIVE_DOUBLE);
    ADD_MEMBER(error,PredType::NATIVE_DOUBLE);
    ADD_MEMBER(linsys,PredType::NATIVE_DOUBLE);
    ADD_MEMBER(presel,PredType::NATIVE_DOUBLE);
    ADD_MEMBER(prejac,PredType::NATIVE_DOUBLE);
    ADD_MEMBER(sel,PredType::NATIVE_DOUBLE);
  }
};
#undef THIS_TYPE

// Define compound type for the Config struct
#define THIS_TYPE dvo::DenseTracker::Config
class CompTypeConfig : public BaseCompType<THIS_TYPE>
{
public:
  CompTypeConfig()
  {
    // Note: Bool variables are stored into NATIVE_UCHAR (1 byte)
    // BUT there is not standard specification for BOOL size,
    // so take care in different machines
    // TODO: Trying to assert but sizeof is giving 1 and 16, why?
//    assert(sizeof(bool) == sizeof(PredType::NATIVE_UCHAR) &&
//           "Bool size is not the same as uchar in this machine, correct bool types");

    ADD_MEMBER(FirstLevel,PredType::NATIVE_INT);
    ADD_MEMBER(LastLevel,PredType::NATIVE_INT);
    ADD_MEMBER(MaxIterationsPerLevel,PredType::NATIVE_INT);
    ADD_MEMBER(Precision,PredType::NATIVE_DOUBLE);
    ADD_MEMBER(Mu,PredType::NATIVE_DOUBLE);
    ADD_MEMBER(UseInitialEstimate,PredType::NATIVE_UCHAR);
    ADD_MEMBER(UseWeighting,PredType::NATIVE_UCHAR);
    ADD_MEMBER(UseParallel,PredType::NATIVE_UCHAR);
    ADD_MEMBER(InfluenceFunctionType,PredType::NATIVE_INT);
    ADD_MEMBER(InfluenceFunctionParam,PredType::NATIVE_FLOAT);
    ADD_MEMBER(ScaleEstimatorType,PredType::NATIVE_INT);
    ADD_MEMBER(ScaleEstimatorParam,PredType::NATIVE_FLOAT);
    ADD_MEMBER(IntensityDerivativeThreshold,PredType::NATIVE_FLOAT);
    ADD_MEMBER(DepthDerivativeThreshold,PredType::NATIVE_FLOAT);
    ADD_MEMBER(SamplingProportion,PredType::NATIVE_FLOAT);
    ADD_MEMBER(UtilityMapType,PredType::NATIVE_INT);
    ADD_MEMBER(SamplerType,PredType::NATIVE_INT);
  }
};
#undef THIS_TYPE

template <typename T>
class DataSetStream : public DataSet
{
public:
  DataSetStream( const DataSet& dset ) :
    DataSet(dset), idx(0)
  {
    // Get dataset internal configuration
    type = dset.getDataType();
    fspace = dset.getSpace();

    // Get dimensions to configure the stream
    hsize_t nDims = fspace.getSimpleExtentNdims();
    std::vector<hsize_t> dims;
    dims.resize(nDims);
    fspace.getSimpleExtentDims( dims.data() );

    // Define the layout of data in local memory (vector of variables/structs)
    hsize_t dataDim[1] = {dims[0]};
    mspace = DataSpace(1,dataDim);

    // Define the layout of the portion in file to write to
    fSlice.resize(2);
    fSlice[0] = dims[0];
    fSlice[1] = 1;

    // Define the offset (variable)
    fOffset.resize(2);
    fOffset[0] = 0;
    fOffset[1] = 0;
    // 2nd component is the iterator updated after each data-write
  }

  void push( T *dataPtr )
  {
    // Select slab in the file space
    fOffset[1] = idx++;
    fspace.selectHyperslab( H5S_SELECT_SET, fSlice.data(), fOffset.data() );

    // TODO: Be aware of dimensions
    // TODO: Could this be overloaded for vector objects and non-vector?
    // Convert to H5 compatible type
    std::vector<hEquivalent<T>> htype_vector;
    size_t dimData = fSlice[0];
    htype_vector.resize( dimData );
    for(size_t i=0; i<dimData; i++)
    {
      htype_vector[i] = dataPtr[i];
    }

    // Write local data to the dataset in the h5 file
    this->write( htype_vector.data(), type, mspace, fspace );
  }

public:
  // Parameters to control the position in file to write
  size_t idx; // Iterator for the stream (incremented after each push)
  std::vector<hsize_t> fSlice;  // Slice size (count of elements in each dimension)
  std::vector<hsize_t> fOffset; // Slice offset (position of the first point)

  // Store copy of most used parameters to avoid reading enquiring every time
  DataType type;
  DataSpace mspace;
  DataSpace fspace;
};

} // end H5 namespace

#endif // H5FILEBENCHMARK_H
