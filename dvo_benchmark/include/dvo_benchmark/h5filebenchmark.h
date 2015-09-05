#ifndef H5FILEBENCHMARK_H
#define H5FILEBENCHMARK_H

#include <H5Cpp.h>

#include <dvo/dense_tracking.h>

// Useful macro to reduce extension of new types
#define ADD_MEMBER(name,type) \
  this->insertMember( #name, HOFFSET(THIS_TYPE,name),type)

namespace H5
{

template <class T>
class BaseCompType : public CompType
{
public:
  BaseCompType() :
    // Set base object CompType to the desired object type
    CompType( sizeof(T) ) { }
};

// Define new type for LevelStats struct
#define THIS_TYPE dvo::DenseTracker::LevelStats
class CompTypeLevelStats : public BaseCompType<THIS_TYPE>
{
public:
  CompTypeLevelStats()
  {
    ADD_MEMBER(MaxValidPixels,PredType::NATIVE_INT);
    ADD_MEMBER(ValidPixels,PredType::NATIVE_INT);
    ADD_MEMBER(SelectedPixels,PredType::NATIVE_INT);
  }
};
#undef THIS_TYPE

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

class DataSetStream : public DataSet
{
public:
//  DataSetStream() : DataSet(), idx(0) {}

  DataSetStream( const DataSet& dset ) :
    DataSet(dset), idx(0)
  {
    // Get dataset internal configuration
    type = dset.getDataType();
    fspace = dset.getSpace();

    // Get dimensions to configure the stream
    std::vector<hsize_t> dims;
    dims.resize(2);
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

  void push( const dvo::DenseTracker::LevelStats *levelsPtr )
  {
    // Select slab in the file space
    fOffset[1] = idx++;
    fspace.selectHyperslab( H5S_SELECT_SET, fSlice.data(), fOffset.data() );

    // Write local data to the dataset in the h5 file
    this->write(levelsPtr, type, mspace, fspace );
  }

  void push( const dvo::DenseTracker::IterationStats *levelsPtr )
  {
    // Select slab in the file space
    fOffset[1] = idx++;
    fspace.selectHyperslab( H5S_SELECT_SET, fSlice.data(), fOffset.data() );

    // Write local data to the dataset in the h5 file
    this->write(levelsPtr, type, mspace, fspace );
  }

public:
  size_t idx;

  std::vector<hsize_t> fSlice;
  std::vector<hsize_t> fOffset;
  // Store copy of most used parameters to avoid reading enquiring every time
  DataType type;
  DataSpace mspace;
  DataSpace fspace;
};

//DataSet& operator<< ( DataSet& dset, const dvo::DenseTracker::LevelStats& s )
//{

//}

} // end H5 namespace

#endif // H5FILEBENCHMARK_H
