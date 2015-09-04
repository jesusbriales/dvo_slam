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

class DataSetStream /* : public DataSet */
{
public:
//  DataSetStream() : DataSet(), idx(0) {}

  DataSetStream( const DataSet& dset ) :
    dset_( dset ), idx(0)
  {
//    // Get dataset internal configuration
//    type = dset.getDataType();
//    fspace = dset.getSpace();

//    // Get dimensions to configure the stream
//    hsize_t* dims;
////    fspace.getSimpleExtentDims( dims ); // Why not work?
//    dset.getSpace().getSimpleExtentDims( dims );

//    // Define the layout of data in local memory (vector of variables/structs)
//    hsize_t dataDim[1] = {dims[0]};
//    mspace = DataSpace(1,dataDim);

//    // Define the layout of the portion in file to write to
//    fSlice.resize(2);
//    fSlice[0] = dims[0];
//    fSlice[1] = 1;
  }

//  ~DataSetStream()
//  {
//    std::cout << "DataSetStream is being deleted" << std::endl;

//    // Manually close all H5 related objects
//    dset_.close();
//    type.close();
//    mspace.close();
//    fspace.close();
//  }

  void operator<< (const dvo::DenseTracker::LevelStats *levelsPtr)
  {
    std::vector<hsize_t> fSlice;
    // Store copy of most used parameters to avoid reading enquiring every time
    DataType type;
    DataSpace mspace;
    DataSpace fspace;

    // Get dataset internal configuration
    type = dset_.getDataType();
    fspace = dset_.getSpace();

    // Get dimensions to configure the stream
    hsize_t* dims;
//    fspace.getSimpleExtentDims( dims ); // Why not work?
    dset_.getSpace().getSimpleExtentDims( dims );

    // Define the layout of data in local memory (vector of variables/structs)
    hsize_t dataDim[1] = {dims[0]};
    mspace = DataSpace(1,dataDim);

    // Define the layout of the portion in file to write to
    fSlice.resize(2);
    fSlice[0] = dims[0];
    fSlice[1] = 1;

    // Define slab in the file space
    hsize_t offset[2] = { 0, idx++ };
//    DataSpace fspace = this->getSpace();
    fspace.selectHyperslab( H5S_SELECT_SET, fSlice.data(), offset );

    // Write local data to the dataset in the h5 file
//    this->write(levelsPtr, CompTypeLevelStats(), mspace, fspace );
//    dset_.write(levelsPtr, this->getDataType(), mspace, fspace );
    dset_.write(levelsPtr, type, mspace, fspace );
  }

private:
  DataSet dset_;
  size_t idx;

//  std::vector<hsize_t> fSlice;
//  // Store copy of most used parameters to avoid reading enquiring every time
//  DataType type;
//  DataSpace mspace;
//  DataSpace fspace;
};

} // end H5 namespace

#endif // H5FILEBENCHMARK_H
