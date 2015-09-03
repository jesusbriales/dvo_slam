#ifndef H5FILEBENCHMARK_H
#define H5FILEBENCHMARK_H

#include <H5Cpp.h>

#include <dvo/dense_tracking.h>

namespace H5
{

class H5LevelStats : public CompType
{
public:
  H5LevelStats() : CompType( sizeof(dvo::DenseTracker::LevelStats) )
  {
    this->insertMember(
          "MaxValidPixels",
          HOFFSET(dvo::DenseTracker::LevelStats, MaxValidPixels), H5::PredType::NATIVE_UINT );
    this->insertMember( "ValidPixels", HOFFSET(dvo::DenseTracker::LevelStats, ValidPixels), H5::PredType::NATIVE_UINT );
    this->insertMember( "SelectedPixels", HOFFSET(dvo::DenseTracker::LevelStats, SelectedPixels), H5::PredType::NATIVE_UINT );
  }
};

class DataSetStream : public DataSet
{
public:
  DataSetStream() : DataSet(), idx(0) {}

  // Copy constructor
  DataSetStream( const DataSet& original ) :
    DataSet( original ), idx(0) {}

  void operator<< (const dvo::DenseTracker::LevelStats *levelsPtr)
  {
    // Define slab in the file space
    hsize_t offset[2] = { 0, idx++ };
    hsize_t fdims[2]  = { 3, 1 };
    DataSpace fspace = this->getSpace();
    fspace.selectHyperslab( H5S_SELECT_SET, fdims, offset );

    // Define the data to write
    hsize_t dataDim[1] = {3};
    H5::DataSpace mspace(1,dataDim);

    // Write local data to the dataset in the h5 file
    this->write(levelsPtr, H5LevelStats(), mspace, fspace );
  }

private:
  size_t idx;
};

class H5FileBenchmark : public H5File
{
public:
  H5FileBenchmark( const char* name, unsigned int flags ) :
    H5File(name,flags)
  {
    group = Group( this->createGroup("experiment") );

    hsize_t fdims[2] = {3, 500}; // TODO: Read size
    DataSpace fspace( 2, fdims );

    dset = DataSetStream( group.createDataSet("dset", H5LevelStats(), fspace ) );
  }
//	~H5FileBenchmark();

public: // TODO: Temporal, make protected
  Group group;
  DataSetStream dset;
};

} // end H5 namespace

#endif // H5FILEBENCHMARK_H
