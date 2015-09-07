#ifndef H5WRAP_H
#define H5WRAP_H

#include <H5Cpp.h>
#include <vector>

namespace H5
{

// Trait to check in compilation time if an equivalent
// class should be used to bridge this type
template <typename T>
struct needs_equivalent
{
  static const bool flag = false;
};

// Define equivalent H5 structs as a template
// for better generalization
template <typename T>
struct hEquivalent {
  // Define empty operators for compatibility
  // TODO: Search alternative to this
  hEquivalent() {}
  hEquivalent(const T&) {}
};

// Macro to declare the H5-compatible type
// corresponding to a certain type
// The implementation must be done by the user
#define DECLARE_EQUIVALENT_TYPE(type) \
template <> struct needs_equivalent<type> \
{ \
  static const bool flag = true; \
}; \
template <> struct hEquivalent<type>;

// TODO: Short macros could be defined so that
// - If specific changes must be done, set a macro for it
//   H5_EQUIVALENT_VL(typename,varnames_with_VL)

// Declare base for all H5 custom compound types
template <class T>
struct BaseCompType : public CompType
{
public:
  BaseCompType() :
    // Set base object CompType to the desired object type
    CompType( sizeof(T) ) { }
};

// Useful macro to reduce extension of new types
#define ADD_MEMBER(name,type) \
  this->insertMember( #name, HOFFSET(THIS_TYPE,name),type)

// Class for streaming in 2D DataSet
// The 1st (row) dimension is the width of the stream,
// while stream or stack occurs in the 2nd (column) dimension
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

  // Push method for vector of variables
  // represented by its first component's pointer
  template<typename T>
  void push( T *dataPtr )
  {
    // Select slab in the file space
    fOffset[1] = idx++;
    fspace.selectHyperslab( H5S_SELECT_SET, fSlice.data(), fOffset.data() );

    // TODO: Be aware of dimensions
    // TODO: Could this be overloaded for vector objects and non-vector?
    // TODO: Try to template this
    if(needs_equivalent<T>::flag)
    {
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
    else
      this->write( dataPtr, type, mspace, fspace );
  }

  // Push method for vectors of variables
  template<typename T>
  void push( std::vector<T>& vec )
  {
    // Select slab in the file space
    fOffset[1] = idx++;
    fspace.selectHyperslab( H5S_SELECT_SET, fSlice.data(), fOffset.data() );

    // TODO: Be aware of dimensions
    // TODO: Could this be overloaded for vector objects and non-vector?
    // TODO: Try to template this
    if(needs_equivalent<T>::flag)
    {
      // Convert to H5 compatible type
      std::vector<hEquivalent<T>> htype_vector;
      size_t dimData = fSlice[0];
      htype_vector.resize( dimData );
      for(size_t i=0; i<dimData; i++)
      {
        htype_vector[i] = vec[i];
      }

      // Write local data to the dataset in the h5 file
      this->write( htype_vector.data(), type, mspace, fspace );
    }
    else
      this->write( vec.data(), type, mspace, fspace );
  }

  // Push method for single variables
  template<typename T>
  void push( T& value)
  {
    // Select slab in the file space
    fOffset[1] = idx++;
    fspace.selectHyperslab( H5S_SELECT_SET, fSlice.data(), fOffset.data() );

    this->write( &value, type, mspace, fspace );
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

// Overload operator<< to simulate stream with strings
template<typename T>
DataSetStream& operator<< (DataSetStream& dset, T& value)
{
  // Use class internal method to write and increment iterator
  dset.push( value );

  // Return the same lhs object to chain operations if wanted
  return dset;
}

} // end H5 namespace

#endif // H5WRAP_H

