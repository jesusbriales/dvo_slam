#include <dvo_benchmark/h5filebenchmark.h>

using namespace H5;

// Implement predefined instances
const CompType MyPredType::IterationStats = CompTypeIterationStats();
const CompType MyPredType::LevelStats = CompTypeLevelStats();
const CompType MyPredType::TimeStats = CompTypeTimeStats();
const CompType MyPredType::Config = CompTypeConfig();

//const ArrayType MyPredType::Matrix4d = createArrayType<PredType::NATIVE_DOUBLE>(4,4);

ArrayType H5::createArrayType2D(const DataType& base_type, hsize_t rows, hsize_t cols)
{
  hsize_t arrSize[2] = {rows,cols};
  return ArrayType(base_type, 2, arrSize);
}

// Define new type for IterationStats struct
#define THIS_TYPE dvo::DenseTracker::IterationStats
CompTypeIterationStats::CompTypeIterationStats()
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
#undef THIS_TYPE

// Define new type for LevelStats struct
#define THIS_TYPE hEquivalent<dvo::DenseTracker::LevelStats>
CompTypeLevelStats::CompTypeLevelStats()
{
  ADD_MEMBER(Id,PredType::NATIVE_UINT);
  ADD_MEMBER(MaxValidPixels,PredType::NATIVE_UINT);
  ADD_MEMBER(ValidPixels,PredType::NATIVE_UINT);
  ADD_MEMBER(SelectedPixels,PredType::NATIVE_UINT);
  ADD_MEMBER(Iterations,VarLenType(&MyPredType::IterationStats));
}
#undef THIS_TYPE

// Define new type for Times struct
#define THIS_TYPE dvo::DenseTracker::TimeStats
CompTypeTimeStats::CompTypeTimeStats()
{
  ADD_MEMBER(presel,PredType::NATIVE_DOUBLE);
  ADD_MEMBER(prejac,PredType::NATIVE_DOUBLE);
  ADD_MEMBER(util,PredType::NATIVE_DOUBLE);
  ADD_MEMBER(sel,PredType::NATIVE_DOUBLE);
  ADD_MEMBER(level,PredType::NATIVE_DOUBLE);
  ADD_MEMBER(loop,PredType::NATIVE_DOUBLE);
  ADD_MEMBER(it,PredType::NATIVE_DOUBLE);
  ADD_MEMBER(error,PredType::NATIVE_DOUBLE);
  ADD_MEMBER(linsys,PredType::NATIVE_DOUBLE);
}
#undef THIS_TYPE

// Define compound type for the Config struct
#define THIS_TYPE dvo::DenseTracker::Config
CompTypeConfig::CompTypeConfig()
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
#undef THIS_TYPE
