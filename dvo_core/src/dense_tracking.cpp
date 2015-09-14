/**
 *  This file is part of dvo.
 *
 *  Copyright 2012 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iomanip>

#include <dvo/dense_tracking.h>
#include <dvo/dense_tracking_impl.h>

#include <assert.h>
#include <sophus/se3.hpp>

#include <Eigen/Core>

#include <dvo/core/datatypes.h>
#include <dvo/core/point_selection_predicates.h>
#include <dvo/util/revertable.h>
#include <dvo/util/stopwatch.h>
#include <dvo/util/id_generator.h>
#include <dvo/util/histogram.h>
//#include <dvo/visualization/visualizer.h>

#include <dvo/selection/selector.h>

namespace dvo
{

using namespace dvo::core;
using namespace dvo::util;

const DenseTracker::Config& DenseTracker::getDefaultConfig()
{
  static Config defaultConfig;

  return defaultConfig;
}

static const Eigen::IOFormat YamlArrayFmt(Eigen::FullPrecision, Eigen::DontAlignCols, ",", ",", "", "", "[", "]");

DenseTracker::DenseTracker(const Config& config) :
    itctx_(cfg),
    weight_calculation_(),
    selection_predicate_(),
    reference_selection_(selection_predicate_),
    information_selection_(),
    saliency_selection_()
{
  configure(config);
}

DenseTracker::DenseTracker(const DenseTracker& other) :
  itctx_(cfg),
  weight_calculation_(),
  selection_predicate_(),
  reference_selection_(selection_predicate_),
  information_selection_(),
  saliency_selection_()
{
  configure(other.configuration());
}

void DenseTracker::configure(const Config& config)
{
  assert(config.IsSane());

  cfg = config;

  selection_predicate_.intensity_threshold = cfg.IntensityDerivativeThreshold;
  selection_predicate_.depth_threshold = cfg.DepthDerivativeThreshold;

  if(cfg.UseWeighting)
  {
    weight_calculation_
      .scaleEstimator(ScaleEstimators::get(cfg.ScaleEstimatorType))
      .scaleEstimator()->configure(cfg.ScaleEstimatorParam);

    weight_calculation_
      .influenceFunction(InfluenceFunctions::get(cfg.InfluenceFunctionType))
      .influenceFunction()->configure(cfg.InfluenceFunctionParam);
  }
  else
  {
    weight_calculation_
      .scaleEstimator(ScaleEstimators::get(ScaleEstimators::Unit))
      .influenceFunction(InfluenceFunctions::get(InfluenceFunctions::Unit));
  }

  if(cfg.SamplingProportion < 0.99999f)
  {
    if(cfg.SamplerType != dvo::selection::Samplers::Saliency)
    {
      information_selection_.samplingRatio = cfg.SamplingProportion;
      information_selection_.utilCalc = selection::Utilities::get(cfg.UtilityType);
      size_t dim = selection::Utilities::dim(cfg.UtilityType);
      information_selection_.map = selection::UtilityMaps::get(cfg.UtilityMapType,dim);
      information_selection_.sampler = selection::Samplers::get(cfg.SamplerType,dim);
    }
  }
}

bool DenseTracker::match(RgbdImagePyramid& reference, RgbdImagePyramid& current, Eigen::Affine3d& transformation)
{
  Result result;
  result.Transformation = transformation;

  bool success = match(reference, current, result);

  transformation = result.Transformation;

  return success;
}

bool DenseTracker::match(dvo::core::PointSelection& reference, RgbdImagePyramid& current, Eigen::Affine3d& transformation)
{
  Result result;
  result.Transformation = transformation;

  bool success = match(reference, current, result);

  transformation = result.Transformation;

  return success;
}

bool DenseTracker::match(dvo::core::RgbdImagePyramid& reference, dvo::core::RgbdImagePyramid& current, dvo::DenseTracker::Result& result)
{
  reference.compute(cfg.getNumLevels());
  reference_selection_.setRgbdImagePyramid(reference);

  return match(reference_selection_, current, result);
}

bool DenseTracker::match(dvo::core::PointSelection& reference, dvo::core::RgbdImagePyramid& current, dvo::DenseTracker::Result& result)
{
  // List of stopwatches to time everything of interest
  static stopwatch_collection sw_level(5, "l", 100); // Each level loop: Comprises presel, prejac, sel and loop
  static stopwatch_collection sw_presel(5, "presel@l", 100); // Compute point cloud, image gradient and preselect valid points
  static stopwatch_collection sw_prejac(5, "prejac@l", 100); // Precompute jacobian matrices
  static stopwatch_collection sw_util(5, "util@l", 100); // Compute utility for the points
  static stopwatch_collection sw_sel(5, "sel@l", 100); // Select points according to their information
  static stopwatch_collection sw_loop(5, "loop@l", 100); // The while loop for solving non-linear problem iteratively
  static stopwatch_collection sw_it(5, "it@l", 500); // The iteration that occurs inside the loop
  static stopwatch_collection sw_error(5, "err@l", 500); // Compute residuals and weights, as well as distribution parameters
  static stopwatch_collection sw_linsys(5, "linsys@l", 500); // Build and solve linear system

  // Compute pyramid for the current image
  current.compute(cfg.getNumLevels());

  bool success = true;

  if(cfg.UseInitialEstimate)
  {
    assert(!result.isNaN() && "Provided initialization is NaN!");
  }
  else
  {
    result.setIdentity();
  }

  // our first increment is the given guess
  Sophus::SE3d inc(result.Transformation.rotation(), result.Transformation.translation());

  Revertable<Sophus::SE3d> initial(inc);
  Revertable<Sophus::SE3d> estimate;

  bool accept = true;

  // allocate the tracker vectors for points, residuals and weights
  // in order to store the residue-related values for the linear system
  if(points_error.size() < reference.getMaximumNumberOfPoints(cfg.LastLevel))
    points_error.resize(reference.getMaximumNumberOfPoints(cfg.LastLevel));
  if(residuals.size() < reference.getMaximumNumberOfPoints(cfg.LastLevel))
    residuals.resize(reference.getMaximumNumberOfPoints(cfg.LastLevel));
  if(jacobians.size() < reference.getMaximumNumberOfPoints(cfg.LastLevel))
    jacobians.resize(reference.getMaximumNumberOfPoints(cfg.LastLevel));
  if(weights.size() < reference.getMaximumNumberOfPoints(cfg.LastLevel))
    weights.resize(reference.getMaximumNumberOfPoints(cfg.LastLevel));

  std::vector<uint8_t> valid_residuals;

  bool debug = false;
  if(debug)
  {
    reference.debug(true);
    valid_residuals.resize(reference.getMaximumNumberOfPoints(cfg.LastLevel));
  }
  /*
  std::stringstream name;
  name << std::setiosflags(std::ios::fixed) << std::setprecision(2) << current.timestamp() << "_error.avi";

  cv::Size s = reference.getRgbdImagePyramid().level(size_t(cfg.LastLevel)).intensity.size();
  cv::Mat video_frame(s.height, s.width * 2, CV_32FC1), video_frame_u8;
  cv::VideoWriter vw(name.str(), CV_FOURCC('P','I','M','1'), 30, video_frame.size(), false);
  float rgb_max = 0.0;
  float depth_max = 0.0;

  std::stringstream name1;
  name1 << std::setiosflags(std::ios::fixed) << std::setprecision(2) << current.timestamp() << "_ref.png";

  cv::imwrite(name1.str(), current.level(0).rgb);

  std::stringstream name2;
  name2 << std::setiosflags(std::ios::fixed) << std::setprecision(2) << current.timestamp() << "_cur.png";

  cv::imwrite(name2.str(), reference.getRgbdImagePyramid().level(0).rgb);
  */
  Eigen::Vector2f mean;
  mean.setZero();
  Eigen::Matrix2f /*first_precision,*/ precision;
  precision.setZero();

  // Define matrix for the linear system, also for information estimate
  Matrix6d A;
  A.setIdentity();

  for(itctx_.Level = cfg.FirstLevel; itctx_.Level >= cfg.LastLevel; --itctx_.Level)
  {
    sw_level[itctx_.Level].start();

    result.Statistics.Levels.push_back(LevelStats());
    LevelStats& level_stats = result.Statistics.Levels.back();

    mean.setZero();
    precision.setZero();

    // reset error after every pyramid level? yes because errors from different levels are not comparable
    itctx_.Iteration = 0;
    itctx_.Error = std::numeric_limits<double>::max();

    RgbdImage& cur = current.level(itctx_.Level);
    const IntrinsicMatrix& K = cur.camera().intrinsics();

    // select the pixels to use from the reference image
    // using the validity of the point
    // plus some threshold in the derivatives
    sw_presel[itctx_.Level].start();

    PointSelection::PointIterator first_point, last_point;
    reference.select(itctx_.Level, first_point, last_point);
    cur.buildAccelerationStructure();

    sw_presel[itctx_.Level].stop();

    level_stats.Id = itctx_.Level;
    level_stats.MaxValidPixels = reference.getMaximumNumberOfPoints(itctx_.Level);
    size_t numValidPixels = last_point - first_point;
    level_stats.ValidPixels = numValidPixels;


    // set the coefficients for the combination of both image variables in the residue,
    // since for the IntensityAndDepth aligned 8-vector with elements v = {i,z,idx,idy,zdx,zdy,_,_}
    // the value to use in the increment computation (residues and derivatives)
    // takes a different form for each variable
    // (please note one image must be previously warped onto the other):
    // res_i = (cur_i - ref_i)/255 [intensity normalized to [0-1] in the residue]
    // res_z = (cur_z - ref_z) [values in m already]
    // idx = ave(cur_idx + ref_idx)/255 * K.fx [average, normalized, scaled to m]
    // idy = ave(cur_idy + ref_idy)/255 * K.fy [average, normalized, scaled to m]
    // zdx = cur_zdx * K.fx (from one image, scaled to m)
    // zdy = cur_zdy * K.fy (from one image, scaled to m)
    // Note on derivatives:
    // since image gradients measure distances between points in pixels,
    // scaling is necessary to get derivatives wrt real world distances (usually in m)
    Vector8f wcur, wref;
    //float wcur_id = 0.5f, wref_id = 0.5f, wcur_zd = 1.0f, wref_zd = 0.0f;
    float wcur_id = 0.0f, wref_id = 1.0f, wcur_zd = 0.0f, wref_zd = 1.0f;
    // Elems:     i             z              idx                      idy                         zdx               zdy
    wcur <<  1.0f / 255.0f,    1.0f,  wcur_id * K.fx() / 255.0f, wcur_id * K.fy() / 255.0f,   wcur_zd * K.fx(), wcur_zd * K.fy(),   0.0f, 0.0f;
    wref << -1.0f / 255.0f,   -1.0f,  wref_id * K.fx() / 255.0f, wref_id * K.fy() / 255.0f,   wref_zd * K.fx(), wref_zd * K.fy(),   0.0f, 0.0f;

    // compute jacobian at selected points
    // precompute jacobian using this image
    sw_prejac[itctx_.Level].start();

    {
      Eigen::Matrix2f scale_i, scale_z;
      scale_i << K.fx() / 255.0f, 0,
          0, K.fy() / 255.0f;
      scale_z << K.fx(), 0,
          0, K.fy();
      Matrix2x6 Jw;
      Vector6 Jz, intensityJacobian, depthJacobian;
      for(PointWithIntensityAndDepth::VectorType::iterator point_it = first_point;
          point_it != last_point; ++point_it)
      {
        computeJacobianOfProjectionAndTransformation(point_it->getPointVec4f(), Jw);
        compute3rdRowOfJacobianOfTransformation(point_it->getPointVec4f(), Jz);
        intensityJacobian = point_it->getIntensityDerivativeVec2f().transpose() * scale_i * Jw;
        depthJacobian = point_it->getDepthDerivativeVec2f().transpose() * scale_z * Jw - Jz.transpose();
        point_it->getIntensityJacobianVec6f() = intensityJacobian;
        point_it->getDepthJacobianVec6f() = depthJacobian;
      }
    }
    sw_prejac[itctx_.Level].stop();

    // Set a minimum number of points to sample, for stability (500?)
    // The ratio is set so that the num of points taken is not below 500
    float samplingRatio = ( cfg.SamplingProportion * numValidPixels < 500.0 ) ?
        500.0 / numValidPixels : cfg.SamplingProportion;
    // Need to choose here between types of selection
    if(cfg.SamplerType == dvo::selection::Samplers::Saliency)
    {
      sw_util[itctx_.Level].start();
      // No utility to compute
      sw_util[itctx_.Level].stop();

      sw_sel[itctx_.Level].start();
      // The method by Meilland, directly uses the Jacobian components
      // Use specific class for the saliency map selection
      if(cfg.SamplingProportion < 0.9999f)
      {
        size_t numOfSamples = samplingRatio * numValidPixels;
        saliency_selection_.setNumPixels(numOfSamples);
        std::vector<Vector6> allJacobians( numValidPixels );

        // Create vector of jacobians to pass to the saliency selector
        std::vector<Vector6>::iterator jac_it = allJacobians.begin();
        for(PointWithIntensityAndDepth::VectorType::iterator point_it = first_point;
            point_it != last_point; ++point_it, ++jac_it)
        {
          // store complete photometric jacobian
          *jac_it = point_it->getIntensityJacobianVec6f();
        }

        // Find list of indeces for the best pixels according to Meilland's method
        const std::set<size_t>& ids =
            saliency_selection_.selectInformation( &allJacobians[ 0 ], allJacobians.size() );

        // Keep selected points only
        {
          PointWithIntensityAndDepth::VectorType::iterator point_it = first_point;
          for(std::set<size_t>::const_iterator it = ids.begin(); it != ids.end(); ++it, ++point_it)
          {
            // Substitute old point with the new selected one
            *point_it = *(first_point + *it);
          }
          last_point = point_it; // Not necessary to remove last one? Or one less element stays than in ids...
        }
        size_t numSelectedPixels = last_point - first_point;
        level_stats.SelectedPixels = numSelectedPixels;
      }
      sw_sel[itctx_.Level].stop();
    }
    else
    {
      // The cases which uses a certain utility function
      // followed by a sampling process which uses those utilities

      // select the most informative pixels to speed up the rest of steps
      {
        sw_util[itctx_.Level].start();
        std::vector<float> utilities (numValidPixels); // one element per valid pixel
        if(cfg.SamplingProportion < 0.9999f) // Numerical precision
          // TODO: Try different proportions for different levels
        { // begin sampling block
          // compute utility (some metric) for each pixel
//          information_selection_.utilCalc->compute( first_point, last_point, A, utilities );
          information_selection_.utilCalc->compute( first_point, last_point, A );
        }
        sw_util[itctx_.Level].stop();

        // Block separated from utility computation for timing only
        sw_sel[itctx_.Level].start();
        if(cfg.SamplingProportion < 0.9999f) // Numerical precision
        {          
          // Sample points to use in the odometry
          // according to their computed utilities
//          information_selection_.map->setup( utilities, samplingRatio );

//          information_selection_.sampler->setup(
//                *information_selection_.map, utilities, samplingRatio);

//          information_selection_.selectPoints<
//              PointWithIntensityAndDepth::VectorType::iterator> (
//                first_point, last_point );

          information_selection_.selectPoints<
              PointWithIntensityAndDepth::VectorType::iterator> (
                samplingRatio, first_point, last_point );
        } // end sampling block

        size_t numSelectedPixels = last_point - first_point;
        level_stats.SelectedPixels = numSelectedPixels;

        sw_sel[itctx_.Level].stop();
      }
    }


    // create variables for the linear system
    NormalEquationsLeastSquares ls;
//    Matrix6d A; // Define before to use it as information container to compute utilities
    Vector6d x, b;
    x = inc.log();

    ComputeResidualsResult compute_residuals_result;
    compute_residuals_result.first_point_error = points_error.begin();
    compute_residuals_result.first_residual = residuals.begin();
    compute_residuals_result.first_jacobian = jacobians.begin();
    compute_residuals_result.first_valid_flag = valid_residuals.begin();

    sw_loop[itctx_.Level].start();

    // solve the non-linear problem iteratively, by linear approximations
    do
    {
      level_stats.Iterations.push_back(IterationStats());
      IterationStats& iteration_stats = level_stats.Iterations.back();
      iteration_stats.Id = itctx_.Iteration;

      sw_it[itctx_.Level].start();

      double total_error = 0.0f;
      sw_error[itctx_.Level].start();
      Eigen::Affine3f transformf;

      inc = Sophus::SE3d::exp(x);
      initial.update() = inc.inverse() * initial();
      estimate.update() = inc * estimate();

      transformf = estimate().matrix().cast<float>();

      if(debug)
      {
        dvo::core::computeResidualsAndValidFlagsSse(first_point, last_point, cur, K, transformf, wref, wcur, compute_residuals_result);
      }
      else
      {
        dvo::core::computeResidualsSse(first_point, last_point, cur, K, transformf, wref, wcur, compute_residuals_result);
      }
      size_t n = (compute_residuals_result.last_residual - compute_residuals_result.first_residual);
      iteration_stats.ValidConstraints = n;

      if(n < 6)
      {
        initial.revert();
        estimate.revert();

        level_stats.TerminationCriterion = TerminationCriteria::TooFewConstraints;

        break;
      }

      if(itctx_.IsFirstIterationOnLevel())
      {
        std::fill(weights.begin(), weights.begin() + n, 1.0f);
      }
      else
      {
        dvo::core::computeWeightsSse(compute_residuals_result.first_residual, compute_residuals_result.last_residual, weights.begin(), mean, precision);
      }

      precision = dvo::core::computeScaleSse(compute_residuals_result.first_residual, compute_residuals_result.last_residual, weights.begin(), mean).inverse();

      float ll = computeCompleteDataLogLikelihood(compute_residuals_result.first_residual, compute_residuals_result.last_residual, weights.begin(), mean, precision);

      iteration_stats.TDistributionLogLikelihood = -ll;
      iteration_stats.TDistributionMean = mean.cast<double>();
      iteration_stats.TDistributionPrecision = precision.cast<double>();
      iteration_stats.PriorLogLikelihood = cfg.Mu * initial().log().squaredNorm();

      total_error = -ll;//iteration_stats.TDistributionLogLikelihood + iteration_stats.PriorLogLikelihood;

      itctx_.LastError = itctx_.Error;
      itctx_.Error = total_error;

      sw_error[itctx_.Level].stop();

      // accept the last increment?
      accept = itctx_.Error < itctx_.LastError;

      if(!accept)
      {
        initial.revert();
        estimate.revert();

        level_stats.TerminationCriterion = TerminationCriteria::LogLikelihoodDecreased;

        break;
      }

      // now build equation system
      sw_linsys[itctx_.Level].start();

      ResidualVectorType::iterator r_it, last_r_it;
      r_it = residuals.begin();
      last_r_it = compute_residuals_result.last_residual;
      JacobianVectorType::iterator j_it = jacobians.begin();
      WeightVectorType::iterator w_it = weights.begin();
      ls.initialize(1);
      for(;r_it != last_r_it; ++r_it, ++j_it, ++w_it)
      {
        ls.update(*j_it, *r_it, (*w_it) * precision);
      }
      ls.finish();

      A = ls.A.cast<double>() + cfg.Mu * Matrix6d::Identity();
      b = ls.b.cast<double>() + cfg.Mu * initial().log();
      x = A.ldlt().solve(b);

      sw_linsys[itctx_.Level].stop();

      iteration_stats.EstimateIncrement = x;
      iteration_stats.EstimateInformation = A;

      itctx_.Iteration++;
      sw_it[itctx_.Level].stop();
    }
    while(accept && x.lpNorm<Eigen::Infinity>() > cfg.Precision && !itctx_.IterationsExceeded());

    if(x.lpNorm<Eigen::Infinity>() <= cfg.Precision)
      level_stats.TerminationCriterion = TerminationCriteria::IncrementTooSmall;

    if(itctx_.IterationsExceeded())
      level_stats.TerminationCriterion = TerminationCriteria::IterationsExceeded;

    sw_loop[itctx_.Level].stop();
    sw_level[itctx_.Level].stop();

    // Store stopwatch times for statistical purposes
    result.Statistics.Times.push_back(TimeStats());
    TimeStats& time_stats = result.Statistics.Times.back();
    time_stats.level = sw_level[itctx_.Level].computeSumAndReset();
    time_stats.loop = sw_loop[itctx_.Level].computeSumAndReset();
    time_stats.it = sw_it[itctx_.Level].computeSumAndReset();
    time_stats.error = sw_error[itctx_.Level].computeSumAndReset();
    time_stats.linsys = sw_linsys[itctx_.Level].computeSumAndReset();
    time_stats.presel = sw_presel[itctx_.Level].computeSumAndReset();
    time_stats.prejac = sw_prejac[itctx_.Level].computeSumAndReset();
    time_stats.sel = sw_sel[itctx_.Level].computeSumAndReset();
    time_stats.util = sw_util[itctx_.Level].computeSumAndReset();
  }

  LevelStats& last_level = result.Statistics.Levels.back();
  IterationStats& last_iteration = last_level.TerminationCriterion != TerminationCriteria::LogLikelihoodDecreased ? last_level.Iterations[last_level.Iterations.size() - 1] : last_level.Iterations[last_level.Iterations.size() - 2];

  result.Transformation = estimate().inverse().matrix();
  result.Information = last_iteration.EstimateInformation * 0.008 * 0.008;
  result.LogLikelihood = last_iteration.TDistributionLogLikelihood + last_iteration.PriorLogLikelihood;

  return success;
}

cv::Mat DenseTracker::computeIntensityErrorImage(dvo::core::RgbdImagePyramid& reference, dvo::core::RgbdImagePyramid& current, const dvo::core::AffineTransformd& transformation, size_t level)
{
  reference.compute(level + 1);
  current.compute(level + 1);
  reference_selection_.setRgbdImagePyramid(reference);
  reference_selection_.debug(true);

  std::vector<uint8_t> valid_residuals;

  if(points_error.size() < reference_selection_.getMaximumNumberOfPoints(level))
    points_error.resize(reference_selection_.getMaximumNumberOfPoints(level));
  if(residuals.size() < reference_selection_.getMaximumNumberOfPoints(level))
    residuals.resize(reference_selection_.getMaximumNumberOfPoints(level));

  valid_residuals.resize(reference_selection_.getMaximumNumberOfPoints(level));

  PointSelection::PointIterator first_point, last_point;
  reference_selection_.select(level, first_point, last_point);

  RgbdImage& cur = current.level(level);
  cur.buildAccelerationStructure();
  const IntrinsicMatrix& K = cur.camera().intrinsics();

  Vector8f wcur, wref;
  // i z idx idy zdx zdy
  float wcur_id = 0.5f, wref_id = 0.5f, wcur_zd = 1.0f, wref_zd = 0.0f;

  wcur <<  1.0f / 255.0f,  1.0f, wcur_id * K.fx() / 255.0f, wcur_id * K.fy() / 255.0f, wcur_zd * K.fx(), wcur_zd * K.fy(), 0.0f, 0.0f;
  wref << -1.0f / 255.0f, -1.0f, wref_id * K.fx() / 255.0f, wref_id * K.fy() / 255.0f, wref_zd * K.fx(), wref_zd * K.fy(), 0.0f, 0.0f;

  ComputeResidualsResult compute_residuals_result;
  compute_residuals_result.first_point_error = points_error.begin();
  compute_residuals_result.first_residual = residuals.begin();
  compute_residuals_result.first_valid_flag = valid_residuals.begin();

  dvo::core::computeResidualsAndValidFlagsSse(first_point, last_point, cur, K, transformation.cast<float>(), wref, wcur, compute_residuals_result);

  cv::Mat result = cv::Mat::zeros(reference.level(level).intensity.size(), CV_32FC1), debug_idx;

  reference_selection_.getDebugIndex(level, debug_idx);

  uint8_t *valid_pixel_it = debug_idx.ptr<uint8_t>();
  ValidFlagIterator valid_residual_it = compute_residuals_result.first_valid_flag;
  ResidualIterator residual_it = compute_residuals_result.first_residual;

  float *result_it = result.ptr<float>();
  float *result_end = result_it + result.total();

  for(; result_it != result_end; ++result_it)
  {
    if(*valid_pixel_it == 1)
    {
      if(*valid_residual_it == 1)
      {
        *result_it = std::abs(residual_it->coeff(0));

        ++residual_it;
      }
      ++valid_residual_it;
    }
    ++valid_pixel_it;
  }

  reference_selection_.debug(false);

  return result;
}


// jacobian computation
inline void DenseTracker::computeJacobianOfProjectionAndTransformation(const Vector4& p, Matrix2x6& j)
{
  NumType z = 1.0f / p(2);
  NumType z_sqr = 1.0f / (p(2) * p(2));

  j(0, 0) =  z;
  j(0, 1) =  0.0f;
  j(0, 2) = -p(0) * z_sqr;
  j(0, 3) = j(0, 2) * p(1);//j(0, 3) = -p(0) * p(1) * z_sqr;
  j(0, 4) = 1.0f - j(0, 2) * p(0);//j(0, 4) =  (1.0 + p(0) * p(0) * z_sqr);
  j(0, 5) = -p(1) * z;

  j(1, 0) =  0.0f;
  j(1, 1) =  z;
  j(1, 2) = -p(1) * z_sqr;
  j(1, 3) = -1.0f + j(1, 2) * p(1); //j(1, 3) = -(1.0 + p(1) * p(1) * z_sqr);
  j(1, 4) = -j(0, 3); //j(1, 4) =  p(0) * p(1) * z_sqr;
  j(1, 5) =  p(0) * z;
}

inline void DenseTracker::compute3rdRowOfJacobianOfTransformation(const Vector4& p, Vector6& j)
{
  j(0) = 0.0;
  j(1) = 0.0;
  j(2) = 1.0;
  j(3) = p(1);
  j(4) = -p(0);
  j(5) = 0.0;
}

} /* namespace dvo */
