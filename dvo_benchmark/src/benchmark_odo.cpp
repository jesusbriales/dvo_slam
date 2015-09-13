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

#include <ros/ros.h>

#include <dvo/core/intrinsic_matrix.h>
#include <dvo/core/rgbd_image.h>
#include <dvo/core/surface_pyramid.h>

#include <dvo/visualization/camera_trajectory_visualizer.h>
#include <dvo/visualization/pcl_camera_trajectory_visualizer.h>
#include <dvo_ros/visualization/ros_camera_trajectory_visualizer.h>

#include <dvo/dense_tracking.h>
#include <dvo_slam/keyframe_tracker.h>
#include <dvo_slam/serialization/map_serializer.h>
#include <dvo_slam/visualization/graph_visualizer.h>
#include <dvo_slam/KeyframeSlamConfig.h>

#include <dvo_ros/util/configtools.h>
#include <dvo/util/stopwatch.h>
#include <dvo/util/id_generator.h>

#include <dvo_benchmark/file_reader.h>
#include <dvo_benchmark/rgbd_pair.h>
#include <dvo_benchmark/groundtruth.h>
#include <dvo_benchmark/tools.h>

#include <H5Cpp.h>
#include <dvo_benchmark/h5filebenchmark.h>
#include <libgen.h>

dvo::core::RgbdImagePyramidPtr load(dvo::core::RgbdCameraPyramid& camera, std::string rgb_file, std::string depth_file)
{
  cv::Mat rgb, grey, grey_s16, depth, depth_inpainted, depth_mask, depth_mono, depth_float;

  bool rgb_available = false;
  rgb = cv::imread(rgb_file, 1);
  depth = cv::imread(depth_file, -1);

  if(rgb.total() == 0 || depth.total() == 0) return dvo::core::RgbdImagePyramidPtr();

  if(rgb.type() != CV_32FC1)
  {
    if(rgb.type() == CV_8UC3)
    {
      cv::cvtColor(rgb, grey, CV_BGR2GRAY);
      rgb_available = true;
    }
    else
    {
      grey = rgb;
    }

    grey.convertTo(grey_s16, CV_32F);
  }
  else
  {
    grey_s16 = rgb;
  }

  if(depth.type() != CV_32FC1)
  {
    dvo::core::SurfacePyramid::convertRawDepthImageSse(depth, depth_float, 1.0f / 5000.0f);
  }
  else
  {
    depth_float = depth;
  }


  //depth_float.setTo(dvo::core::InvalidDepth, depth_float > 1.2f);

  dvo::core::RgbdImagePyramidPtr result = camera.create(grey_s16, depth_float);

  if(rgb_available)
    rgb.convertTo(result->level(0).rgb, CV_32FC3);

  return result;
}

class BenchmarkNode
{
public:
  struct Config
  {
    bool EstimateTrajectory;
    std::string TrajectoryFile;
    bool RenderVideo;
    std::string VideoFolder;

    std::string CameraFile;

    std::string RgbdPairFile;
    std::string GroundtruthFile;
    std::string Hdf5File;
    std::string Hdf5Group;
    bool Hdf5Overwrite;

    bool ShowGroundtruth;
    bool ShowEstimate;

    bool KeepAlive;

    bool EstimateRequired();
    bool VisualizationRequired();
  };

  struct Storage
  {
    H5::H5File file;
    H5::EGroup group;
    H5::Group mainGroup;
//    H5::Attribute config;

    // Default empty constructor
    Storage( )
    {
      group = H5::EGroup();
    }
    // Constructor from file and group data
    Storage(const std::string& file, const std::string& group_path);
  };

  BenchmarkNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  bool configure();

  void run();

  void createReferenceCamera(dvo::visualization::CameraTrajectoryVisualizerInterface* visualizer, const dvo::core::RgbdImage& img, const Eigen::Affine3d& pose);

  void renderWhileSwitchAndNotTerminated(dvo::visualization::CameraTrajectoryVisualizerInterface* visualizer, const dvo::visualization::Switch& s);

  void processInput(dvo::visualization::CameraTrajectoryVisualizerInterface* visualizer);
private:
  ros::NodeHandle &nh_, nh_vis_, &nh_private_;
  Config cfg_;
  Storage store_;

  std::ostream *trajectory_out_;
  dvo_benchmark::FileReader<dvo_benchmark::Groundtruth> *groundtruth_reader_;
  dvo_benchmark::FileReader<dvo_benchmark::RgbdPair> *rgbdpair_reader_;

  dvo::visualization::Switch dump_camera_pose_, load_camera_pose_;
};

bool BenchmarkNode::Config::EstimateRequired()
{
  return EstimateTrajectory || ShowEstimate;
}

bool BenchmarkNode::Config::VisualizationRequired()
{
  return ShowGroundtruth || ShowEstimate || RenderVideo;
}

BenchmarkNode::BenchmarkNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
    nh_(nh),
    nh_vis_(nh, "dvo_vis"),
    nh_private_(nh_private),
    trajectory_out_(0),
    groundtruth_reader_(0),
    rgbdpair_reader_(0),
    dump_camera_pose_(false),
    load_camera_pose_(false)
{
}

bool BenchmarkNode::configure()
{
  // dataset files related stuff
  if(nh_private_.getParam("rgbdpair_file", cfg_.RgbdPairFile))
  {
    rgbdpair_reader_ = new dvo_benchmark::FileReader<dvo_benchmark::RgbdPair>(cfg_.RgbdPairFile);
    rgbdpair_reader_->skipComments();

    if(!rgbdpair_reader_->next())
    {
      std::cerr << "Failed to open '" << cfg_.RgbdPairFile << "'!" << std::endl;
      return false;
    }
  }
  else
  {
    std::cerr << "Missing 'rgbdpair_file' parameter!" << std::endl;
    return false;
  }

  // trajectory estimation related stuff
  nh_private_.param("estimate_trajectory", cfg_.EstimateTrajectory, false);
  if(cfg_.EstimateTrajectory)
  {
    if(nh_private_.getParam("trajectory_file", cfg_.TrajectoryFile) && !cfg_.TrajectoryFile.empty())
    {
      trajectory_out_ = new std::ofstream(cfg_.TrajectoryFile.c_str());

      if(trajectory_out_->fail())
      {
        delete trajectory_out_;

        std::cerr << "Failed to open '" << cfg_.TrajectoryFile << "'!" << std::endl;
        return false;
      }
    }
    else
    {
      trajectory_out_ = &std::cout;
    }
  }

  // video rendering related stuff
  nh_private_.param("render_video", cfg_.RenderVideo, false);
  if(cfg_.RenderVideo)
  {
    if(!nh_private_.getParam("video_folder", cfg_.VideoFolder) || cfg_.VideoFolder.empty())
    {
      std::cerr << "Missing 'video_folder' parameter!" << std::endl;
      return false;
    }
  }

  nh_private_.param("camera_file", cfg_.CameraFile, std::string(""));

  // ground truth related stuff
  nh_private_.param("show_groundtruth", cfg_.ShowGroundtruth, false);
  if(cfg_.ShowGroundtruth)
  {
    if(nh_private_.getParam("groundtruth_file", cfg_.GroundtruthFile))
    {
      groundtruth_reader_ = new dvo_benchmark::FileReader<dvo_benchmark::Groundtruth>(cfg_.GroundtruthFile);
      groundtruth_reader_->skipComments();

      if(!groundtruth_reader_->next())
      {
        std::cerr << "Failed to open '" << cfg_.GroundtruthFile << "'!" << std::endl;
        return false;
      }
    }
    else
    {
      std::cerr << "Missing 'groundtruth_file' parameter!" << std::endl;
      return false;
    }
  }

  nh_private_.param("show_estimate", cfg_.ShowEstimate, false);
  nh_private_.param("keep_alive", cfg_.KeepAlive, cfg_.VisualizationRequired());

  // hdf5 storage stuff
  if( nh_private_.getParam("hdf5_file", cfg_.Hdf5File) )
  {
    nh_private_.getParam("hdf5_group", cfg_.Hdf5Group);
    store_ = Storage(cfg_.Hdf5File,cfg_.Hdf5Group);

    nh_private_.getParam("hdf5_overwrite", cfg_.Hdf5Overwrite);
  }

  return true;
}

template <typename T>
H5::Group openOrCreateGroup( T& base, const char* name )
{
  // Set the errors?
  herr_t status = H5Eset_auto(0,NULL,NULL);
  // Look for the object
  status = H5Gget_objinfo( base.getId(), name, 0, NULL );

  if( status == 0 )
    // If the object is found, it exists, open it
    return base.openGroup( name );
  else
    // If the object is not found, create it
    return base.createGroup( name );
}

bool groupExists( H5::H5File& file, std::string group_path )
{
//  bool exists;
  herr_t status = H5Eset_auto(0,NULL,NULL);
  status = H5Gget_objinfo( file.getId(), group_path.c_str(), 0, NULL );
  return (status==0) ? true : false;
//  if( status == 0 )
//    exists = true;
//  else
//    exists = false;
}

void createRecursive( H5::H5File& file, std::string path )
{
  // Find parent group of the given path
  size_t found = path.find_last_of("/\\");
  std::string parent = path.substr(0,found);

  // Check if:
  // 1. The parent is empty (this group is at root level)
  // 2. If not empty, if parent does not exist
  if( parent != "" && !groupExists( file, parent ) )
    // If the parent groups does not exist, create it recursively
    createRecursive( file, parent );

  H5::Group( file.createGroup( path ) );
}

BenchmarkNode::Storage::Storage(
    const std::string& file_path,
    const std::string& group_path)
{
  // Check if file exists
  if (std::ifstream(file_path))
  {
    // Case the file already exists, check it is HDF5
    if(H5::H5File::isHdf5(file_path))
    {
      // Open HDF5 file for the experiment
      file = H5::H5File( file_path, H5F_ACC_RDWR );
    }
    else
    {
      std::cerr << "The existing file is not HDF5" << std::endl;
      std::terminate();
    }
  }
  else
  {
    // If not openable because it does not exist,
    // create a new file (overwriting)
    file = H5::H5File( file_path, H5F_ACC_TRUNC );
  }

  // Open group
  {
    // If the group does not exist, create it and all its parents recursively
    if( !groupExists( file, group_path ) )
      createRecursive( file, group_path );

    // Open already existing group
    group = H5::EGroup( file.openGroup( group_path ) );
  }

  // Open main group (parent group) for writing attribute
  {
    char *main_group = { dirname((char*)group_path.c_str()) }; // Warning: It changes original string
    mainGroup = file.openGroup( main_group );
  }
}

void BenchmarkNode::renderWhileSwitchAndNotTerminated(dvo::visualization::CameraTrajectoryVisualizerInterface* visualizer, const dvo::visualization::Switch& s)
{
  if(cfg_.VisualizationRequired())
  {
    while(s.value() && ros::ok())
    {
      ros::spinOnce();
      //processInput(visualizer);

      // manual render in case we want to render a video
      if(cfg_.RenderVideo)
      {
        //((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->render(5);
      }
      else
      {
        ros::Rate(5).sleep();
      }
    }
  }
}

void BenchmarkNode::processInput(dvo::visualization::CameraTrajectoryVisualizerInterface* visualizer)
{
  if(cfg_.VisualizationRequired())
  {
    if(dump_camera_pose_.value())
    {
      if(!cfg_.CameraFile.empty())
      {
        std::cerr << "saving camera pose to '" << cfg_.CameraFile << "'" << std::endl;

        std::vector<pcl::visualization::Camera> cams;
        ((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->visualizer().getCameras(cams);

        // same output format as PCLVisualizerInteractorStyle (when pressing 'c' key)
        std::ofstream camera_pose_file(cfg_.CameraFile.c_str());
        camera_pose_file
        << cams[0].clip[0] << "," << cams[0].clip[1] << "/"
        << cams[0].focal[0] << "," << cams[0].focal[1] << "," << cams[0].focal[2] << "/"
        << cams[0].pos[0] << "," << cams[0].pos[1] << "," << cams[0].pos[2] << "/"
        << cams[0].view[0] << "," << cams[0].view[1] << "," << cams[0].view[2] << "/"
        << cams[0].fovy << "/"
        << cams[0].window_size[0] << "," << cams[0].window_size[1] << "/"
        << cams[0].window_pos[0] << "," << cams[0].window_pos[1];
        camera_pose_file.close();
      }
      else
      {
        std::cerr << "Can't save camera pose, set the 'camera_file' parameter!" << std::endl;
      }

      dump_camera_pose_.toggle();
    }

    if(load_camera_pose_.value())
    {
      if(!cfg_.CameraFile.empty())
      {
        std::cerr << "loading camera pose from '" << cfg_.CameraFile << "'" << std::endl;

        char* option_string = "-cam";
        char parameter_string[2048];

        std::ifstream camera_pose_file(cfg_.CameraFile.c_str());
        camera_pose_file.getline(parameter_string, 2048, '\n');
        camera_pose_file.close();

        char** argv = new char*[3];
        argv[1] = option_string;
        argv[2] = parameter_string;

        ((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->visualizer().getCameraParameters(3, argv);
        ((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->visualizer().updateCamera();
        delete [] argv;
      }
      else
      {
        std::cerr << "Can't load camera pose, set the 'camera_file' parameter!" << std::endl;
      }

      load_camera_pose_.toggle();
    }
  }
}

void BenchmarkNode::createReferenceCamera(dvo::visualization::CameraTrajectoryVisualizerInterface* visualizer, const dvo::core::RgbdImage& img, const Eigen::Affine3d& pose)
{
  dvo::core::RgbdImagePtr first = img.camera().create(img.intensity, img.depth);
  first->rgb = img.rgb.clone();

  cv::Vec3f* p = first->rgb.ptr<cv::Vec3f>();

  for(int idx = 0; idx < first->rgb.total(); ++idx, ++p)
  {
    p->val[2] = 0.666f * p->val[2];
    p->val[1] = 0.666f * p->val[1];
    p->val[0] = std::min(255.0f, 1.333f * p->val[0]);
  }

  visualizer->camera("reference")->
      color(dvo::visualization::Color::blue()).
      update(*first, pose).
      show();
}

void BenchmarkNode::run()
{
  // setup visualizer
  dvo::visualization::Switch pause_switch(false), dummy_switch(true);
  dvo::visualization::CameraTrajectoryVisualizerInterface* visualizer;
  dvo_slam::visualization::GraphVisualizer* graph_visualizer;

  if(cfg_.VisualizationRequired())
  {
    visualizer = new dvo_ros::visualization::RosCameraTrajectoryVisualizer(nh_vis_);
    graph_visualizer = new dvo_slam::visualization::GraphVisualizer(*dynamic_cast<dvo_ros::visualization::RosCameraTrajectoryVisualizer*>(visualizer));
    //((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->bindSwitchToKey(pause_switch, "p");
    //((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->bindSwitchToKey(dump_camera_pose_, "s");
    //
    //if(cfg_.RenderVideo)
    //{
    //  ((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->bindSwitchToKey(load_camera_pose_, "l");
    //  ((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->visualizer().getRenderWindow()->SetSize(1280, 960);
    //}
  }
  else
  {
    visualizer = new dvo::visualization::NoopCameraTrajectoryVisualizer();
  }

  dvo::util::IdGenerator frame_ids(cfg_.VideoFolder + std::string("/frame_"));

  // setup camera parameters
  // TODO: load from file
  //dvo::core::IntrinsicMatrix intrinsics = dvo::core::IntrinsicMatrix::create(525.0f, 525.0f, 320.0f, 240.0f);
  //fr1
  dvo::core::IntrinsicMatrix intrinsics = dvo::core::IntrinsicMatrix::create(517.3, 516.5, 318.6, 255.3);

  //fr2
  //dvo::core::IntrinsicMatrix intrinsics = dvo::core::IntrinsicMatrix::create(520.9f, 521.0f, 325.1f, 249.7f);

  //fr3
  //dvo::core::IntrinsicMatrix intrinsics = dvo::core::IntrinsicMatrix::create(535.4f, 539.2f, 320.1f, 247.6f);

  dvo::core::RgbdCameraPyramid camera(640, 480, intrinsics);

  // setup tracker configuration
  dvo_ros::CameraDenseTrackerConfig dynreconfg_cfg = dvo_ros::CameraDenseTrackerConfig::__getDefault__();
  dynreconfg_cfg.__fromServer__(nh_private_);

  dvo::DenseTracker::Config cfg = dvo::DenseTracker::getDefaultConfig();
  dvo_ros::util::updateConfigFromDynamicReconfigure(dynreconfg_cfg, cfg);

  ROS_WARN_STREAM_NAMED("config", "config: \"" << cfg << "\"");

  // Store attributes of the current experiment if not done yet
  htri_t status = H5Aexists( store_.mainGroup.getId(), "Config" );
  if( status == 0 ) // htri_t 0 means false, not exists
  {
    // If the attribute "Config" does not exist, save it
    H5::Attribute attr = store_.mainGroup.createAttribute(
          "Config", H5::MyPredType::Config, H5::DataSpace(H5S_SCALAR) );
    attr.write( H5::MyPredType::Config, &cfg );
  }

  // setup tracker
  dvo::DenseTracker dense_tracker(cfg);
  camera.build(cfg.getNumLevels());

  // initialize first pose
  Eigen::Affine3d trajectory, relative;

  if(groundtruth_reader_ != 0)
  {
    dvo_benchmark::findClosestEntry(*groundtruth_reader_, rgbdpair_reader_->entry().RgbTimestamp());
    dvo_benchmark::toPoseEigen(groundtruth_reader_->entry(), trajectory);
  }
  else
  {
    trajectory.setIdentity();
  }

  std::string folder = cfg_.RgbdPairFile.substr(0, cfg_.RgbdPairFile.find_last_of("/") + 1);

  std::vector<dvo_benchmark::RgbdPair> pairs;
  rgbdpair_reader_->readAllEntries(pairs);


  // Setup datasets for storage in HDF5
  H5::DataSetStream dsetLevelStats, dsetTimeStats, dsetTimeMatch, dsetTrajPoses, dsetTrajEval;
  // Take dimension from current configuration
  try
  {
  size_t numOfLevels = cfg.FirstLevel - cfg.LastLevel + 1;
  dsetLevelStats = store_.group.createDataSet2D(
        "LevelStats",
        H5::MyPredType::LevelStats,
        numOfLevels, pairs.size() - 1 );
  dsetTimeStats = store_.group.createDataSet2D(
        "TimeStats",
        H5::MyPredType::TimeStats,
        numOfLevels, pairs.size() - 1 );
  dsetTimeMatch = store_.group.createDataSet2D(
        "TimeMatch",
        H5::PredType::NATIVE_DOUBLE,
        1, pairs.size() - 1 );
  dsetTrajPoses = store_.group.createDataSet2D(
        "Trajectory",
        H5::createArrayType2D(H5::PredType::NATIVE_DOUBLE, 4,4),
        1, pairs.size() );
  dsetTrajEval = store_.group.createDataSet2D(
        "TrajectoryTUM",
        H5::PredType::NATIVE_DOUBLE,
        8, pairs.size() - 1 );
  }
  catch (...)
  {
    char name[150];
    H5Iget_name(store_.group.getId(), name, 150);
    if(cfg_.Hdf5Overwrite)
    {
      std::cerr << "WARNING: Some of the datasets already exists in group "
                << std::string(name) << "." << std::endl
                << "The datasets are going to be rewritten."
                << std::endl
                << "Set hdf5_overwrite false to avoid overwriting."
                << std::endl;
      dsetLevelStats = store_.group.openDataSet("LevelStats");
      dsetTimeStats = store_.group.openDataSet("TimeStats");
      dsetTimeMatch = store_.group.openDataSet("TimeMatch");
      dsetTrajPoses = store_.group.openDataSet("Trajectory");
      dsetTrajEval = store_.group.openDataSet("TrajectoryTUM");
    }
    else
    {
      std::cerr << "WARNING: Some of the datasets already exists in group "
                << std::string(name) << "." << std::endl
                << "The execution is going to be terminated."
                << std::endl
                << "Set hdf5_overwrite true to overwrite."
                << std::endl;
      std::terminate();
    }
  }

  // Save first trajectory pose (identity by default)
  dsetTrajPoses << trajectory.data();

  dvo::core::RgbdImagePyramid::Ptr reference, current;

  dvo::util::stopwatch sw_online("online", 1), sw_postprocess("postprocess", 1);
  sw_online.start();
  int frameCounter = 0;
  for(std::vector<dvo_benchmark::RgbdPair>::iterator it = pairs.begin(); ros::ok() && it != pairs.end(); ++it, ++frameCounter)
  {
    reference = current;
    current = load(camera, folder + it->RgbFile(), folder + it->DepthFile());

    if(!reference) continue;

    // pause in the beginning
    renderWhileSwitchAndNotTerminated(visualizer, pause_switch);
    processInput(visualizer);

    if(cfg_.ShowGroundtruth)
    {
      Eigen::Affine3d groundtruth_pose;

      dvo_benchmark::findClosestEntry(*groundtruth_reader_, it->RgbTimestamp());
      dvo_benchmark::toPoseEigen(groundtruth_reader_->entry(), groundtruth_pose);

      visualizer->trajectory("groundtruth")->
          color(dvo::visualization::Color::green()).
          add(groundtruth_pose);

      visualizer->camera("groundtruth")->
          color(dvo::visualization::Color::green()).
          update(current->level(0), groundtruth_pose).
          show(cfg_.ShowEstimate ? dvo::visualization::CameraVisualizer::ShowCamera : dvo::visualization::CameraVisualizer::ShowCameraAndCloud);
    }

    if(cfg_.EstimateRequired())
    {
      static dvo::util::stopwatch sw_match("match", 100);
      double match_time;
      dvo::DenseTracker::Result result; // Stores statistics too
      sw_match.start();
      {
        dense_tracker.match(*reference, *current, result);
        relative = result.Transformation;
      }
      sw_match.stop();
      match_time = sw_match.computeSumAndReset();

      trajectory = trajectory * relative;

      // Store results and statistics
      dsetLevelStats << result.Statistics.Levels;
      dsetTimeStats << result.Statistics.Times;
      dsetTimeMatch << &match_time;
      dsetTrajPoses << trajectory.data();

      if(cfg_.EstimateTrajectory)
      {
        Eigen::Quaterniond q(trajectory.rotation());

        (*trajectory_out_)
            << it->RgbTimestamp() << " "
            << trajectory.translation()(0) << " "
            << trajectory.translation()(1) << " "
            << trajectory.translation()(2) << " "
            << q.x() << " "
            << q.y() << " "
            << q.z() << " "
            << q.w() << " "
            << std::endl;

        // Store as dataset of the .h5 file too
        Eigen::Matrix<double,8,1> traj;
        traj << it->RgbTimestamp().toSec(),
                trajectory.translation()(0),
                trajectory.translation()(1),
                trajectory.translation()(2),
                q.x(), q.y(), q.z(), q.w();
        dsetTrajEval << traj.data();
      }

      if(cfg_.ShowEstimate)
      {
        //visualizer->trajectory("estimate")->
        //    color(dvo::visualization::Color::red()).
        //    add(trajectory);

        visualizer->camera("estimate")->
            color(dvo::visualization::Color::red()).
            update(current->level(0), trajectory).
            show(dvo::visualization::CameraVisualizer::ShowCamera);
      }
    }
    ros::spinOnce();

    //if(cfg_.RenderVideo)
    //{
    //  ((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->render(5);
    //  ((dvo::visualization::PclCameraTrajectoryVisualizer*) visualizer)->visualizer().saveScreenshot(frame_ids.next() + std::string(".png"));
    //}
  }
  sw_online.stop();
  //std::cerr << "input:" << std::endl;
  //std::string tmp;
  //std::cin >> tmp;

  sw_postprocess.start();
  sw_postprocess.stop();

  sw_online.printMean(); sw_postprocess.printMean();

  // keep visualization alive
  if(cfg_.KeepAlive)
  {
    renderWhileSwitchAndNotTerminated(visualizer, dummy_switch);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "benchmark", ros::init_options::AnonymousName);

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  BenchmarkNode benchmark(nh, nh_private);

  if(benchmark.configure())
  {
    benchmark.run();
  }

  return 0;
}
