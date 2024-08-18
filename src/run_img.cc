#include "util/image_util.h"
#include "pangolin_viewer/viewer.h"
#include "stella_vslam/system.h"
#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/config.h"
#include "stella_vslam/camera/base.h"
#include "stella_vslam/util/yaml.h"

#include <iostream>
#include <chrono>
#include <fstream>
#include <numeric>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>

#include <ghc/filesystem.hpp>
namespace fs = ghc::filesystem;

#include <pangolin/pangolin.h>
#include <vector>
#include <string>
struct Point3D {
    float x, y, z;
};

void WritePointCloudToPLY(const std::string& filename, const std::vector<Point3D>& points) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        throw std::runtime_error("Could not open file for writing.");
    }
    // Write PLY header
    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex " << points.size() << "\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "end_header\n";
    // Write point data
    for (const auto& point : points) {
        ofs << point.x << " " << point.y << " " << point.z << "\n";
    }

    ofs.close();
}


int mono_tracking(const std::shared_ptr<stella_vslam::system>& slam,
                  const std::shared_ptr<stella_vslam::config>& cfg,
                  const std::string& image_dir_path
                  ) {

    const image_sequence sequence(image_dir_path);
    const auto frames = sequence.get_frames();

    std::shared_ptr<pangolin_viewer::viewer> viewer;
    
    viewer = std::make_shared<pangolin_viewer::viewer>(
        stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "PangolinViewer"),
        slam,
        slam->get_frame_publisher(),
        slam->get_map_publisher());
    
    std::vector<double> track_times;
    track_times.reserve(frames.size());
    double timestamp;
    
    std::thread thread([&]() {
        for (unsigned int i = 0; i < frames.size(); ++i) {


            const auto& frame = frames.at(i);
            const auto img = cv::imread(frame.img_path_, cv::IMREAD_UNCHANGED);
            const auto tp_1 = std::chrono::steady_clock::now();

            
            if (!img.empty()) {
                cv::Mat emptyMat;
                slam->feed_monocular_frame(img, timestamp , emptyMat );
            }

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
                if ( i < frames.size() - 1) {

                const auto wait_time = 1.0 / slam->get_camera()->fps_ - track_time;
                if (0.0 < wait_time) {
                    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
                }
            }

            timestamp += 1.0 / slam->get_camera()->fps_;

            // if termination of slam is requested 
            if (slam->terminate_is_requested()) {
                break;
            }

        }

        // wait until the loop BA is finished
        while (slam->loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
    });
        viewer->run();
    

    thread.join();

    // shutdown the slam process
    slam->shutdown();

    
        //  trajectories  
        slam->save_frame_trajectory( "./frame_trajectory.txt", "TUM");
        // slam->save_keyframe_trajectory("./keyframe_trajectory.txt", "TUM");


    return EXIT_SUCCESS;
}

int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    backward::SignalHandling sh;
#endif

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto img_dir_path = op.add<popl::Value<std::string>>("d", "img-dir", "directory path which contains images");
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    auto log_level = op.add<popl::Value<std::string>>("", "log-level", "log level", "info");

    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!op.unknown_options().empty()) {
        for (const auto& unknown_option : op.unknown_options()) {
            std::cerr << "unknown_options: " << unknown_option << std::endl;
        }
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!vocab_file_path->is_set() || !img_dir_path->is_set() || !config_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    
    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    spdlog::set_level(spdlog::level::from_str(log_level->value()));

    // load configuration
    std::shared_ptr<stella_vslam::config> cfg;
    try {
        cfg = std::make_shared<stella_vslam::config>(config_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif

    double timestamp = 0.0;
    std::cerr << "--start-timestamp is not set. using system timestamp." << std::endl;
    
    std::chrono::system_clock::time_point start_time_system = std::chrono::system_clock::now();
    timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(start_time_system.time_since_epoch()).count();
   
    // build slam system
    auto slam = std::make_shared<stella_vslam::system>(cfg, vocab_file_path->value());
    bool need_initialize = true;
    slam->startup(need_initialize);

    // run tracking
    int ret;
    if (slam->get_camera()->setup_type_ == stella_vslam::camera::setup_type_t::Monocular) {
        ret = mono_tracking(slam,
                            cfg,
                            img_dir_path->value()
                            );
    }
    else {
        throw std::runtime_error("Invalid setup type: " + slam->get_camera()->get_setup_type_string());
    }

auto map_db = slam->get_map_db();
std::vector<Point3D> points;
if (map_db) {
    const auto landmarks = map_db->get_all_landmarks();
    for (const auto& landmark : landmarks) {
        const auto& pos = landmark->get_pos_in_world();
        points.push_back({pos(0), pos(1), pos(2)});
    }
}
WritePointCloudToPLY("./pointcloud.ply", points);

    return ret;
}