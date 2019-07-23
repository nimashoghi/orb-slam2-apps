#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sys/stat.h>

#include <opencv2/core/core.hpp>
#include <fmt/core.h>

#include <orb_slam2/System.h>

#include "./util.hpp"

using namespace std;

auto is_number(const string& s)
{
    return !s.empty() && find_if(s.begin(),
        s.end(), [](char c) { return !isdigit(c); }) == s.end();
}

auto now()
{
    return chrono::high_resolution_clock::now();
}

auto stats(const vector<double> &data)
{
    const auto sum = accumulate(data.begin(), data.end(), 0.0f, [](const auto acc, const auto x) { return acc + x; });
    const auto mean = sum / double(data.size());
    const auto std_dev = accumulate(data.begin(), data.end(), 0.0f, [mean](const auto acc, const auto x) { return pow(x - mean, 2); });

    return make_tuple(sum, mean, std_dev);
}

auto run_stereo(const string &name, const string &url_left, const string &url_right, ORB_SLAM2::ORBVocabulary *vocabulary, const string &settings_file)
{
    // Read rectification parameters
    cv::FileStorage settings(settings_file, cv::FileStorage::READ);
    if (!settings.isOpened())
    {
        throw runtime_error{fmt::format("ERROR: Wrong path to settings")};
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    settings["LEFT.K"] >> K_l;
    settings["RIGHT.K"] >> K_r;

    settings["LEFT.P"] >> P_l;
    settings["RIGHT.P"] >> P_r;

    settings["LEFT.R"] >> R_l;
    settings["RIGHT.R"] >> R_r;

    settings["LEFT.D"] >> D_l;
    settings["RIGHT.D"] >> D_r;

    int rows_l = settings["LEFT.height"];
    int cols_l = settings["LEFT.width"];
    int rows_r = settings["RIGHT.height"];
    int cols_r = settings["RIGHT.width"];

    if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
        rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
    {
        throw runtime_error{fmt::format("ERROR: Calibration parameters to rectify stereo are missing!")};
    }

    cv::Mat M1l, M2l, M1r, M2r;
    cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
    cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);

    // Create system system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System system{vocabulary, settings_file, ORB_SLAM2::System::STEREO, false};

    // Vector for tracking time statistics
    map<double, double> times;

    cv::VideoCapture video_capture_left;
    if (is_number(url_left))
    {
        video_capture_left.open(stoi(url_left));
    }
    else
    {
        video_capture_left.open(url_left);
    }

    cv::VideoCapture video_capture_right;
    if (is_number(url_right))
    {
        video_capture_left.open(stoi(url_right));
    }
    else
    {
        video_capture_left.open(url_right);
    }

    while (true)
    {
        double timestamp = chrono::duration_cast<chrono::duration<double>>(chrono::system_clock::now().time_since_epoch()).count();

        cv::Mat left_image, right_image, left_image_rect, right_image_rect;
        video_capture_left.read(left_image);
        video_capture_right.read(right_image);

        cv::remap(left_image, left_image_rect, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(right_image, right_image_rect, M1r, M2r, cv::INTER_LINEAR);

        if (left_image.empty() || right_image.empty())
        {
            throw runtime_error{fmt::format("Failed to load image at {} and {}", url_left, url_right)};
        }

        const auto start = now();
        system.TrackStereo(left_image_rect, right_image_rect, timestamp);
        const auto end = now();

        times[timestamp] = chrono::duration_cast<chrono::duration<double>>(end - start).count();
    }

    system.SaveTrajectoryTUM(fmt::format("/output/{}_keyframe_stereo.csv", name));

    // Stop all threads
    system.Shutdown();

    return times;
}

auto write_times(const string &name, const string &type, map<double, double> &times)
{
    // sort(times.begin(), times.end(), [](const auto lhs, const auto rhs) {
    //     return lhs.first < rhs.first;
    // });

    ofstream stream{fmt::format("/output/{}_times_{}.csv", name, type)};
    auto total_time = 0.0;
    for (const auto &pair : times)
    {
        total_time += pair.second;
        stream << fmt::format("{},{}", pair.first, pair.second) << "\n";
    }
    fmt::print("Time taken to process {}: {}", name, total_time);
}

auto load_file(const string &path_or_url, string prepend_string = "")
{
    if (path_or_url.rfind("http", 0) != 0)
    {
        return path_or_url;
    }

    static const auto tmp_path = [] {
        const auto path = fmt::format("/tmp/{}", random_string(32));
        mkdir(path.c_str(), 0777);
        return path;
    }();
    const auto path = fmt::format("{}/{}{}", tmp_path, prepend_string, path_or_url.substr(path_or_url.rfind("/") + 1));
    const auto command = fmt::format("wget -O {} {}", path, path_or_url);
    if (system(command.c_str()) != 0)
    {
        throw runtime_error{fmt::format("Failed to run {}", command)};
    }

    return path;
}

auto main(int argc, const char **argv) -> int
{
    if (argc < 6)
    {
        fmt::print("Usage: ./app <vocabulary> [<name> <settings> <left_image_url> <right_image_url>]...\n");
        return 1;
    }

    if (access("/output/", F_OK) == -1)
    {
        mkdir("/output/", 0777);
    }

    const string voc_file = load_file(argv[1]);
    auto vocabulary = ORB_SLAM2::System::parse_vocabulary(voc_file);

    const auto get_nth_arg = [argv](const auto n) {
        return argv[n];
    };

    const string name = get_nth_arg(2);
    const string settings_file = load_file(get_nth_arg(3), name);
    const string url_left = get_nth_arg(4);
    const string url_right = get_nth_arg(5);

    fmt::print("[{}] Processing stereo...", name);
    auto stereo_times = run_stereo(name, url_left, url_right, vocabulary.get(), settings_file);
    write_times(name, "stereo", stereo_times);

    return 0;
}
