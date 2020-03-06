#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <ctime>

#include <opencv2/core/core.hpp>
#include <fmt/core.h>

#include <orb_slam2/System.h>

#include <sys/socket.h>
#include <sys/un.h>

#include "./json.hpp"

#include "./util.hpp"

using namespace std;

auto ipc_connect(std::string path)
{
    const auto fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd == -1) { throw std::runtime_error{"Invalid fd"}; }
    std::cout << "Connected to fd " << fd << '\n';

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
    if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) == -1) { throw std::runtime_error{"Could not connect"}; }

    return [fd] (std::string data) {
        auto buffer = data + ";;;;";
        std::cout << "Sent the following packet: " << buffer << '\n';

        const auto length = buffer.length();
        if (write(fd, buffer.data(), length) != length) { throw std::runtime_error{"Invalid write size"}; }
    };
}

auto now()
{
    return std::chrono::high_resolution_clock::now();
}

auto stats(const std::vector<double> &data)
{
    const auto sum = std::accumulate(data.begin(), data.end(), 0.0f, [](const auto acc, const auto x) { return acc + x; });
    const auto mean = sum / double(data.size());
    const auto std_dev = std::accumulate(data.begin(), data.end(), 0.0f, [mean](const auto acc, const auto x) { return pow(x - mean, 2); });

    return std::make_tuple(sum, mean, std_dev);
}

struct image_info
{
    image_info(string left_image, string right_image, const double timestamp) : left_image{move(left_image)}, right_image{move(right_image)}, timestamp{timestamp}
    {
    }

    string left_image;
    string right_image;
    double timestamp;
};

auto load_images(const string &left_path, const string &right_path, const string &times_path)
{
    vector<image_info> images;
    images.reserve(5000);

    ifstream times_file{times_path.c_str()};

    auto begin_time = 0.0;
    while (!times_file.eof())
    {
        string time;
        getline(times_file, time);
        if (time.empty())
        {
            continue;
        }
        auto time_double = stod(time) / 1e9;
        if (begin_time == 0.0)
        {
            begin_time = time_double;
        }

        images.emplace_back(
            fmt::format("{}/{}.png", left_path, time),
            fmt::format("{}/{}.png", right_path, time),
            time_double - begin_time);
    }

    return images;
}

auto run_mono(const string &name, const vector<image_info> &images, ORB_SLAM2::ORBVocabulary *vocabulary, const string &settings_file, const bool is_http)
{
    // Create system system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System system{vocabulary, settings_file, ORB_SLAM2::System::MONOCULAR, false};

    wait_for_times();

    // Vector for tracking time statistics
    map<double, double> times;

    const auto send_packet = ipc_connect("/var/resource-allocator-ipc.sock");

    // how many skipped frames?
    auto skipped = 0;

    auto prev_algorithm_time = now();
    auto prev_timestamp = 0.0;
    for (const auto &image : images)
    {
        const auto time_since_prev = std::chrono::duration_cast<std::chrono::duration<double>>(now() - prev_algorithm_time).count();
        const auto expected_time = image.timestamp - prev_timestamp;

        fmt::print("time_since_prev: {}\nexpected_time: {}\ntime_since_prev - expected_time: {}\n", time_since_prev, expected_time, time_since_prev - expected_time);

        if (time_since_prev > expected_time * 0.005)
        {
            const auto time_diff = (image.timestamp - prev_timestamp) - std::chrono::duration_cast<std::chrono::duration<double>>(now() - prev_algorithm_time).count();
            if (time_diff > 0)
            {
                usleep(1000000 * time_diff);
            }

            nlohmann::json j;
            j["source"] = std::string{"visual_slam"};
            j["timestamp"] = image.timestamp;
            send_packet(j.dump());

            prev_algorithm_time = now();
            prev_timestamp = image.timestamp;
            fmt::print("Skipped frame {}", image.timestamp);
            ++skipped;
            continue;
        }

        cv::Mat left_image;

        if (is_http)
        {
            cv::VideoCapture(image.left_image).read(left_image);
        }
        else
        {
            left_image = cv::imread(image.left_image, CV_LOAD_IMAGE_UNCHANGED);
        }

        if (left_image.empty())
        {
            throw runtime_error{fmt::format("Failed to load image at {}.", image.left_image)};
        }

        const auto start = now();
        system.TrackMonocular(left_image, image.timestamp);
        const auto end = now();

        times[image.timestamp] = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();

        const auto time_diff = (image.timestamp - prev_timestamp) - std::chrono::duration_cast<std::chrono::duration<double>>(now() - prev_algorithm_time).count();
        if (time_diff > 0)
        {
            usleep(1000000 * time_diff);
        }

        prev_algorithm_time = now();
        prev_timestamp = image.timestamp;
    }

    system.SaveKeyFrameTrajectoryTUM(fmt::format("/output/{}_keyframe_mono.csv", name));

    // Stop all threads
    system.Shutdown();

    fmt::print("Total number of skipped frames: {}", skipped);

    return times;
}

auto run_stereo(const string &name, const vector<image_info> &images, ORB_SLAM2::ORBVocabulary *vocabulary, const string &settings_file, const bool is_http)
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

    wait_for_times();

    // Vector for tracking time statistics
    map<double, double> times;

    // for (int ni = 0; ni < image_count; ni++)
    for (const auto &image : images)
    {
        cv::Mat left_image, right_image, left_image_rect, right_image_rect;
        // Read image from file
        if (is_http)
        {
            cv::VideoCapture(image.left_image).read(left_image);
            cv::VideoCapture(image.right_image).read(right_image);
        }
        else
        {
            left_image = cv::imread(image.left_image, CV_LOAD_IMAGE_UNCHANGED);
            right_image = cv::imread(image.right_image, CV_LOAD_IMAGE_UNCHANGED);
        }

        cv::remap(left_image, left_image_rect, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(right_image, right_image_rect, M1r, M2r, cv::INTER_LINEAR);

        if (left_image.empty() || right_image.empty())
        {
            throw runtime_error{fmt::format("Failed to load image at {} and {}", image.left_image, image.right_image)};
        }

        const auto start = now();
        system.TrackStereo(left_image_rect, right_image_rect, image.timestamp);
        const auto end = now();

        times[image.timestamp] = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
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

/*
    auto load_file(const string &path_or_url)
    {
        if (path_or_url.rfind("http", 0) != 0)
        {
            return path_or_url;
        }

        const auto tmp_path = fmt::format("/{}", random_string(32));
        const auto path = fmt::format("{}/{}", tmp_path, path_or_url.substr(path_or_url.rfind("/") + 1));

        const auto command = fmt::format("mkdir {0} && httpfs2 {1} {0}/", tmp_path, path_or_url);
        if (system(command.c_str()) != 0)
        {
            throw runtime_error{fmt::format("Failed to run {}", command)};
        }
        else
        {
            fmt::print("Mounted url '{}' to file '{}'", path_or_url, path);
        }

        return path;
    }
*/

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
    if (argc < 9)
    {
        fmt::print("Usage: ./app <vocabulary> [<name> <type=mono|stereo|both> <settings_mono> <settings_stereo> <left_image_folder> <right_image_folder> <times_file>]...\n");
        return 1;
    }

    if (access("/output/", F_OK) == -1)
    {
        mkdir("/output/", 0777);
    }

    const string voc_file = load_file(argv[1]);
    auto vocabulary = ORB_SLAM2::System::parse_vocabulary(voc_file);

    const auto args_per_run = 7;
    const auto get_nth_arg = [argv](const auto i, const auto n) {
        return argv[args_per_run * i + n];
    };

    const auto n = (argc - 2) / args_per_run;
    for (auto i = 0; i < n; ++i)
    {
        const string name = get_nth_arg(i, 2);
        const string type = get_nth_arg(i, 3);
        if (type != "mono" && type != "stereo" && type != "both")
        {
            throw runtime_error{fmt::format("Type input must be 'mono', 'stereo', or 'both'. Given: '{}'.", type)};
        }
        const string settings_file_mono = load_file(get_nth_arg(i, 4), name);
        const string settings_file_stereo = load_file(get_nth_arg(i, 5), name);
        const string left_image_path = get_nth_arg(i, 6);
        const string right_image_path = get_nth_arg(i, 7);
        const string timestamps = load_file(get_nth_arg(i, 8), name);

        const auto is_http = left_image_path.rfind("http", 0) == 0 && right_image_path.rfind("http", 0) == 0;
        const auto images = load_images(left_image_path, right_image_path, timestamps);
        if (images.size() <= 0)
        {
            throw runtime_error{fmt::format("[{}] ERROR: Failed to load images", name)};
        }

        if (type == "stereo" || type == "both")
        {
            fmt::print("[{}] Processing stereo...", name);
            auto stereo_times = run_stereo(name, images, vocabulary.get(), settings_file_stereo, is_http);
            write_times(name, "stereo", stereo_times);
        }

        if (type == "mono" || type == "both")
        {
            fmt::print("[{}] Processing mono...", name);
            auto mono_times = run_mono(name, images, vocabulary.get(), settings_file_mono, is_http);
            write_times(name, "mono", mono_times);
        }
    }

    return 0;
}
