#define PY_ARRAY_UNIQUE_SYMBOL pbcvt_ARRAY_API
#include <opencv2/core/core.hpp>
#include <pyboostcvconverter.hpp>
#include <KeyFrame.h>
#include <Converter.h>
#include <Tracking.h>
#include <MapPoint.h>
#include "ORBSlamPython.h"

static void* init_ar() {
    Py_Initialize();

    import_array();
    return NULL;
}

BOOST_PYTHON_MODULE(orbslam2)
{
    init_ar();

    boost::python::to_python_converter<cv::Mat, pbcvt::matToNDArrayBoostConverter>();
    pbcvt::matFromNDArrayBoostConverter();

    boost::python::enum_<ORB_SLAM2::Tracking::eTrackingState>("TrackingState")
        .value("SYSTEM_NOT_READY", ORB_SLAM2::Tracking::eTrackingState::SYSTEM_NOT_READY)
        .value("NO_IMAGES_YET", ORB_SLAM2::Tracking::eTrackingState::NO_IMAGES_YET)
        .value("NOT_INITIALIZED", ORB_SLAM2::Tracking::eTrackingState::NOT_INITIALIZED)
        .value("OK", ORB_SLAM2::Tracking::eTrackingState::OK)
        .value("LOST", ORB_SLAM2::Tracking::eTrackingState::LOST);

    boost::python::enum_<ORB_SLAM2::System::eSensor>("Sensor")
        .value("MONOCULAR", ORB_SLAM2::System::eSensor::MONOCULAR)
        .value("STEREO", ORB_SLAM2::System::eSensor::STEREO)
        .value("RGBD", ORB_SLAM2::System::eSensor::RGBD);

    boost::python::class_<ORBSlamPython, boost::noncopyable>("System", boost::python::init<const char*, const char*, const char*, boost::python::optional<ORB_SLAM2::System::eSensor, bool, bool, int, const char*>>())
        .def(boost::python::init<std::string, std::string, std::string, boost::python::optional<ORB_SLAM2::System::eSensor, bool, bool, int, std::string>>())
        .def("initialize", &ORBSlamPython::initialize)
        .def("load_and_process_mono", &ORBSlamPython::loadAndProcessMono)
        .def("process_image_mono", &ORBSlamPython::processMono)
        .def("load_and_process_stereo", &ORBSlamPython::loadAndProcessStereo)
        .def("process_image_stereo", &ORBSlamPython::processStereo)
        .def("load_and_process_rgbd", &ORBSlamPython::loadAndProcessRGBD)
        .def("process_image_rgbd", &ORBSlamPython::processRGBD)
        .def("shutdown", &ORBSlamPython::shutdown)
        .def("is_running", &ORBSlamPython::isRunning)
        .def("reset", &ORBSlamPython::reset)
        .def("set_mode", &ORBSlamPython::setMode)
        .def("set_use_viewer", &ORBSlamPython::setUseViewer)
        .def("set_dynamic_instance", &ORBSlamPython::predictDynamicInstance)
        .def("get_keyframe", &ORBSlamPython::getKeyframe)
        .def("get_tracked_mappoints", &ORBSlamPython::getTrackedMappoints)
        .def("get_tracking_state", &ORBSlamPython::getTrackingState)
        .def("save_settings", &ORBSlamPython::saveSettings)
        .def("load_settings", &ORBSlamPython::loadSettings)
        .def("save_settings_file", &ORBSlamPython::saveSettingsFile)
        .staticmethod("save_settings_file")
        .def("load_settings_file", &ORBSlamPython::loadSettingsFile)
        .staticmethod("load_settings_file");
}

ORBSlamPython::ORBSlamPython(std::string vocabFile, std::string settingsFile, std::string trajPath, ORB_SLAM2::System::eSensor sensorMode, const bool useViewer, const bool turnOffLC, const int initFr, std::string sequence)
    : vocabluaryFile(vocabFile),
    settingsFile(settingsFile),
    sensorMode(sensorMode),
    trajectoryPath(trajPath),
    system(nullptr),
    bUseViewer(useViewer),
    bturnOffLC(turnOffLC),
    bUseRGB(true),
    initFr(initFr),
    sequence(sequence)
{

}

ORBSlamPython::ORBSlamPython(const char* vocabFile, const char* settingsFile, const char* trajPath, ORB_SLAM2::System::eSensor sensorMode, const bool useViewer, const bool turnOffLC, const int initFr, const char* sequence)
    : vocabluaryFile(vocabFile),
    settingsFile(settingsFile),
    sensorMode(sensorMode),
    trajectoryPath(trajPath),
    system(nullptr),
    bUseViewer(useViewer),
    bturnOffLC(turnOffLC),
    bUseRGB(true),
    initFr(initFr),
    sequence(sequence)
{

}

ORBSlamPython::~ORBSlamPython()
{
    if (system)
    {
        system->Shutdown();
    }
}

bool ORBSlamPython::initialize()
{
    system = std::make_shared<ORB_SLAM2::System>(vocabluaryFile, settingsFile, trajectoryPath, sensorMode, bUseViewer);
    return true;
}

bool ORBSlamPython::isRunning()
{
    return system != nullptr;
}

void ORBSlamPython::reset()
{
    if (system)
    {
        system->Reset();
    }
}

bool ORBSlamPython::loadAndProcessMono(std::string imageFile, double timestamp)
{
    if (!system)
    {
        return false;
    }
    cv::Mat im = cv::imread(imageFile, cv::IMREAD_COLOR);
    if (bUseRGB)
    {
        cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
    }
    return this->processMono(im, timestamp);
}

bool ORBSlamPython::processMono(cv::Mat image, double timestamp)
{
    if (!system || !image.data)
    {
        return false;
    }

    cv::Mat pose = system->TrackMonocular(image, timestamp);
    //return !pose.empty();
    return system->GetTrackingState() == ORB_SLAM2::Tracking::eTrackingState::OK;
}

bool ORBSlamPython::loadAndProcessStereo(std::string leftImageFile, std::string rightImageFile, double timestamp)
{
    if (!system)
    {
        return false;
    }
    cv::Mat leftImage = cv::imread(leftImageFile, cv::IMREAD_COLOR);
    cv::Mat rightImage = cv::imread(rightImageFile, cv::IMREAD_COLOR);
    if (bUseRGB) {
        cv::cvtColor(leftImage, leftImage, cv::COLOR_BGR2RGB);
        cv::cvtColor(rightImage, rightImage, cv::COLOR_BGR2RGB);
    }
    return this->processStereo(leftImage, rightImage, timestamp);
}

bool ORBSlamPython::processStereo(cv::Mat leftImage, cv::Mat rightImage, double timestamp)
{
    if (!system | !leftImage.data | !rightImage.data)
    {
        return false;
    }
    cv::Mat pose = system->TrackStereo(leftImage, rightImage, timestamp);
    return system->GetTrackingState() == ORB_SLAM2::Tracking::eTrackingState::OK;
}

// bool ORBSlamPython::loadAndProcessRGBD(std::string imageFile, std::string depthImageFile, cv::Mat maskImage, double timestamp)
// {
//     if (!system)
//     {
//         return false;
//     }
//     cv::Mat im = cv::imread(imageFile, cv::IMREAD_COLOR);
//     if (bUseRGB)
//     {
//         cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
//     }
//     cv::Mat imDepth = cv::imread(depthImageFile, cv::IMREAD_UNCHANGED);
//
//     // std::string maskImageFile = "/home/lzm/Project2/Open-Vocabulary-dynamic-Scene-Graph/data/fr3_walking_rpy/mask/" + std::to_string(timestamp) + ".png";
//     // cv::Mat imMask = cv::imread(maskImageFile, cv::IMREAD_UNCHANGED);
//     return this->processRGBD(im, imDepth, maskImage, timestamp);
// }

boost::python::tuple ORBSlamPython::loadAndProcessRGBD(std::string imageFile, std::string depthImageFile, cv::Mat maskImage, double timestamp)
{
    if (!system)
    {
        return boost::python::tuple();
    }
    cv::Mat im = cv::imread(imageFile, cv::IMREAD_COLOR);
    if (bUseRGB)
    {
        cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
    }
    cv::Mat imDepth = cv::imread(depthImageFile, cv::IMREAD_UNCHANGED);
    // std::string maskImageFile = "/home/lzm/Project2/Open-Vocabulary-dynamic-Scene-Graph/data/fr3_walking_rpy/mask/" + std::to_string(timestamp) + ".png";
    // cv::Mat imMask = cv::imread(maskImageFile, cv::IMREAD_UNCHANGED);
    return this->processRGBD(im, imDepth, maskImage, timestamp);
}

boost::python::tuple ORBSlamPython::processRGBD(cv::Mat image, cv::Mat depthImage, cv::Mat maskImage, double timestamp)
{
    if (!system | !image.data | !depthImage.data | !maskImage.data)
    {
        return boost::python::tuple();
    }
    cv::Mat Tcw = system->TrackRGBD(image, depthImage, maskImage, timestamp);

    if (Tcw.empty()) {
        std::cout<< "orbslam2 error: " << system->GetTrackingState() << std::endl;
        return boost::python::make_tuple(
            timestamp,
            false,
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        );
    }
    cv::Mat Twc = Tcw.inv();

    const Eigen::Matrix3d Rwc = ORB_SLAM2::Converter::toMatrix3d(Twc.rowRange(0,3).colRange(0,3).clone());
    const Eigen::Vector3d twc = ORB_SLAM2::Converter::toVector3d(Twc.rowRange(0,3).col(3).clone());
    return boost::python::make_tuple(
        timestamp,
        system->GetTrackingState() == ORB_SLAM2::Tracking::eTrackingState::OK,
        Rwc(0, 0), Rwc(0, 1), Rwc(0, 2), twc(0),
        Rwc(1, 0), Rwc(1, 1), Rwc(1, 2), twc(1),
        Rwc(2, 0), Rwc(2, 1), Rwc(2, 2), twc(2)
    );
    // return system->GetTrackingState() == ORB_SLAM2::Tracking::eTrackingState::OK;
}

void ORBSlamPython::shutdown()
{
    if (system)
    {
        system->Shutdown();
        system.reset();
    }
}

ORB_SLAM2::Tracking::eTrackingState ORBSlamPython::getTrackingState() const
{
    if (system)
    {
        return static_cast<ORB_SLAM2::Tracking::eTrackingState>(system->GetTrackingState());
    }
    return ORB_SLAM2::Tracking::eTrackingState::SYSTEM_NOT_READY;
}

boost::python::list ORBSlamPython::getTrackedMappoints() const
{
    if (!system)
    {
        return boost::python::list();
    }

    vector<ORB_SLAM2::MapPoint*> Mps = system->GetTrackedMapPoints();// This call only get map points of current frame.

    boost::python::list map_points;
    for(size_t i=0; i<Mps.size(); i++)    {
        if (Mps[i] != NULL)
        {
        cv::Mat wp = Mps[i]->GetWorldPos();
        map_points.append(boost::python::make_tuple(
                wp.at<float>(0),
                wp.at<float>(1),
                wp.at<float>(2)
            ));
        }
    }

    return map_points;
}

void ORBSlamPython::setMode(ORB_SLAM2::System::eSensor mode)
{
    sensorMode = mode;
}

void ORBSlamPython::setUseViewer(bool useViewer)
{
    bUseViewer = useViewer;
}

void ORBSlamPython::setRGBMode(bool rgb)
{
    bUseRGB = rgb;
}

bool ORBSlamPython::saveSettings(boost::python::dict settings) const
{
    return ORBSlamPython::saveSettingsFile(settings, settingsFile);
}

boost::python::dict ORBSlamPython::loadSettings() const
{
    return ORBSlamPython::loadSettingsFile(settingsFile);
}

boost::python::dict ORBSlamPython::loadSettingsFile(std::string settingsFilename)
{
    cv::FileStorage fs(settingsFilename.c_str(), cv::FileStorage::READ);
    cv::FileNode root = fs.root();
    if (root.isMap())
    {
        return readMap(root);
    }
    else if (root.isSeq())
    {
        boost::python::dict settings;
        settings["root"] = readSequence(root);
        return settings;
    }
    return boost::python::dict();
}

bool ORBSlamPython::saveSettingsFile(boost::python::dict settings, std::string settingsFilename)
{
    cv::FileStorage fs(settingsFilename.c_str(), cv::FileStorage::WRITE);

    boost::python::list keys = settings.keys();
    for (int index = 0; index < boost::python::len(keys); ++index)
    {
        boost::python::extract<std::string> extractedKey(keys[index]);
        if (!extractedKey.check())
        {
            continue;
        }
        std::string key = extractedKey;

        boost::python::extract<int> intValue(settings[key]);
        if (intValue.check())
        {
            fs << key << int(intValue);
            continue;
        }

        boost::python::extract<float> floatValue(settings[key]);
        if (floatValue.check())
        {
            fs << key << float(floatValue);
            continue;
        }

        boost::python::extract<std::string> stringValue(settings[key]);
        if (stringValue.check())
        {
            fs << key << std::string(stringValue);
            continue;
        }
    }

    return true;
}

cv::Mat ORBSlamPython::numpyFloat32ToCvMatCopy(const boost::python::object& obj,
                                const std::string& name) {
    namespace bp = boost::python;

    if (!PyArray_Check(obj.ptr())) {
        std::cerr << "[C++] Error: '" << name << "' is not a NumPy array!" << std::endl;
        return cv::Mat();
    }

    PyArrayObject* arr = reinterpret_cast<PyArrayObject*>(obj.ptr());

    if (PyArray_NDIM(arr) != 2) {
        std::cerr << "[C++] Error: '" << name << "' is not a 2D array!" << std::endl;
        return cv::Mat();
    }

    if (PyArray_TYPE(arr) != NPY_FLOAT32) {
        std::cerr << "[C++] Error: '" << name << "' must be float32!" << std::endl;
        return cv::Mat();
    }

    int height = static_cast<int>(PyArray_DIM(arr, 0));
    int width  = static_cast<int>(PyArray_DIM(arr, 1));

    float* ptr = static_cast<float*>(PyArray_DATA(arr));

    cv::Mat mat(height, width, CV_32FC1, ptr);

    return mat.clone();
}

std::vector<std::vector<Eigen::Vector3f>> ORBSlamPython::extractDynamicBox(const boost::python::dict& data) {
    if (!data.contains("dynamic_box")) {
        PyErr_SetString(PyExc_KeyError, "Missing 'dynamic_box' key in dictionary!");
        boost::python::throw_error_already_set();
    }

    std::vector<std::vector<Eigen::Vector3f>> dynamic_boxes;
    boost::python::object box_obj = data["dynamic_box"];
    if (box_obj.ptr() != Py_None && PyArray_Check(box_obj.ptr())) {
        PyArrayObject* arr = reinterpret_cast<PyArrayObject*>(box_obj.ptr());

        if (PyArray_NDIM(arr) != 3) {
            return dynamic_boxes;
        }

        int num_boxes   = static_cast<int>(PyArray_DIM(arr, 0));
        if (PyArray_TYPE(arr) != NPY_FLOAT64) {
            return dynamic_boxes;
        }

        double* ptr = static_cast<double*>(PyArray_DATA(arr));
        for (int i = 0; i < num_boxes; ++i) {
            std::vector<Eigen::Vector3f> corners;
            corners.reserve(8);
            for (int j = 0; j < 8; ++j) {
                int base = i * 8 * 3 + j * 3;

                double x = ptr[base + 0];
                double y = ptr[base + 1];
                double z = ptr[base + 2];
                corners.emplace_back(x, y, z);
            }
            dynamic_boxes.push_back(corners);
        }
    }
    return dynamic_boxes;
}

void ORBSlamPython::predictDynamicInstance(const boost::python::dict& data) {
    namespace bp = boost::python;

    // 1. Parse timestamp
    if (!data.contains("timestamp")) {
        PyErr_SetString(PyExc_KeyError, "Missing 'timestamp' key in dictionary!");
        bp::throw_error_already_set();
    }
    double timestamp = bp::extract<double>(data["timestamp"]);

    std::vector<std::vector<Eigen::Vector3f>> dynamic_boxes = extractDynamicBox(data);

    // 2. Parse uncertainty_map
    if (!data.contains("uncertainty_map")) {
        PyErr_SetString(PyExc_KeyError, "Missing 'uncertainty_map' key in dictionary!");
        bp::throw_error_already_set();
    }

    cv::Mat uncertainty_mat = numpyFloat32ToCvMatCopy(data["uncertainty_map"], "uncertainty_map");

    std::map<double, cv::Mat> history_maps;
    if (data.contains("history_dynamic_map")) {
        bp::object history_obj = data["history_dynamic_map"];
        if (history_obj.ptr() != Py_None && PyDict_Check(history_obj.ptr())) {
            bp::dict history_dict = bp::extract<bp::dict>(history_obj);
            int history_size = bp::len(history_dict);
            bp::list keys = history_dict.keys();
            for (int i = 0; i < history_size; ++i) {
                bp::object key_obj = keys[i];

                double hist_timestamp = bp::extract<double>(key_obj);
                bp::object map_obj = history_dict[key_obj];

                cv::Mat hist_mat = numpyFloat32ToCvMatCopy(
                    map_obj,
                    "history_dynamic_map value"
                );

                if (hist_mat.empty()) {
                    std::cerr << "[C++] Warning: failed to parse history map at timestamp "
                              << hist_timestamp
                              << ", skip it."
                              << std::endl;
                    continue;
                }

                history_maps[hist_timestamp] = hist_mat;
            }
        }
    }

    // TODO: Pass 'uncertainty_mat' to your ORB-SLAM tracking or mapping module
    // this->processFrameUncertainty(timestamp, uncertainty_mat);

    // ==================== 5. 渲染与可视化逻辑 ====================
    // cv::Mat norm_vis;

    // // 根据输入类型，统一归一化转换为 0~255 的 CV_8UC1 图像
    // if (dtype == NPY_FLOAT32) {
    //     // float32 (0.0~1.0) -> uint8 (0~255)
    //     uncertainty_mat.convertTo(norm_vis, CV_8U, 255.0);
    // } else {
    //     // uint8 原样复制或浅拷贝
    //     norm_vis = uncertainty_mat;
    // }

    // // 应用伪彩色映射 (COLORMAP_JET: 蓝为低概率/静态，红为高概率/动态)
    // cv::Mat heatmap_vis;
    // cv::applyColorMap(norm_vis, heatmap_vis, cv::COLORMAP_JET);

    // // 显示伪彩色热力图
    // cv::imshow("C++ Uncertainty Heatmap", heatmap_vis);

    // 【必须调用】给 OpenCV GUI 刷新窗口事件的时间
    // 设为 1 毫秒（非阻塞显示，适合SLAM等实时系统）
    // cv::waitKey(1);

    // TODO: Pass 'uncertainty_mat' to your ORB-SLAM tracking or mapping module
    // this->processFrameUncertainty(timestamp, uncertainty_mat);

    if (uncertainty_mat.empty()) {
        std::cerr << "[C++] Error: failed to parse uncertainty_map." << std::endl;
    } else {
        system->GetDynamicDetector()->SetUncertaintyMap(timestamp, uncertainty_mat);
    }

    if (!history_maps.empty()) {
        system->GetDynamicDetector()->SetHistoryUncertaintyMap(history_maps);
    }

    if (!dynamic_boxes.empty()) {
        system->GetDynamicDetector()->SetDynamicBox(dynamic_boxes);
    }

    // history_maps TODO ...
}

boost::python::tuple ORBSlamPython::getKeyframe() const
{
    if (!system)
    {
        return boost::python::tuple();
    }
    ORB_SLAM2::KeyFrame* pKF = system->GetDynamicDetector()->GetCurrKF();
    if (pKF == nullptr || pKF->isBad())
    {
        return boost::python::tuple();
    }
    const cv::Mat Twc = pKF->GetPoseInverse();

    const Eigen::Matrix3d Rwc = ORB_SLAM2::Converter::toMatrix3d(Twc.rowRange(0,3).colRange(0,3).clone());
    const Eigen::Vector3d twc = ORB_SLAM2::Converter::toVector3d(Twc.rowRange(0,3).col(3).clone());
    const double timestamp = pKF->mTimeStamp;
    return boost::python::make_tuple(
        timestamp,
        Rwc(0, 0), Rwc(0, 1), Rwc(0, 2), twc(0),
        Rwc(1, 0), Rwc(1, 1), Rwc(1, 2), twc(1),
        Rwc(2, 0), Rwc(2, 1), Rwc(2, 2), twc(2)
    );
}

// ----------- HELPER DEFINITIONS -----------
boost::python::dict readMap(cv::FileNode fn, int depth)
{
    boost::python::dict map;
    if (fn.isMap()) {
        cv::FileNodeIterator it = fn.begin(), itEnd = fn.end();
        for (; it != itEnd; ++it) {
            cv::FileNode item = *it;
            std::string key = item.name();

            if (item.isNone())
            {
                map[key] = boost::python::object();
            }
            else if (item.isInt())
            {
                map[key] = int(item);
            }
            else if (item.isString())
            {
                map[key] = std::string(item);
            }
            else if (item.isReal())
            {
                map[key] = float(item);
            }
            else if (item.isSeq() && depth > 0)
            {
                map[key] = readSequence(item, depth-1);
            }
            else if (item.isMap() && depth > 0)
            {
                map[key] = readMap(item, depth-1);  // Depth-limited recursive call to read inner maps
            }
        }
    }
    return map;
}

boost::python::list readSequence(cv::FileNode fn, int depth)
{
    boost::python::list sequence;
    if (fn.isSeq()) {
        cv::FileNodeIterator it = fn.begin(), itEnd = fn.end();
        for (; it != itEnd; ++it) {
            cv::FileNode item = *it;

            if (item.isNone())
            {
                sequence.append(boost::python::object());
            }
            else if (item.isInt())
            {
                sequence.append(int(item));
            }
            else if (item.isString())
            {
                sequence.append(std::string(item));
            }
            else if (item.isReal())
            {
                sequence.append(float(item));
            }
            else if (item.isSeq() && depth > 0)
            {
                sequence.append(readSequence(item, depth-1)); // Depth-limited recursive call to read nested sequences
            }
            else if (item.isMap() && depth > 0)
            {
                sequence.append(readMap(item, depth-1));
            }
        }
    }
    return sequence;
}
