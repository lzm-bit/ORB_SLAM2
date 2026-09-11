#ifndef ORBSLAMPYTHON_H
#define ORBSLAMPYTHON_H

#include <memory>
#include <Python.h>
#include <boost/python.hpp>
#include <numpy/arrayobject.h>
#include <opencv2/opencv.hpp>
#include <System.h>
#include <DynamicDetection.h>
#include <Tracking.h>

class ORBSlamPython
{
public:
    ORBSlamPython(std::string vocabFile, std::string settingsFile, std::string trajPath,
        ORB_SLAM2::System::eSensor sensorMode = ORB_SLAM2::System::eSensor::RGBD, const bool useViewer = true, const bool turnOffLC = false, const int initFr = 0, std::string sequence = std::string());
    ORBSlamPython(const char* vocabFile, const char* settingsFile, const char* trajPath,
        ORB_SLAM2::System::eSensor sensorMode = ORB_SLAM2::System::eSensor::RGBD, const bool useViewer = true, const bool turnOffLC = false, const int initFr = 0, const char* sequence = "");
    ~ORBSlamPython();
    
    bool initialize();
    bool isRunning();
    // bool isLastFrameKeyframe();
    bool loadAndProcessMono(std::string imageFile, double timestamp);
    bool processMono(cv::Mat image, double timestamp);
    bool loadAndProcessStereo(std::string leftImageFile, std::string rightImageFile, double timestamp);
    bool processStereo(cv::Mat leftImage, cv::Mat rightImage, double timestamp);
    // bool loadAndProcessRGBD(std::string imageFile, std::string depthImageFile, cv::Mat maskImage, double timestamp);
    boost::python::tuple loadAndProcessRGBD(std::string imageFile, std::string depthImageFile, cv::Mat maskImage, double timestamp);
    boost::python::tuple processRGBD(cv::Mat image, cv::Mat depthImage, cv::Mat maskImage, double timestamp);
    void reset();
    void shutdown();
    ORB_SLAM2::Tracking::eTrackingState getTrackingState() const;
    unsigned int getLastBigChangeIdx() const;
    boost::python::tuple getKeyframe() const;
    boost::python::list getTrackedMappoints() const;
    boost::python::tuple getGoodMappoints() const;
    bool saveSettings(boost::python::dict settings) const;
    boost::python::dict loadSettings() const;
    void setMode(ORB_SLAM2::System::eSensor mode);
    void setRGBMode(bool rgb);
    void setUseViewer(bool useViewer);
    void predictDynamicInstance(const boost::python::dict& data);
    std::vector<std::vector<Eigen::Vector3f>> extractDynamicBox(const boost::python::dict& data);

    static bool saveSettingsFile(boost::python::dict settings, std::string settingsFilename);
    static boost::python::dict loadSettingsFile(std::string settingsFilename);
    cv::Mat numpyFloat32ToCvMatCopy(const boost::python::object& obj, const std::string& name);

private:
    std::string vocabluaryFile;
    std::string settingsFile;
    std::string trajectoryPath;
    std::string sequence;
    ORB_SLAM2::System::eSensor sensorMode;
    std::shared_ptr<ORB_SLAM2::System> system;
    // std::list<KeyFrame*> mlNewKeyFrames;
    bool bUseViewer;
    bool bturnOffLC;
    bool bUseRGB;
    int initFr;
};

// Helpers for reading cv::FileNode objects into python objects.
boost::python::list readSequence(cv::FileNode fn, int depth=10);
boost::python::dict readMap(cv::FileNode fn, int depth=10);

#endif // ORBSLAMPYTHON_H
