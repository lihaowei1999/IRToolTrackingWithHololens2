#pragma once
#include "HL2ResearchMode.g.h"
#include "ResearchModeApi.h"
#include "TimeConverter.h"
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <wchar.h>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <future>
#include <cmath>
#include <DirectXMath.h>
#include <vector>
#include <map>
#include <MemoryBuffer.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.Spatial.Preview.h>
#include <winrt/Windows.Media.Devices.Core.h>
#include <winrt/Windows.Media.Capture.Frames.h>
#include <winrt/Windows.Graphics.Imaging.h>
#include <winrt/Windows.Foundation.h>

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <Eigen/Eigen>

namespace winrt::HL2UnityPlugin::implementation
{
    struct HL2ResearchMode : HL2ResearchModeT<HL2ResearchMode>
    {
        struct DetectedMarker
        {
            int32_t markerId;

            Eigen::Vector3f point;
            Eigen::Vector3f dir;

            // Image position
            int x;
            int y;
        };

        struct TrackedMarkerStereo
        {
            int32_t markerId;
            // UpperLeft(xyz), UpperRight(xyz), LowerRight(xyz), LowerLeft(xyz)
            float corners[12] = { 0 };
        };

        HL2ResearchMode();
        static HRESULT CheckCamConsent();
        static HRESULT CheckImuConsent();

        UINT16 GetCenterDepth();
        int GetDepthBufferSize();
        int GetLongDepthBufferSize();
        hstring PrintDepthResolution();
        hstring PrintDepthExtrinsics();
        hstring PrintLongDepthExtrinsics();
        hstring PrintSpatialCameraResolution();
        hstring PrintLFExtrinsics();
        hstring PrintRFExtrinsics();
        hstring PrintLLExtrinsics();
        hstring PrintRRExtrinsics();
        hstring PrintPVIntrinsics();

        void InitializeDepthSensor();
        void InitializeLongDepthSensor();
        void InitializeSpatialCamerasFront();
        void InitializeSpatialCamerasAll();
        void InitializeIMUSensor();
        winrt::Windows::Foundation::IAsyncAction InitializePVCamera(int width = 960);
        winrt::Windows::Foundation::IAsyncAction InitializePVCameraAsync();
        void InitializeArucoTrackingStereo();

        void StartDepthSensorLoop(bool reconstructPointCloud, bool startIRToolTrack);
        void StartLongDepthSensorLoop();
        void StartIMUSensorLoop();
        void StartSpatialCamerasFrontLoop();
        void StartSpatialCamerasAllLoop();

        winrt::Windows::Foundation::IAsyncAction StopAllSensorDevice();

        bool DepthMapUpdated();
        bool DepthMapTextureUpdated();
        bool PointCloudUpdated();
        bool LongDepthMapTextureUpdated();
        bool LongDepthMapUpdated();
        bool IMUSampleUpdated();
        bool LFImageUpdated();
        bool RFImageUpdated();
        bool LLImageUpdated();
        bool RRImageUpdated();
        bool PVImageUpdated();
        bool LongThrowLUTUpdated();
        bool ShortThrowLUTUpdated();
        bool IRToolUpdated();
        bool PVInterrupted();

        void SetReferenceCoordinateSystem(Windows::Perception::Spatial::SpatialCoordinateSystem refCoord);
        void SetPointCloudRoiInSpace(float centerX, float centerY, float centerZ, float boundX, float boundY, float boundZ);
        void SetPointCloudDepthOffset(uint16_t offset);
        com_array<uint8_t> GetDepthMapBuffer();
        com_array<uint8_t> GetDepthMapTextureBuffer();
        com_array<uint8_t> GetShortAbImageBuffer();
        com_array<uint8_t> GetLongDepthMapBuffer();
        com_array<uint8_t> GetLongDepthMapTextureBuffer();
        com_array<uint8_t> GetLFCameraBuffer(int64_t& ts, bool withExtrinsics = false);
        com_array<uint8_t> GetRFCameraBuffer(int64_t& ts, bool withExtrinsics = false);
        com_array<uint8_t> GetLRFCameraBuffer(int64_t& ts_left, int64_t& ts_right, bool withExtrinsics = false);
        com_array<uint8_t> GetLLCameraBuffer();
        com_array<uint8_t> GetRRCameraBuffer();
        com_array<uint8_t> GetPVCameraBuffer();
        com_array<float> GetIMUSample();
        com_array<float> GetPointCloudBuffer();
        com_array<float> GetCenterPoint();
        com_array<float> GetShortThrowLUT();
        com_array<float> GetAhatTransformBuffer();
        com_array<float> GetLongThrowLUT();
        com_array<float> GetPVFloatBuffer();
        com_array<float> GetRigTransformBuffer();
        INT64 GetPVTimestamp();
        INT64 GetLongDepthTimestamp();
        INT64 GetShortDepthTimestamp();
        INT64 GetIMUTimestamp();
        com_array<float> GetArucoCornersStereo(int id);
        float GetSpatialLfFps();
        float GetSpatialRfFps();
        com_array<float> GetIRToolCenters();

    protected:
        void OnFrameArrived(const winrt::Windows::Media::Capture::Frames::MediaFrameReader& sender,
            const winrt::Windows::Media::Capture::Frames::MediaFrameArrivedEventArgs& args);
    private:
        float* m_pointCloud = nullptr;
        int m_pointcloudLength = 0;
        float* m_lut_short, * m_lut_long = nullptr;
        int m_lutLength_short, m_lutLength_long = 0;
        float* m_floatContainer = nullptr;
        float* m_ahatTransform = nullptr;
        int m_containerLength = 0;
        float* m_rigtoWorldTransform = nullptr;
        int m_transformLength = 0;
        UINT8* m_depthMap = nullptr;
        UINT8* m_depthMapTexture = nullptr;
        UINT8* m_shortAbImage = nullptr;
        UINT8* m_longDepthMap = nullptr;
        UINT8* m_longDepthMapTexture = nullptr;
        UINT8* m_LLImage = nullptr;
        UINT8* m_RRImage = nullptr;
        UINT8* m_PVImage = nullptr;
        float* m_imuSample = nullptr;

        float* m_irToolCenters = nullptr;
        int m_irToolCenterSize = 0;

        IResearchModeSensor* m_depthSensor = nullptr;
        IResearchModeCameraSensor* m_pDepthCameraSensor = nullptr;
        IResearchModeSensor* m_longDepthSensor = nullptr;
        IResearchModeCameraSensor* m_pLongDepthCameraSensor = nullptr;
        IResearchModeSensor* m_accelSensor = nullptr;
        IResearchModeAccelSensor* m_pAccelIMUSensor = nullptr;
        IResearchModeSensor* m_gyroSensor = nullptr;
        IResearchModeGyroSensor* m_pGyroIMUSensor = nullptr;
        IResearchModeSensor* m_LFSensor = nullptr;
        IResearchModeCameraSensor* m_LFCameraSensor = nullptr;
        IResearchModeSensor* m_RFSensor = nullptr;
        IResearchModeCameraSensor* m_RFCameraSensor = nullptr;
        IResearchModeSensor* m_LLSensor = nullptr;
        IResearchModeCameraSensor* m_LLCameraSensor = nullptr;
        IResearchModeSensor* m_RRSensor = nullptr;
        IResearchModeCameraSensor* m_RRCameraSensor = nullptr;
        ResearchModeSensorResolution m_depthResolution;
        ResearchModeSensorResolution m_longDepthResolution;
        ResearchModeSensorResolution m_SpatialCameraResolution;
        IResearchModeSensorDevice* m_pSensorDevice = nullptr;
        std::vector<ResearchModeSensorDescriptor> m_sensorDescriptors;
        IResearchModeSensorDeviceConsent* m_pSensorDeviceConsent = nullptr;
        Windows::Perception::Spatial::SpatialLocator m_locator = 0;
        Windows::Perception::Spatial::SpatialCoordinateSystem m_refFrame = nullptr;
        std::atomic_int m_depthBufferSize, m_shortAbImageBufferSize = 0;
        std::atomic_int m_longDepthBufferSize = 0;
        std::atomic_int m_spatialBufferSize = 0;
        std::atomic_int m_PVbufferSize = 0;
        std::atomic_uint16_t m_centerDepth = 0;
        float m_centerPoint[3]{ 0,0,0 };
        std::atomic_bool m_depthSensorLoopStarted = false;
        std::atomic_bool m_longDepthSensorLoopStarted = false;
        std::atomic_bool m_imuSensorLoopStarted = false;
        std::atomic_bool m_spatialCamerasFrontLoopStarted = false;
        std::atomic_bool m_spatialCamerasAllLoopStarted = false;
        std::atomic_bool m_depthMapUpdated, m_depthMapTextureUpdated = false;
        std::atomic_bool m_longDepthMapTextureUpdated = false;
        std::atomic_bool m_longDepthMapUpdated = false;
        std::atomic_bool m_pointCloudUpdated = false;
        std::atomic_bool m_useRoiFilter = false;
        std::atomic_bool m_LFImageUpdated = false;
        std::atomic_bool m_RFImageUpdated = false;
        std::atomic_bool m_LLImageUpdated = false;
        std::atomic_bool m_RRImageUpdated = false;
        std::atomic_bool m_PVImageUpdated = false;
        std::atomic_bool m_imuSampleUpdated = false;
        std::atomic_bool m_irToolCentersUpdated = false;

        std::atomic_bool m_LUTGenerated_long, m_LUTGenerated_short = false;
        std::atomic_bool m_PVIntrinsicsRetrived = false;
        std::atomic_bool m_startArucoTrackingStereo = false;
        std::atomic_bool m_startArucoTrackingRGB = false;
        std::atomic_bool m_reconstructShortThrowPointCloud = false;
        std::atomic_bool m_startIRToolTracking = false;

        cv::Ptr<cv::aruco::DetectorParameters> m_arucoDetectorParameters;
        cv::Ptr<cv::aruco::Dictionary> m_arucoDictionary;

        float m_roiBound[3]{ 0,0,0 };
        float m_roiCenter[3]{ 0,0,0 };
        static void DepthSensorLoop(HL2ResearchMode* pHL2ResearchMode);
        static void LongDepthSensorLoop(HL2ResearchMode* pHL2ResearchMode);
        static void IMUSensorLoop(HL2ResearchMode* pHL2ResearchMode);
        static void SpatialCamerasFrontLoop(HL2ResearchMode* pHL2ResearchMode);
        static void SpatialCamerasAllLoop(HL2ResearchMode* pHL2ResearchMode);
        std::map<int32_t, TrackedMarkerStereo> TrackArUcoMarkersStereo(IResearchModeSensorFrame* pLeftSensorFrame, IResearchModeSensorFrame* pRightSensorFrame);
        static void CamAccessOnComplete(ResearchModeSensorConsent consent);
        static void ImuAccessOnComplete(ResearchModeSensorConsent consent);
        std::string MatrixToString(DirectX::XMFLOAT4X4 mat);
        static DirectX::XMMATRIX HL2ResearchMode::SpatialLocationToDxMatrix(Windows::Perception::Spatial::SpatialLocation location);
        DirectX::XMFLOAT4X4 m_depthCameraPose;
        DirectX::XMMATRIX m_depthCameraPoseInvMatrix;
        DirectX::XMFLOAT4X4 m_longDepthCameraPose;
        DirectX::XMMATRIX m_longDepthCameraPoseInvMatrix;
        DirectX::XMFLOAT4X4 m_accelPose;
        DirectX::XMMATRIX m_accelInvMatrix;
        DirectX::XMFLOAT4X4 m_gyroPose;
        DirectX::XMMATRIX m_gyroInvMatrix;
        DirectX::XMFLOAT4X4 m_LFCameraPose;
        DirectX::XMMATRIX m_LFCameraPoseInvMatrix;
        DirectX::XMFLOAT4X4 m_RFCameraPose;
        DirectX::XMMATRIX m_RFCameraPoseInvMatrix;
        DirectX::XMFLOAT4X4 m_LLCameraPose;
        DirectX::XMMATRIX m_LLCameraPoseInvMatrix;
        DirectX::XMFLOAT4X4 m_RRCameraPose;
        DirectX::XMMATRIX m_RRCameraPoseInvMatrix;
        std::thread* m_pDepthUpdateThread;
        std::thread* m_pLongDepthUpdateThread;
        std::thread* m_pIMUUpdateThread;
        std::thread* m_pSpatialCamerasFrontUpdateThread;
        std::thread* m_pSpatialCamerasAllUpdateThread;
        //static long long checkAndConvertUnsigned(UINT64 val);
        struct DepthCamRoi {
            float kRowLower = 0.2;
            float kRowUpper = 0.5;
            float kColLower = 0.3;
            float kColUpper = 0.7;
            UINT16 depthNearClip = 200; // Unit: mm
            UINT16 depthFarClip = 800;
        } depthCamRoi;
        UINT16 m_depthOffset = 0;

        winrt::Windows::Foundation::IAsyncAction m_PVCameraOperation = nullptr;
        winrt::Windows::Media::Capture::MediaCapture mediaCapture = nullptr;
        winrt::Windows::Media::Capture::Frames::MediaFrameReader m_mediaFrameReader = nullptr;
        winrt::event_token m_OnFrameArrivedRegistration;
        int kImageWidth = 960;
        const wchar_t kSensorName[3] = L"PV";
        std::stringstream m_PVCameraIntrinsics;
        std::map<int32_t, TrackedMarkerStereo> m_arucoResult;

        std::shared_mutex m_arucoMutex;
        std::shared_mutex m_frameMutex;
        std::shared_mutex mu;
        UINT64 m_prevTimestamp = 0;
        UINT64 m_prevIMUTimestamp = 0;
        long long m_latestLongDepthTimestamp, m_latestShortDepthTimestamp = 0;
        long long m_latestPVTimestamp = 0;
        long long m_latestIMUTimestamp = 0;
        winrt::Windows::Media::Capture::Frames::MediaFrameReference m_latestFrame = nullptr;
        std::atomic_bool m_pvFrameArrived = false;
        std::atomic_bool m_pvFrameInterrupted = false;

        // writing thread
        static void CameraWriteThread(HL2ResearchMode* pProcessor);
        std::thread* m_pWriteThread = nullptr;
        bool m_fExit = false;

        /*struct PvFrame
        {
            SoftwareBitmap softwareBitmap;
            SpatialCoordinateSystem imgCoord;
            long long timestamp_rel;
            float fx, fy, px, py;
        } m_latestFrame;*/

        struct Frame
        {
            UINT64 timestamp; // QPC 
            int64_t timestamp_ft; // FileTime
            UINT8* image = nullptr;
            float* extrin = nullptr;
        };

        struct SpatialCameraFrame
        {
            Frame LFFrame;
            Frame RFFrame;
            Frame LLFrame;
            Frame RRFrame;
        } m_lastSpatialFrame;

        std::atomic<float> spatialLfFps, spatialRfFps = 0.0;
        UINT64 lastLfTs, lastRfTs = 0;
    };
}
namespace winrt::HL2UnityPlugin::factory_implementation
{
    struct HL2ResearchMode : HL2ResearchModeT<HL2ResearchMode, implementation::HL2ResearchMode>
    {
    };
}
