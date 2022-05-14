#include "pch.h"
#include "HL2ResearchMode.h"
#include "HL2ResearchMode.g.cpp"

#include <winrt/Windows.Foundation.Collections.h>

extern "C"
HMODULE LoadLibraryA(
    LPCSTR lpLibFileName
);

static ResearchModeSensorConsent camAccessCheck;
static HANDLE camConsentGiven;
static ResearchModeSensorConsent imuAccessCheck;
static HANDLE imuConsentGiven;

using namespace DirectX;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::Spatial::Preview;
using namespace winrt::Windows::Foundation::Collections;
using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Media::Devices;
using namespace winrt::Windows::Graphics::Imaging;

//typedef std::chrono::duration<int64_t, std::ratio<1, 10'000'000>> HundredsOfNanoseconds;
//static constexpr UINT64 kMaxLongLong = static_cast<UINT64>(std::numeric_limits<long long>::max());

namespace winrt::HL2UnityPlugin::implementation
{
    HL2ResearchMode::HL2ResearchMode() 
    {
        // Load Research Mode library
        camConsentGiven = CreateEvent(nullptr, true, false, nullptr);
        imuConsentGiven = CreateEvent(nullptr, true, false, nullptr);
        HMODULE hrResearchMode = LoadLibraryA("ResearchModeAPI");
        HRESULT hr = S_OK;

        if (hrResearchMode)
        {
            typedef HRESULT(__cdecl* PFN_CREATEPROVIDER) (IResearchModeSensorDevice** ppSensorDevice);
            PFN_CREATEPROVIDER pfnCreate = reinterpret_cast<PFN_CREATEPROVIDER>(GetProcAddress(hrResearchMode, "CreateResearchModeSensorDevice"));
            if (pfnCreate)
            {
                winrt::check_hresult(pfnCreate(&m_pSensorDevice));
            }
            else
            {
                winrt::check_hresult(E_INVALIDARG);
            }
        }

        // get spatial locator of rigNode
        GUID guid;
        IResearchModeSensorDevicePerception* pSensorDevicePerception;
        winrt::check_hresult(m_pSensorDevice->QueryInterface(IID_PPV_ARGS(&pSensorDevicePerception)));
        winrt::check_hresult(pSensorDevicePerception->GetRigNodeId(&guid));
        pSensorDevicePerception->Release();
        m_locator = SpatialGraphInteropPreview::CreateLocatorForNode(guid);

        size_t sensorCount = 0;

        winrt::check_hresult(m_pSensorDevice->QueryInterface(IID_PPV_ARGS(&m_pSensorDeviceConsent)));
        winrt::check_hresult(m_pSensorDeviceConsent->RequestCamAccessAsync(HL2ResearchMode::CamAccessOnComplete));
        winrt::check_hresult(m_pSensorDeviceConsent->RequestIMUAccessAsync(HL2ResearchMode::ImuAccessOnComplete));

        m_pSensorDevice->DisableEyeSelection();

        winrt::check_hresult(m_pSensorDevice->GetSensorCount(&sensorCount));
        m_sensorDescriptors.resize(sensorCount);
        winrt::check_hresult(m_pSensorDevice->GetSensorDescriptors(m_sensorDescriptors.data(), m_sensorDescriptors.size(), &sensorCount));
    
        std::stringstream ss;
        ss << "Sensor count: " << sensorCount << "\n";
        std::string msg = ss.str();
        std::wstring widemsg = std::wstring(msg.begin(), msg.end());
        OutputDebugString(widemsg.c_str());
    }

    void HL2ResearchMode::InitializeDepthSensor() 
    {
       
        for (auto sensorDescriptor : m_sensorDescriptors)
        {
            if (sensorDescriptor.sensorType == DEPTH_AHAT)
            {
                m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_depthSensor);
                m_depthSensor->QueryInterface(IID_PPV_ARGS(&m_pDepthCameraSensor));
                m_pDepthCameraSensor->GetCameraExtrinsicsMatrix(&m_depthCameraPose);
                m_depthCameraPoseInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_depthCameraPose));
                OutputDebugString(L"Short Depth Sensor initialized.\n");
                break;
            }
        }
    }

    void HL2ResearchMode::InitializeLongDepthSensor()
    {
        for (auto sensorDescriptor : m_sensorDescriptors)
        {
            if (sensorDescriptor.sensorType == DEPTH_LONG_THROW)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_longDepthSensor));
                winrt::check_hresult(m_longDepthSensor->QueryInterface(IID_PPV_ARGS(&m_pLongDepthCameraSensor)));
                winrt::check_hresult(m_pLongDepthCameraSensor->GetCameraExtrinsicsMatrix(&m_longDepthCameraPose));
                m_longDepthCameraPoseInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_longDepthCameraPose));
                OutputDebugString(L"Long Depth Sensor initialized.\n");
                break;
            }
        }
    }

    void HL2ResearchMode::InitializeIMUSensor()
    {
        for (auto sensorDescriptor : m_sensorDescriptors)
        {
            if (sensorDescriptor.sensorType == IMU_ACCEL)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_accelSensor));
                //winrt::check_hresult(m_accelSensor->QueryInterface(IID_PPV_ARGS(&m_pAccelIMUSensor)));
                //winrt::check_hresult(m_pAccelIMUSensor->GetExtrinsicsMatrix(&m_accelPose));
                //m_accelInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_accelPose));
                //break;
            }
            if (sensorDescriptor.sensorType == IMU_GYRO)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_gyroSensor));
        /*        winrt::check_hresult(m_gyroSensor->QueryInterface(IID_PPV_ARGS(&m_pGyroIMUSensor)));
                winrt::check_hresult(m_pGyroIMUSensor->GetExtrinsicsMatrix(&m_gyroPose));
                m_gyroInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_gyroPose));
                break;*/
            }
        }
    }

    void HL2ResearchMode::InitializeSpatialCamerasAll()
    {
        for (auto sensorDescriptor : m_sensorDescriptors)
        {
            if (sensorDescriptor.sensorType == LEFT_FRONT)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_LFSensor));
                winrt::check_hresult(m_LFSensor->QueryInterface(IID_PPV_ARGS(&m_LFCameraSensor)));
                winrt::check_hresult(m_LFCameraSensor->GetCameraExtrinsicsMatrix(&m_LFCameraPose));
                m_LFCameraPoseInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_LFCameraPose));
            }
            if (sensorDescriptor.sensorType == RIGHT_FRONT)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_RFSensor));
                winrt::check_hresult(m_RFSensor->QueryInterface(IID_PPV_ARGS(&m_RFCameraSensor)));
                winrt::check_hresult(m_RFCameraSensor->GetCameraExtrinsicsMatrix(&m_RFCameraPose));
                m_RFCameraPoseInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_RFCameraPose));
            }
            if (sensorDescriptor.sensorType == LEFT_LEFT)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_LLSensor));
                winrt::check_hresult(m_LLSensor->QueryInterface(IID_PPV_ARGS(&m_LLCameraSensor)));
                winrt::check_hresult(m_LLCameraSensor->GetCameraExtrinsicsMatrix(&m_LLCameraPose));
                m_LLCameraPoseInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_LLCameraPose));
            }
            if (sensorDescriptor.sensorType == RIGHT_RIGHT)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_RRSensor));
                winrt::check_hresult(m_RRSensor->QueryInterface(IID_PPV_ARGS(&m_RRCameraSensor)));
                winrt::check_hresult(m_RRCameraSensor->GetCameraExtrinsicsMatrix(&m_RRCameraPose));
                m_RRCameraPoseInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_RRCameraPose));
            }
        }
    }

    void HL2ResearchMode::InitializeSpatialCamerasFront()
    {
        for (auto sensorDescriptor : m_sensorDescriptors)
        {
            if (sensorDescriptor.sensorType == LEFT_FRONT)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_LFSensor));
                winrt::check_hresult(m_LFSensor->QueryInterface(IID_PPV_ARGS(&m_LFCameraSensor)));
                winrt::check_hresult(m_LFCameraSensor->GetCameraExtrinsicsMatrix(&m_LFCameraPose));
                m_LFCameraPoseInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_LFCameraPose));
            }
            if (sensorDescriptor.sensorType == RIGHT_FRONT)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_RFSensor));
                winrt::check_hresult(m_RFSensor->QueryInterface(IID_PPV_ARGS(&m_RFCameraSensor)));
                winrt::check_hresult(m_RFCameraSensor->GetCameraExtrinsicsMatrix(&m_RFCameraPose));
                m_RFCameraPoseInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_RFCameraPose));
            }
        }
    }

    void HL2ResearchMode::InitializeArucoTrackingStereo()
    {
        m_arucoDetectorParameters = cv::aruco::DetectorParameters::create();
        m_arucoDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        m_startArucoTrackingStereo = true;
    }

    winrt::Windows::Foundation::IAsyncAction HL2ResearchMode::InitializePVCamera()
    {
        co_await HL2ResearchMode::InitializePVCameraAsync();
    }

    winrt::Windows::Foundation::IAsyncAction HL2ResearchMode::InitializePVCameraAsync()
    {
        auto mediaFrameSourceGroups{ co_await MediaFrameSourceGroup::FindAllAsync() };

        MediaFrameSourceGroup selectedSourceGroup = nullptr;
        MediaCaptureVideoProfile profile = nullptr;
        MediaCaptureVideoProfileMediaDescription desc = nullptr;
        std::vector<MediaFrameSourceInfo> selectedSourceInfos;

        // Find MediaFrameSourceGroup
        for (const MediaFrameSourceGroup& mediaFrameSourceGroup : mediaFrameSourceGroups)
        {
            auto knownProfiles = MediaCapture::FindKnownVideoProfiles(mediaFrameSourceGroup.Id(), KnownVideoProfile::VideoConferencing);
            for (const auto& knownProfile : knownProfiles)
            {
                for (auto knownDesc : knownProfile.SupportedRecordMediaDescription())
                {
                    std::stringstream ss;
                    ss << "Resolution: " << knownDesc.Width() << "x" << knownDesc.Height() << "@" << knownDesc.FrameRate() << "fps\n" ;
                    std::string msg = ss.str();
                    std::wstring widemsg = std::wstring(msg.begin(), msg.end());
                    OutputDebugString(widemsg.c_str());
                }
            }
        }
        for (const MediaFrameSourceGroup& mediaFrameSourceGroup : mediaFrameSourceGroups)
        {
            auto knownProfiles = MediaCapture::FindKnownVideoProfiles(mediaFrameSourceGroup.Id(), KnownVideoProfile::VideoConferencing);
            for (const auto& knownProfile : knownProfiles)
            {
                for (auto knownDesc : knownProfile.SupportedRecordMediaDescription())
                {
                    if ((knownDesc.Width() == kImageWidth)) // && (std::round(knownDesc.FrameRate()) == 15))
                    {
                        profile = knownProfile;
                        desc = knownDesc;
                        selectedSourceGroup = mediaFrameSourceGroup;
                        break;
                    }
                }
            }
        }
        winrt::check_bool(selectedSourceGroup != nullptr);

        for (auto sourceInfo : selectedSourceGroup.SourceInfos())
        {
            // Workaround since multiple Color sources can be found,
            // and not all of them are necessarily compatible with the selected video profile
            if (sourceInfo.SourceKind() == MediaFrameSourceKind::Color)
            {
                selectedSourceInfos.push_back(sourceInfo);
            }
        }
        winrt::check_bool(!selectedSourceInfos.empty());

        // Initialize a MediaCapture object
        MediaCaptureInitializationSettings settings;
        settings.VideoProfile(profile);
        settings.RecordMediaDescription(desc);
        settings.VideoDeviceId(selectedSourceGroup.Id());
        settings.StreamingCaptureMode(StreamingCaptureMode::Video);
        settings.MemoryPreference(MediaCaptureMemoryPreference::Cpu);
        settings.SharingMode(MediaCaptureSharingMode::SharedReadOnly);
        settings.SourceGroup(selectedSourceGroup);

        mediaCapture = MediaCapture();
        co_await mediaCapture.InitializeAsync(settings);

        //co_await mediaCapture.VideoDeviceController().ExposureControl().SetAutoAsync(false);

        MediaFrameSource selectedSource = nullptr;
        MediaFrameFormat preferredFormat = nullptr;

        for (MediaFrameSourceInfo sourceInfo : selectedSourceInfos)
        {
            auto tmpSource = mediaCapture.FrameSources().Lookup(sourceInfo.Id());
            for (MediaFrameFormat format : tmpSource.SupportedFormats())
            {
                if (format.VideoFormat().Width() == kImageWidth)
                {
                    selectedSource = tmpSource;
                    preferredFormat = format;
                    break;
                }
            }
        }

        winrt::check_bool(preferredFormat != nullptr);

        co_await selectedSource.SetFormatAsync(preferredFormat);
        m_mediaFrameReader = co_await mediaCapture.CreateFrameReaderAsync(selectedSource);
        auto status = co_await m_mediaFrameReader.StartAsync();

        winrt::check_bool(status == MediaFrameReaderStartStatus::Success);

        // reserve for 10 seconds at 30fps
        //m_PVFrameLog.reserve(10 * 30);
        m_pWriteThread = new std::thread(CameraWriteThread, this);
        m_OnFrameArrivedRegistration = m_mediaFrameReader.FrameArrived({ this, &HL2ResearchMode::OnFrameArrived });
    }

    void HL2ResearchMode::CameraWriteThread(HL2ResearchMode* pHL2ResearchMode)
    {
        long long last_timestamp_rel = 0;
        

        while (!pHL2ResearchMode->m_fExit)
        {
            //std::lock_guard<std::mutex> guard(pHL2ResearchMode->m_storageMutex);
            SoftwareBitmap softwareBitmap = nullptr;
            SpatialCoordinateSystem imgCoord = nullptr;
            float fx, fy = 0;
            {
                std::shared_lock<std::shared_mutex> lock(pHL2ResearchMode->m_frameMutex);
                if (pHL2ResearchMode->m_latestFrame == nullptr) continue;
                auto frame = pHL2ResearchMode->m_latestFrame;
                if (!pHL2ResearchMode->m_PVIntrinsicsRetrived && frame.VideoMediaFrame().CameraIntrinsics() != nullptr)
                {
                    pHL2ResearchMode->m_PVCameraIntrinsics << frame.VideoMediaFrame().CameraIntrinsics().PrincipalPoint().x << "," << frame.VideoMediaFrame().CameraIntrinsics().PrincipalPoint().y << ","
                        << frame.VideoMediaFrame().CameraIntrinsics().ImageWidth() << "," << frame.VideoMediaFrame().CameraIntrinsics().ImageHeight();
                    pHL2ResearchMode->m_PVIntrinsicsRetrived = true;
                }

                long long timestamp_rel = frame.SystemRelativeTime().Value().count();
                if (timestamp_rel == last_timestamp_rel) continue;
                last_timestamp_rel = timestamp_rel;
                        
                softwareBitmap = frame.VideoMediaFrame().SoftwareBitmap(); // SoftwareBitmap::Convert(frame.VideoMediaFrame().SoftwareBitmap(), BitmapPixelFormat::Bgra8);
                auto ts = PerceptionTimestampHelper::FromSystemRelativeTargetTime(HundredsOfNanoseconds(timestamp_rel));
                pHL2ResearchMode->m_latestPVTimestamp = ts.TargetTime().time_since_epoch().count();

                fx = frame.VideoMediaFrame().CameraIntrinsics().FocalLength().x;
                fy = frame.VideoMediaFrame().CameraIntrinsics().FocalLength().y;

                imgCoord = frame.CoordinateSystem();
                if (fx == 0 || fy == 0) return;
            }
            // Compose buffer bitmap with timesatmp, transform and focal length
            if (softwareBitmap != nullptr)
            {
                // Get bitmap buffer object of the frame
                BitmapBuffer bitmapBuffer = softwareBitmap.LockBuffer(BitmapBufferAccessMode::Read);
                    
                // Get raw pointer to the buffer object
                uint32_t pixelBufferDataLength = 0;
                uint8_t* pixelBufferData;

                auto spMemoryBufferByteAccess{ bitmapBuffer.CreateReference().as<::Windows::Foundation::IMemoryBufferByteAccess>() };
                winrt::check_hresult(spMemoryBufferByteAccess->GetBuffer(&pixelBufferData, &pixelBufferDataLength));
                pHL2ResearchMode->m_PVbufferSize = pixelBufferDataLength;

                // Create float contatiner for 3 x 4 transform and fx, fy
                std::vector<float> floatContainer(14);
                auto pFloatContainer = floatContainer.data();

                *pFloatContainer++ = fx;
                *pFloatContainer++ = fy;
                
                auto PVtoWorld = imgCoord.TryGetTransformTo(pHL2ResearchMode->m_refFrame);
                if (PVtoWorld)
                {
                    // column major -> row major
                    winrt::Windows::Foundation::Numerics::float4x4 PVtoWorldtransform = PVtoWorld.Value();
                    // Rotation
                    *pFloatContainer++ = PVtoWorldtransform.m11;
                    *pFloatContainer++ = PVtoWorldtransform.m12;
                    *pFloatContainer++ = PVtoWorldtransform.m13;

                    *pFloatContainer++ = PVtoWorldtransform.m21;
                    *pFloatContainer++ = PVtoWorldtransform.m22;
                    *pFloatContainer++ = PVtoWorldtransform.m23;

                    *pFloatContainer++ = PVtoWorldtransform.m31;
                    *pFloatContainer++ = PVtoWorldtransform.m32;
                    *pFloatContainer++ = PVtoWorldtransform.m33;

                    // Translation
                    *pFloatContainer++ = PVtoWorldtransform.m41;
                    *pFloatContainer++ = PVtoWorldtransform.m42;
                    *pFloatContainer++ = PVtoWorldtransform.m43;
                }

                {
                    std::unique_lock<std::shared_mutex> l(pHL2ResearchMode->mu);
                    if (!pHL2ResearchMode->m_PVImage)
                    {
                        OutputDebugString(L"Create Space for Photo Video Image...\n");
                        pHL2ResearchMode->m_PVImage = new UINT8[pixelBufferDataLength];
                    }
                    memcpy(pHL2ResearchMode->m_PVImage, pixelBufferData, pixelBufferDataLength * sizeof(UINT8));

                    if (!pHL2ResearchMode->m_floatContainer)
                    {
                        OutputDebugString(L"Create Space for float container...\n");
                        pHL2ResearchMode->m_floatContainer = new float[14];
                    }
                    memcpy(pHL2ResearchMode->m_floatContainer, floatContainer.data(), floatContainer.size() * sizeof(float));
                    pHL2ResearchMode->m_containerLength = floatContainer.size();
                }
                pHL2ResearchMode->m_PVImageUpdated = true;
            }
        }
    }

    void HL2ResearchMode::OnFrameArrived(const MediaFrameReader& sender, const MediaFrameArrivedEventArgs& args)
    {
        if (MediaFrameReference frame = sender.TryAcquireLatestFrame())
        {
            if (frame == nullptr) return;
            std::unique_lock<std::shared_mutex> lock(m_frameMutex);
            m_latestFrame = frame;
        }
    }

    void HL2ResearchMode::StartDepthSensorLoop(bool reconstructPointCloud)
    {
        //std::thread th1([this] {this->DepthSensorLoopTest(); });
        if (reconstructPointCloud && m_refFrame == nullptr)
        {
            std::unique_lock<std::shared_mutex> l(mu);
            m_refFrame = m_locator.GetDefault().CreateStationaryFrameOfReferenceAtCurrentLocation().CoordinateSystem();
            m_reconstructShortThrowPointCloud = true;
        }

        m_pDepthUpdateThread = new std::thread(HL2ResearchMode::DepthSensorLoop, this);
    }

    void HL2ResearchMode::DepthSensorLoop(HL2ResearchMode* pHL2ResearchMode)
    {
        // prevent starting loop for multiple times
        if (!pHL2ResearchMode->m_depthSensorLoopStarted)
            pHL2ResearchMode->m_depthSensorLoopStarted = true;
        else return;

        OutputDebugString(L"Opening Depth Stream...\n");
        winrt::check_hresult(pHL2ResearchMode->m_depthSensor->OpenStream());
        OutputDebugString(L"Opened Depth Stream\n");

        try 
        {
            UINT64 lastTs = 0;
            while (pHL2ResearchMode->m_depthSensorLoopStarted)
            {
                IResearchModeSensorFrame* pDepthSensorFrame = nullptr;
                ResearchModeSensorResolution resolution;
                pHL2ResearchMode->m_depthSensor->GetNextBuffer(&pDepthSensorFrame);
                
                // process sensor frame
                pDepthSensorFrame->GetResolution(&resolution);
                pHL2ResearchMode->m_depthResolution = resolution;
                
                IResearchModeSensorDepthFrame* pDepthFrame = nullptr;
                winrt::check_hresult(pDepthSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame)));

                size_t outBufferCount = 0;
                const UINT16* pDepth = nullptr;
                pDepthFrame->GetBuffer(&pDepth, &outBufferCount);

                size_t outAbBufferCount = 0;
                const UINT16* pAbImage = nullptr;
                pDepthFrame->GetAbDepthBuffer(&pAbImage, &outAbBufferCount);

                auto pDepthTexture = std::make_unique<uint8_t[]>(outBufferCount);
                std::vector<float> pointCloud;

                // get tracking transform
                ResearchModeSensorTimestamp timestamp;
                pDepthSensorFrame->GetTimeStamp(&timestamp);

                if (timestamp.HostTicks == lastTs) continue;
                lastTs = timestamp.HostTicks;

                auto ts = PerceptionTimestampHelper::FromSystemRelativeTargetTime(HundredsOfNanoseconds(checkAndConvertUnsigned(timestamp.HostTicks)));
                auto transToWorld = pHL2ResearchMode->m_locator.TryLocateAtTimestamp(ts, pHL2ResearchMode->m_refFrame);
                if (transToWorld == nullptr) continue;
                pHL2ResearchMode->m_latestShortDepthTimestamp = ts.TargetTime().time_since_epoch().count();

                XMMATRIX depthToWorld = pHL2ResearchMode->m_depthCameraPoseInvMatrix * SpatialLocationToDxMatrix(transToWorld);
                
                pHL2ResearchMode->mu.lock();
                auto roiCenterFloat = XMFLOAT3(pHL2ResearchMode->m_roiCenter[0], pHL2ResearchMode->m_roiCenter[1], pHL2ResearchMode->m_roiCenter[2]);
                auto roiBoundFloat = XMFLOAT3(pHL2ResearchMode->m_roiBound[0], pHL2ResearchMode->m_roiBound[1], pHL2ResearchMode->m_roiBound[2]);
                pHL2ResearchMode->mu.unlock();
                XMVECTOR roiCenter = XMLoadFloat3(&roiCenterFloat);
                XMVECTOR roiBound = XMLoadFloat3(&roiBoundFloat);
                
                for (UINT i = 0; i < resolution.Height; i++)
                {
                    for (UINT j = 0; j < resolution.Width; j++)
                    {
                        auto idx = resolution.Width * i + j;
                        UINT16 depth = pDepth[idx];
                        depth = (depth > 4090) ? 0 : depth - pHL2ResearchMode->m_depthOffset;

                        // back-project point cloud within Roi
                        if (pHL2ResearchMode->m_reconstructShortThrowPointCloud &&
                            i > pHL2ResearchMode->depthCamRoi.kRowLower*resolution.Height&& i < pHL2ResearchMode->depthCamRoi.kRowUpper * resolution.Height &&
                            j > pHL2ResearchMode->depthCamRoi.kColLower* resolution.Width&& j < pHL2ResearchMode->depthCamRoi.kColUpper * resolution.Width &&
                            depth > pHL2ResearchMode->depthCamRoi.depthNearClip && depth < pHL2ResearchMode->depthCamRoi.depthFarClip)
                        {
                            float xy[2] = { 0, 0 };
                            float uv[2] = { j, i };
                            pHL2ResearchMode->m_pDepthCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);
                            auto pointOnUnitPlane = XMFLOAT3(xy[0], xy[1], 1);
                            auto tempPoint = (float)depth / 1000 * XMVector3Normalize(XMLoadFloat3(&pointOnUnitPlane));
                            // apply transformation
                            auto pointInWorld = XMVector3Transform(tempPoint, depthToWorld);

                            // filter point cloud based on region of interest
                            if (!pHL2ResearchMode->m_useRoiFilter ||
                                (pHL2ResearchMode->m_useRoiFilter && XMVector3InBounds(pointInWorld - roiCenter, roiBound)))
                            {
                                pointCloud.push_back(XMVectorGetX(pointInWorld));
                                pointCloud.push_back(XMVectorGetY(pointInWorld));
                                pointCloud.push_back(-XMVectorGetZ(pointInWorld));
                            }
                        }

                        // save as grayscale texture pixel into temp buffer
                        if (depth == 0) { pDepthTexture.get()[idx] = 0; }
                        else { pDepthTexture.get()[idx] = (uint8_t)((float)depth / 1000 * 255); }

                        // save the depth of center pixel
                        if (pHL2ResearchMode->m_reconstructShortThrowPointCloud && 
                            i == (UINT)(0.35 * resolution.Height) && j == (UINT)(0.5 * resolution.Width)
                            && pointCloud.size()>=3)
                        {
                            pHL2ResearchMode->m_centerDepth = depth;
                            if (depth > pHL2ResearchMode->depthCamRoi.depthNearClip && depth < pHL2ResearchMode->depthCamRoi.depthFarClip)
                            {
                                std::unique_lock<std::shared_mutex> l(pHL2ResearchMode->mu);
                                pHL2ResearchMode->m_centerPoint[0] = *(pointCloud.end() - 3);
                                pHL2ResearchMode->m_centerPoint[1] = *(pointCloud.end() - 2);
                                pHL2ResearchMode->m_centerPoint[2] = *(pointCloud.end() - 1);
                            }
                        }
                    }
                }

                winrt::Windows::Foundation::Numerics::float4x4 depthToWorld_float4x4;
                XMStoreFloat4x4(&depthToWorld_float4x4, depthToWorld);

                std::vector<float> floatContainer(12);
                auto pFloatContainer = floatContainer.data();

                if (transToWorld)
                {
                    // Rotation
                    *pFloatContainer++ = depthToWorld_float4x4.m11;
                    *pFloatContainer++ = depthToWorld_float4x4.m12;
                    *pFloatContainer++ = depthToWorld_float4x4.m13;

                    *pFloatContainer++ = depthToWorld_float4x4.m21;
                    *pFloatContainer++ = depthToWorld_float4x4.m22;
                    *pFloatContainer++ = depthToWorld_float4x4.m23;

                    *pFloatContainer++ = depthToWorld_float4x4.m31;
                    *pFloatContainer++ = depthToWorld_float4x4.m32;
                    *pFloatContainer++ = depthToWorld_float4x4.m33;

                    // Translation
                    *pFloatContainer++ = depthToWorld_float4x4.m41;
                    *pFloatContainer++ = depthToWorld_float4x4.m42;
                    *pFloatContainer++ = depthToWorld_float4x4.m43;
                }

                if (!pHL2ResearchMode->m_LUTGenerated_short)
                {
                    float uv[2];
                    float xy[2];
                    std::vector<float> lutTable(size_t(resolution.Width * resolution.Height) * 3);
                    auto pLutTable = lutTable.data();

                    for (size_t y = 0; y < resolution.Height; y++)
                    {
                        uv[1] = (y + 0.5f);
                        for (size_t x = 0; x < resolution.Width; x++)
                        {
                            uv[0] = (x + 0.5f);
                            HRESULT hr = pHL2ResearchMode->m_pDepthCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);
                            if (FAILED(hr))
                            {
                                *pLutTable++ = xy[0];
                                *pLutTable++ = xy[1];
                                *pLutTable++ = 0.f;
                                continue;
                            }
                            float z = 1.0f;
                            const float norm = sqrtf(xy[0] * xy[0] + xy[1] * xy[1] + z * z);
                            const float invNorm = 1.0f / norm;
                            xy[0] *= invNorm;
                            xy[1] *= invNorm;
                            z *= invNorm;

                            // Dump LUT row
                            *pLutTable++ = xy[0];
                            *pLutTable++ = xy[1];
                            *pLutTable++ = z;
                        }
                    }
                    OutputDebugString(L"Create Space for lut...\n");
                    pHL2ResearchMode->m_lut_short = new float[outBufferCount * 3];
                    memcpy(pHL2ResearchMode->m_lut_short, lutTable.data(), lutTable.size() * sizeof(float));
                    pHL2ResearchMode->m_lutLength_short = lutTable.size();
                    pHL2ResearchMode->m_LUTGenerated_short = true;
                }

                // save data
                {
                    std::unique_lock<std::shared_mutex> l(pHL2ResearchMode->mu);

                    // save point cloud
                    if (pHL2ResearchMode->m_reconstructShortThrowPointCloud)
                    {
                        if (!pHL2ResearchMode->m_pointCloud)
                        {
                            OutputDebugString(L"Create Space for point cloud...\n");
                            pHL2ResearchMode->m_pointCloud = new float[outBufferCount * 3];
                        }
                        memcpy(pHL2ResearchMode->m_pointCloud, pointCloud.data(), pointCloud.size() * sizeof(float));
                        pHL2ResearchMode->m_pointcloudLength = pointCloud.size();
                    }

                    // save raw depth map
                    if (!pHL2ResearchMode->m_depthMap)
                    {
                        OutputDebugString(L"Create Space for depth map...\n");
                        pHL2ResearchMode->m_depthMap = new UINT8[outBufferCount * 2];
                    }
                    memcpy(pHL2ResearchMode->m_depthMap, pDepth, outBufferCount * 2);
                    pHL2ResearchMode->m_depthBufferSize = outBufferCount * 2;

                    // save short throw AbImage
                    if (!pHL2ResearchMode->m_shortAbImage)
                    {
                        OutputDebugString(L"Create Space for short-throw AbImage...\n");
                        pHL2ResearchMode->m_shortAbImage = new UINT8[outAbBufferCount * 2];
                    }
                    memcpy(pHL2ResearchMode->m_shortAbImage, pAbImage, outAbBufferCount * sizeof(UINT16));
                    pHL2ResearchMode->m_shortAbImageBufferSize = outAbBufferCount * 2;

                    // save pre-processed depth map texture (for visualization)
                    if (!pHL2ResearchMode->m_depthMapTexture)
                    {
                        OutputDebugString(L"Create Space for depth map texture...\n");
                        pHL2ResearchMode->m_depthMapTexture = new UINT8[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_depthMapTexture, pDepthTexture.get(), outBufferCount * sizeof(UINT8));
              
                    if (!pHL2ResearchMode->m_ahatTransform)
                    {
                        OutputDebugString(L"Create Space for Ahat to world transform...\n");
                        pHL2ResearchMode->m_ahatTransform = new float[12];
                    }
                    memcpy(pHL2ResearchMode->m_ahatTransform, floatContainer.data(), floatContainer.size() * sizeof(float));
                    pHL2ResearchMode->m_transformLength = 12;
                }

                pHL2ResearchMode->m_depthMapUpdated = true;
                pHL2ResearchMode->m_depthMapTextureUpdated = true;
                pHL2ResearchMode->m_pointCloudUpdated = true;

                pDepthTexture.reset();

                // release space
                if (pDepthFrame) {
                    pDepthFrame->Release();
                }
                if (pDepthSensorFrame)
                {
                    pDepthSensorFrame->Release();
                }
                
            }
        }
        catch (const std::exception& exc)  
        {
            std::stringstream ss;
            ss << "Depth Loop Exception: " << exc.what();
            std::string msg = ss.str();
            std::wstring widemsg = std::wstring(msg.begin(), msg.end());
            OutputDebugString(widemsg.c_str());
        }
        pHL2ResearchMode->m_depthSensor->CloseStream();
        pHL2ResearchMode->m_depthSensor->Release();
        pHL2ResearchMode->m_depthSensor = nullptr;
        
    }

    void HL2ResearchMode::StartLongDepthSensorLoop()
    {
        if (m_refFrame == nullptr)
        {
            std::unique_lock<std::shared_mutex> l(mu);
            m_refFrame = m_locator.GetDefault().CreateStationaryFrameOfReferenceAtCurrentLocation().CoordinateSystem();
        }

        m_pLongDepthUpdateThread = new std::thread(HL2ResearchMode::LongDepthSensorLoop, this);
    }

    void HL2ResearchMode::LongDepthSensorLoop(HL2ResearchMode* pHL2ResearchMode)
    {
        // prevent starting loop for multiple times
        if (!pHL2ResearchMode->m_longDepthSensorLoopStarted)
        {
            pHL2ResearchMode->m_longDepthSensorLoopStarted = true;
        }
        else {
            return;
        }

        pHL2ResearchMode->m_longDepthSensor->OpenStream();

        try
        {
            UINT64 lastTs = 0;
            while (pHL2ResearchMode->m_longDepthSensorLoopStarted)
            {
                IResearchModeSensorFrame* pDepthSensorFrame = nullptr;
                ResearchModeSensorResolution resolution;
                pHL2ResearchMode->m_longDepthSensor->GetNextBuffer(&pDepthSensorFrame);

                // process sensor frame
                pDepthSensorFrame->GetResolution(&resolution);
                pHL2ResearchMode->m_longDepthResolution = resolution;

                IResearchModeSensorDepthFrame* pDepthFrame = nullptr;
                winrt::check_hresult(pDepthSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame)));

                size_t outBufferCount = 0;
                const UINT16* pDepth = nullptr;

                const BYTE* pSigma = nullptr;
                pDepthFrame->GetSigmaBuffer(&pSigma, &outBufferCount);
                pDepthFrame->GetBuffer(&pDepth, &outBufferCount);
                pHL2ResearchMode->m_longDepthBufferSize = outBufferCount * sizeof(UINT16);

                // get tracking transform
                ResearchModeSensorTimestamp timestamp;
                pDepthSensorFrame->GetTimeStamp(&timestamp);

                if (timestamp.HostTicks == lastTs) continue;
                lastTs = timestamp.HostTicks;

                auto ts = PerceptionTimestampHelper::FromSystemRelativeTargetTime(HundredsOfNanoseconds(checkAndConvertUnsigned(timestamp.HostTicks)));
                auto transToWorld = pHL2ResearchMode->m_locator.TryLocateAtTimestamp(ts, pHL2ResearchMode->m_refFrame);
                if (transToWorld == nullptr) continue;
                pHL2ResearchMode->m_latestLongDepthTimestamp = ts.TargetTime().time_since_epoch().count();

                XMMATRIX depthToWorld = pHL2ResearchMode->m_longDepthCameraPoseInvMatrix * SpatialLocationToDxMatrix(transToWorld);
                winrt::Windows::Foundation::Numerics::float4x4 depthToWorld_float4x4;
                XMStoreFloat4x4(&depthToWorld_float4x4, depthToWorld);

                std::vector<float> longExtrinsicFloatContainer(12);
                auto pLongExtrinsicFloatContainer = longExtrinsicFloatContainer.data();

                {
                    // Rotation
                    *pLongExtrinsicFloatContainer++ = depthToWorld_float4x4.m11;
                    *pLongExtrinsicFloatContainer++ = depthToWorld_float4x4.m12;
                    *pLongExtrinsicFloatContainer++ = depthToWorld_float4x4.m13;

                    *pLongExtrinsicFloatContainer++ = depthToWorld_float4x4.m21;
                    *pLongExtrinsicFloatContainer++ = depthToWorld_float4x4.m22;
                    *pLongExtrinsicFloatContainer++ = depthToWorld_float4x4.m23;

                    *pLongExtrinsicFloatContainer++ = depthToWorld_float4x4.m31;
                    *pLongExtrinsicFloatContainer++ = depthToWorld_float4x4.m32;
                    *pLongExtrinsicFloatContainer++ = depthToWorld_float4x4.m33;

                    // Translation
                    *pLongExtrinsicFloatContainer++ = depthToWorld_float4x4.m41;
                    *pLongExtrinsicFloatContainer++ = depthToWorld_float4x4.m42;
                    *pLongExtrinsicFloatContainer++ = depthToWorld_float4x4.m43;
                }

                winrt::Windows::Foundation::Numerics::float4x4 rigToWorldtransform = make_float4x4_from_quaternion(transToWorld.Orientation()) * make_float4x4_translation(transToWorld.Position());
                std::vector<float> rigFloatContainer(12);
                auto pRigFloatContainer = rigFloatContainer.data();

                {
                    // Rotation
                    *pRigFloatContainer++ = rigToWorldtransform.m11;
                    *pRigFloatContainer++ = rigToWorldtransform.m12;
                    *pRigFloatContainer++ = rigToWorldtransform.m13;

                    *pRigFloatContainer++ = rigToWorldtransform.m21;
                    *pRigFloatContainer++ = rigToWorldtransform.m22;
                    *pRigFloatContainer++ = rigToWorldtransform.m23;

                    *pRigFloatContainer++ = rigToWorldtransform.m31;
                    *pRigFloatContainer++ = rigToWorldtransform.m32;
                    *pRigFloatContainer++ = rigToWorldtransform.m33;

                    // Translation
                    *pRigFloatContainer++ = rigToWorldtransform.m41;
                    *pRigFloatContainer++ = rigToWorldtransform.m42;
                    *pRigFloatContainer++ = rigToWorldtransform.m43;
                }

                std::vector<BYTE> depthData;
                depthData.reserve(outBufferCount * sizeof(UINT16));
                // Validate depth
                for (size_t i = 0; i < outBufferCount; ++i)
                {
                    UINT16 d;
                    const bool invalid =  ((pSigma[i] & 0x80) > 0);
                    if (invalid)
                    {
                        d = 0;
                    }
                    else
                    {
                        d = pDepth[i];
                    }

                    depthData.push_back((BYTE)d);
                    depthData.push_back((BYTE)(d >> 8));
                }

                if (!pHL2ResearchMode->m_LUTGenerated_long)
                {
                    float uv[2];
                    float xy[2];
                    std::vector<float> lutTable(size_t(resolution.Width * resolution.Height) * 3);
                    auto pLutTable = lutTable.data();

                    for (size_t y = 0; y < resolution.Height; y++)
                    {
                        uv[1] = (y + 0.5f);
                        for (size_t x = 0; x < resolution.Width; x++)
                        {
                            uv[0] = (x + 0.5f);
                            HRESULT hr = pHL2ResearchMode->m_pLongDepthCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);
                            if (FAILED(hr))
                            {
                                *pLutTable++ = xy[0];
                                *pLutTable++ = xy[1];
                                *pLutTable++ = 0.f;
                                continue;
                            }
                            float z = 1.0f;
                            const float norm = sqrtf(xy[0] * xy[0] + xy[1] * xy[1] + z * z);
                            const float invNorm = 1.0f / norm;
                            xy[0] *= invNorm;
                            xy[1] *= invNorm;
                            z *= invNorm;

                            // Dump LUT row
                            *pLutTable++ = xy[0];
                            *pLutTable++ = xy[1];
                            *pLutTable++ = z;
                        }
                    }
                    OutputDebugString(L"Create Space for lut...\n");
                    pHL2ResearchMode->m_lut_long = new float[outBufferCount * 3];
                    memcpy(pHL2ResearchMode->m_lut_long, lutTable.data(), lutTable.size() * sizeof(float));
                    pHL2ResearchMode->m_lutLength_long = lutTable.size();
                    pHL2ResearchMode->m_LUTGenerated_long = true;
                }

                // save data
                {
                    std::unique_lock<std::shared_mutex> l(pHL2ResearchMode->mu);

                    // Save metadata
                    if (!pHL2ResearchMode->m_rigtoWorldTransform)
                    {
                        OutputDebugString(L"Create Space for rig to world transform...\n");
                        pHL2ResearchMode->m_rigtoWorldTransform = new float[12];
                    }
                    memcpy(pHL2ResearchMode->m_rigtoWorldTransform, rigFloatContainer.data(), rigFloatContainer.size() * sizeof(float));
                    pHL2ResearchMode->m_transformLength = rigFloatContainer.size();

                    // save raw depth map
                    if (!pHL2ResearchMode->m_longDepthMap)
                    {
                        OutputDebugString(L"Create Space for depth map...\n");
                        pHL2ResearchMode->m_longDepthMap = new UINT8[depthData.size()];
                    }
                    memcpy(pHL2ResearchMode->m_longDepthMap, &depthData[0], depthData.size());
                }

                //pHL2ResearchMode->m_longDepthMapTextureUpdated = true;
                pHL2ResearchMode->m_longDepthMapUpdated = true;

                // release space
                if (pDepthFrame) {
                    pDepthFrame->Release();
                }
                if (pDepthSensorFrame)
                {
                    pDepthSensorFrame->Release();
                }

            }
        }
        catch (...) {}
        pHL2ResearchMode->m_longDepthSensor->CloseStream();
        pHL2ResearchMode->m_longDepthSensor->Release();
        pHL2ResearchMode->m_longDepthSensor = nullptr;

    }

    void HL2ResearchMode::StartIMUSensorLoop()
    {
        if (m_refFrame == nullptr)
        {
            std::unique_lock<std::shared_mutex> l(mu);
            m_refFrame = m_locator.GetDefault().CreateStationaryFrameOfReferenceAtCurrentLocation().CoordinateSystem();
        }

        m_pIMUUpdateThread = new std::thread(HL2ResearchMode::IMUSensorLoop, this);
    }

    void HL2ResearchMode::IMUSensorLoop(HL2ResearchMode* pHL2ResearchMode)
    {
        // prevent starting loop for multiple times
        if (!pHL2ResearchMode->m_imuSensorLoopStarted)
        {
            pHL2ResearchMode->m_imuSensorLoopStarted = true;
        }
        else {
            return;
        }

        try
        {
            winrt::check_hresult(pHL2ResearchMode->m_accelSensor->OpenStream());
            winrt::check_hresult(pHL2ResearchMode->m_gyroSensor->OpenStream());
            while (pHL2ResearchMode->m_imuSensorLoopStarted)
            {
                IResearchModeSensorFrame* pAccelSensorFrame = nullptr;
                IResearchModeSensorFrame* pGyroSensorFrame = nullptr;
                pHL2ResearchMode->m_accelSensor->GetNextBuffer(&pAccelSensorFrame);
                pHL2ResearchMode->m_gyroSensor->GetNextBuffer(&pGyroSensorFrame);

                IResearchModeAccelFrame* pAccelFrame = nullptr;
                winrt::check_hresult(pAccelSensorFrame->QueryInterface(IID_PPV_ARGS(&pAccelFrame)));
                IResearchModeGyroFrame* pGyroFrame = nullptr;
                winrt::check_hresult(pGyroSensorFrame->QueryInterface(IID_PPV_ARGS(&pGyroFrame)));
               
                DirectX::XMFLOAT3 accelSample;
                pAccelFrame->GetCalibratedAccelaration(&accelSample);

                DirectX::XMFLOAT3 gyroSample;
                pGyroFrame->GetCalibratedGyro(&gyroSample);

                // get tracking transform
                ResearchModeSensorTimestamp timestamp;
                pAccelSensorFrame->GetTimeStamp(&timestamp);

                if (pHL2ResearchMode->m_prevIMUTimestamp == timestamp.HostTicks)
                {
                    continue;
                }
                pHL2ResearchMode->m_prevIMUTimestamp = timestamp.HostTicks;

                auto ts = PerceptionTimestampHelper::FromSystemRelativeTargetTime(HundredsOfNanoseconds(checkAndConvertUnsigned(timestamp.HostTicks)));
                auto transToWorld = pHL2ResearchMode->m_locator.TryLocateAtTimestamp(ts, pHL2ResearchMode->m_refFrame);
                if (transToWorld == nullptr) continue;

                //auto absoluteTimestamp = pHL2ResearchMode->m_converter.RelativeTicksToAbsoluteTicks(HundredsOfNanoseconds((long long)pHL2ResearchMode->m_prevIMUTimestamp)).count();
                pHL2ResearchMode->m_latestIMUTimestamp = ts.TargetTime().time_since_epoch().count();

                //auto rot = transToWorld.Orientation();
                /*{
                    std::stringstream ss;
                    ss << rot.x << "," << rot.y << "," << rot.z << "," << rot.w << "\n";
                    std::string msg = ss.str();
                    std::wstring widemsg = std::wstring(msg.begin(), msg.end());
                    OutputDebugString(widemsg.c_str());
                }*/
    /*            auto quatInDx = XMFLOAT4(rot.x, rot.y, rot.z, rot.w);
                auto rotMat = XMMatrixRotationQuaternion(XMLoadFloat4(&quatInDx));
                auto pos = transToWorld.Position();
                auto posMat = XMMatrixTranslation(pos.x, pos.y, pos.z);
                auto gyroToWorld = pHL2ResearchMode->m_gyroInvMatrix * rotMat * posMat;
                auto accelToWorld = pHL2ResearchMode->m_accelInvMatrix * rotMat * posMat;*/

                // save data
                {
                    std::unique_lock<std::shared_mutex> l(pHL2ResearchMode->mu);

                    // save Accel and Gyro data
                    if (!pHL2ResearchMode->m_imuSample)
                    {
                        OutputDebugString(L"Create Space for imu sample...\n");
                        pHL2ResearchMode->m_imuSample = new float[6];
                    }
                    pHL2ResearchMode->m_imuSample[0] = accelSample.x;
                    pHL2ResearchMode->m_imuSample[1] = accelSample.y;
                    pHL2ResearchMode->m_imuSample[2] = accelSample.z;

                    pHL2ResearchMode->m_imuSample[3] = gyroSample.x;
                    pHL2ResearchMode->m_imuSample[4] = gyroSample.y;
                    pHL2ResearchMode->m_imuSample[5] = gyroSample.z;

                }
                pHL2ResearchMode->m_imuSampleUpdated = true;

                // release space
                if (pAccelFrame) pAccelFrame->Release();
                if (pGyroFrame) pGyroFrame->Release();

                if (pAccelSensorFrame) pAccelSensorFrame->Release();
                if (pGyroSensorFrame) pGyroSensorFrame->Release();
            }
        }
        catch (...) {}
        pHL2ResearchMode->m_accelSensor->CloseStream();
        pHL2ResearchMode->m_accelSensor->Release();
        pHL2ResearchMode->m_accelSensor = nullptr;

        pHL2ResearchMode->m_gyroSensor->CloseStream();
        pHL2ResearchMode->m_gyroSensor->Release();
        pHL2ResearchMode->m_gyroSensor = nullptr;
    }

    void HL2ResearchMode::StartSpatialCamerasAllLoop()
    {
        if (m_refFrame == nullptr)
        {
            std::unique_lock<std::shared_mutex> l(mu);
            m_refFrame = m_locator.GetDefault().CreateStationaryFrameOfReferenceAtCurrentLocation().CoordinateSystem();
        }

        m_pSpatialCamerasAllUpdateThread = new std::thread(HL2ResearchMode::SpatialCamerasAllLoop, this);
    }

    void HL2ResearchMode::SpatialCamerasAllLoop(HL2ResearchMode* pHL2ResearchMode)
    {
        // prevent starting loop for multiple times
        if (!pHL2ResearchMode->m_spatialCamerasAllLoopStarted)
        {
            pHL2ResearchMode->m_spatialCamerasAllLoopStarted = true;
        }
        else return;

        pHL2ResearchMode->m_LFSensor->OpenStream();
        pHL2ResearchMode->m_RFSensor->OpenStream();
        pHL2ResearchMode->m_LLSensor->OpenStream();
        pHL2ResearchMode->m_RRSensor->OpenStream();

        try
        {
            while (pHL2ResearchMode->m_spatialCamerasAllLoopStarted)
            {
                IResearchModeSensorFrame* pLFCameraFrame = nullptr;
                IResearchModeSensorFrame* pRFCameraFrame = nullptr;
                IResearchModeSensorFrame* pLLCameraFrame = nullptr;
                IResearchModeSensorFrame* pRRCameraFrame = nullptr;

                ResearchModeSensorResolution resolution;

                pHL2ResearchMode->m_LFSensor->GetNextBuffer(&pLFCameraFrame);
                pHL2ResearchMode->m_RFSensor->GetNextBuffer(&pRFCameraFrame);
                pHL2ResearchMode->m_LLSensor->GetNextBuffer(&pLLCameraFrame);
                pHL2ResearchMode->m_RRSensor->GetNextBuffer(&pRRCameraFrame);

                // process sensor frame
                pLFCameraFrame->GetResolution(&resolution);

                IResearchModeSensorVLCFrame* pLFFrame = nullptr;
                winrt::check_hresult(pLFCameraFrame->QueryInterface(IID_PPV_ARGS(&pLFFrame)));
                IResearchModeSensorVLCFrame* pRFFrame = nullptr;
                winrt::check_hresult(pRFCameraFrame->QueryInterface(IID_PPV_ARGS(&pRFFrame)));
                IResearchModeSensorVLCFrame* pLLFrame = nullptr;
                winrt::check_hresult(pLLCameraFrame->QueryInterface(IID_PPV_ARGS(&pLLFrame)));
                IResearchModeSensorVLCFrame* pRRFrame = nullptr;
                winrt::check_hresult(pRRCameraFrame->QueryInterface(IID_PPV_ARGS(&pRRFrame)));

                size_t outBufferCount = 0;
                const BYTE *pLFImage = nullptr;
                pLFFrame->GetBuffer(&pLFImage, &outBufferCount);

                const BYTE *pRFImage = nullptr;
                pRFFrame->GetBuffer(&pRFImage, &outBufferCount);

                const BYTE* pLLImage = nullptr;
                pLLFrame->GetBuffer(&pLLImage, &outBufferCount);

                const BYTE* pRRImage = nullptr;
                pRRFrame->GetBuffer(&pRRImage, &outBufferCount);

                // save data
                {
                    std::unique_lock<std::shared_mutex> l(pHL2ResearchMode->mu);

                    pHL2ResearchMode->m_SpatialCameraResolution = resolution;
                    pHL2ResearchMode->m_spatialBufferSize = outBufferCount;

                    // save images
                    if (!pHL2ResearchMode->m_LFImage)
                    {
                        OutputDebugString(L"Create Space for Left Front Image...\n");
                        pHL2ResearchMode->m_LFImage = new UINT8[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_LFImage, pLFImage, outBufferCount * sizeof(UINT8));

                    if (!pHL2ResearchMode->m_RFImage)
                    {
                        OutputDebugString(L"Create Space for Right Front Image...\n");
                        pHL2ResearchMode->m_RFImage = new UINT8[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_RFImage, pRFImage, outBufferCount * sizeof(UINT8));

                    if (!pHL2ResearchMode->m_LLImage)
                    {
                        OutputDebugString(L"Create Space for Left Left Image...\n");
                        pHL2ResearchMode->m_LLImage = new UINT8[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_LLImage, pLLImage, outBufferCount * sizeof(UINT8));

                    if (!pHL2ResearchMode->m_RRImage)
                    {
                        OutputDebugString(L"Create Space for Right Right Image...\n");
                        pHL2ResearchMode->m_RRImage = new UINT8[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_RRImage, pRRImage, outBufferCount * sizeof(UINT8));
                }
                pHL2ResearchMode->m_LFImageUpdated = true;
                pHL2ResearchMode->m_RFImageUpdated = true;
                pHL2ResearchMode->m_LLImageUpdated = true;
                pHL2ResearchMode->m_RRImageUpdated = true;

                // release space
                if (pLFFrame) pLFFrame->Release();
                if (pRFFrame) pRFFrame->Release();
                if (pLLFrame) pLLFrame->Release();
                if (pRRFrame) pRRFrame->Release();

                if (pLFCameraFrame) pLFCameraFrame->Release();
                if (pRFCameraFrame) pRFCameraFrame->Release();
                if (pLLCameraFrame) pLLCameraFrame->Release();
                if (pRRCameraFrame) pRRCameraFrame->Release();
            }
        }
        catch (...) {}
        pHL2ResearchMode->m_LFSensor->CloseStream();
        pHL2ResearchMode->m_LFSensor->Release();
        pHL2ResearchMode->m_LFSensor = nullptr;

        pHL2ResearchMode->m_RFSensor->CloseStream();
        pHL2ResearchMode->m_RFSensor->Release();
        pHL2ResearchMode->m_RFSensor = nullptr;

        pHL2ResearchMode->m_LLSensor->CloseStream();
        pHL2ResearchMode->m_LLSensor->Release();
        pHL2ResearchMode->m_LLSensor = nullptr;

        pHL2ResearchMode->m_RRSensor->CloseStream();
        pHL2ResearchMode->m_RRSensor->Release();
        pHL2ResearchMode->m_RRSensor = nullptr;
    }


    void HL2ResearchMode::StartSpatialCamerasFrontLoop()
    {
        if (m_refFrame == nullptr)
        {
            std::unique_lock<std::shared_mutex> l(mu);
            m_refFrame = m_locator.GetDefault().CreateStationaryFrameOfReferenceAtCurrentLocation().CoordinateSystem();
        }

        m_pSpatialCamerasFrontUpdateThread = new std::thread(HL2ResearchMode::SpatialCamerasFrontLoop, this);
    }

    void HL2ResearchMode::SpatialCamerasFrontLoop(HL2ResearchMode* pHL2ResearchMode)
    {
        // prevent starting loop for multiple times
        if (!pHL2ResearchMode->m_spatialCamerasFrontLoopStarted)
        {
            pHL2ResearchMode->m_spatialCamerasFrontLoopStarted = true;
        }
        else return;

        pHL2ResearchMode->m_LFSensor->OpenStream();
        pHL2ResearchMode->m_RFSensor->OpenStream();

        try
        {
            while (pHL2ResearchMode->m_spatialCamerasFrontLoopStarted)
            {
                IResearchModeSensorFrame* pLFCameraFrame = nullptr;
                IResearchModeSensorFrame* pRFCameraFrame = nullptr;
                ResearchModeSensorResolution LFResolution;
                ResearchModeSensorResolution RFResolution;
                pHL2ResearchMode->m_LFSensor->GetNextBuffer(&pLFCameraFrame);
				pHL2ResearchMode->m_RFSensor->GetNextBuffer(&pRFCameraFrame);
                if (pHL2ResearchMode->m_startArucoTrackingStereo)
                {
                    auto arucoResult = pHL2ResearchMode->TrackArUcoMarkersStereo(pLFCameraFrame, pRFCameraFrame);
                    std::unique_lock<std::shared_mutex> lock(pHL2ResearchMode->m_arucoMutex);
                    pHL2ResearchMode->m_arucoResult = arucoResult;
                }

    //            // process sensor frame
    //            pLFCameraFrame->GetResolution(&LFResolution);
    //            pHL2ResearchMode->m_LFResolution = LFResolution;
    //            pRFCameraFrame->GetResolution(&RFResolution);
    //            pHL2ResearchMode->m_RFResolution = RFResolution;

    //            IResearchModeSensorVLCFrame* pLFFrame = nullptr;
    //            winrt::check_hresult(pLFCameraFrame->QueryInterface(IID_PPV_ARGS(&pLFFrame)));
    //            IResearchModeSensorVLCFrame* pRFFrame = nullptr;
    //            winrt::check_hresult(pRFCameraFrame->QueryInterface(IID_PPV_ARGS(&pRFFrame)));

    //            size_t LFOutBufferCount = 0;
    //            const BYTE *pLFImage = nullptr;
    //            pLFFrame->GetBuffer(&pLFImage, &LFOutBufferCount);
    //            pHL2ResearchMode->m_spatialBufferSize = LFOutBufferCount;
				//size_t RFOutBufferCount = 0;
				//const BYTE *pRFImage = nullptr;
				//pRFFrame->GetBuffer(&pRFImage, &RFOutBufferCount);

    //            // save data
    //            {
    //                std::unique_lock<std::shared_mutex> l(pHL2ResearchMode->mu);

				//	// save LF and RF images
				//	if (!pHL2ResearchMode->m_LFImage)
				//	{
				//		OutputDebugString(L"Create Space for Left Front Image...\n");
				//		pHL2ResearchMode->m_LFImage = new UINT8[LFOutBufferCount];
				//	}
				//	memcpy(pHL2ResearchMode->m_LFImage, pLFImage, LFOutBufferCount * sizeof(UINT8));

				//	if (!pHL2ResearchMode->m_RFImage)
				//	{
				//		OutputDebugString(L"Create Space for Right Front Image...\n");
				//		pHL2ResearchMode->m_RFImage = new UINT8[RFOutBufferCount];
				//	}
				//	memcpy(pHL2ResearchMode->m_RFImage, pRFImage, RFOutBufferCount * sizeof(UINT8));
    //            }
				//pHL2ResearchMode->m_LFImageUpdated = true;
				//pHL2ResearchMode->m_RFImageUpdated = true;

    //            // release space
				//if (pLFFrame) pLFFrame->Release();
				//if (pRFFrame) pRFFrame->Release();

				if (pLFCameraFrame) pLFCameraFrame->Release();
				if (pRFCameraFrame) pRFCameraFrame->Release();
            }
        }
        catch (...) {}
        pHL2ResearchMode->m_LFSensor->CloseStream();
        pHL2ResearchMode->m_LFSensor->Release();
        pHL2ResearchMode->m_LFSensor = nullptr;

		pHL2ResearchMode->m_RFSensor->CloseStream();
		pHL2ResearchMode->m_RFSensor->Release();
		pHL2ResearchMode->m_RFSensor = nullptr;
    }

    std::map<int32_t, HL2ResearchMode::TrackedMarkerStereo> HL2ResearchMode::TrackArUcoMarkersStereo(
        IResearchModeSensorFrame* pLeftSensorFrame, IResearchModeSensorFrame* pRightSensorFrame)
    {
        std::map<int32_t, TrackedMarkerStereo> trackedMarkers;
        std::map<int32_t, DetectedMarker> leftDetectedMarkers, rightDetectedMarkers;

        std::vector<std::vector<cv::Point2f>> leftArucoMarkers, rightArucoMarkers, 
                                   leftArucoRejectedCandidates, rightArucoRejectedCandidates;
        std::vector<int32_t> leftArucoMarkerIds, rightArucoMarkerIds;

        // process sensor frame
        ResearchModeSensorResolution resolution;
        pLeftSensorFrame->GetResolution(&resolution);

        IResearchModeSensorVLCFrame* pLeftVLCFrame = nullptr;
        winrt::check_hresult(pLeftSensorFrame->QueryInterface(IID_PPV_ARGS(&pLeftVLCFrame)));
        IResearchModeSensorVLCFrame* pRightVLCFrame = nullptr;
        winrt::check_hresult(pRightSensorFrame->QueryInterface(IID_PPV_ARGS(&pRightVLCFrame)));

        size_t outBufferCount = 0;
        const BYTE *pLeftImage, *pRightImage = nullptr;
        pLeftVLCFrame->GetBuffer(&pLeftImage, &outBufferCount);
        pRightVLCFrame->GetBuffer(&pRightImage, &outBufferCount);

        cv::Mat wrappedLeftImage(resolution.Height, resolution.Width, CV_8U, (void*)pLeftImage);
        cv::Mat wrappedRightImage(resolution.Height, resolution.Width, CV_8U, (void*)pRightImage);

        cv::aruco::detectMarkers(
            wrappedLeftImage,
            m_arucoDictionary,
            leftArucoMarkers,
            leftArucoMarkerIds,
            m_arucoDetectorParameters,
            leftArucoRejectedCandidates);
        cv::aruco::detectMarkers(
            wrappedRightImage,
            m_arucoDictionary,
            rightArucoMarkers,
            rightArucoMarkerIds,
            m_arucoDetectorParameters,
            rightArucoRejectedCandidates);

        if (!leftArucoMarkerIds.empty() && !rightArucoMarkerIds.empty())
        {
            // get tracking transform
            ResearchModeSensorTimestamp timestamp_left, timestamp_right;
            pLeftSensorFrame->GetTimeStamp(&timestamp_left);
            pRightSensorFrame->GetTimeStamp(&timestamp_right);
            auto ts_l = PerceptionTimestampHelper::FromSystemRelativeTargetTime(HundredsOfNanoseconds(checkAndConvertUnsigned(timestamp_left.HostTicks)));
            auto rigToWorld_l = m_locator.TryLocateAtTimestamp(ts_l, m_refFrame);
            
            auto rigToWorld_r = rigToWorld_l;
            if (timestamp_left.HostTicks != timestamp_right.HostTicks)
            {
                auto ts_r = PerceptionTimestampHelper::FromSystemRelativeTargetTime(HundredsOfNanoseconds(checkAndConvertUnsigned(timestamp_right.HostTicks)));
                rigToWorld_r = m_locator.TryLocateAtTimestamp(ts_r, m_refFrame);
            }

            if (rigToWorld_l == nullptr || rigToWorld_r == nullptr) return trackedMarkers;
            
            //auto rigToWorldQuat = rigToWorld.Orientation();
            //Eigen::Quaterniond rigToWorldQ(rigToWorldQuat.w, rigToWorldQuat.x, rigToWorldQuat.y, rigToWorldQuat.z);
            //Eigen::Matrix3d rigToWorldR = rigToWorldQ.normalized().toRotationMatrix();
            //auto rigToWorldTrans = rigToWorld.Position();
            //Eigen::Vector3d rigToWorldT = Eigen::Vector3d(rigToWorldTrans.x, rigToWorldTrans.y, rigToWorldTrans.z);
            //
            //Eigen::Matrix4d rigToWorldTransform = Eigen::Matrix4d::Identity();
            //rigToWorldTransform.block(0, 0, 3, 3) = rigToWorldR;
            //rigToWorldTransform.block(0, 3, 3, 1) = rigToWorldT;
            auto rigToWorldDx_l = SpatialLocationToDxMatrix(rigToWorld_l);
            auto rigToWorldDx_r = SpatialLocationToDxMatrix(rigToWorld_r);
            auto LfToWorld = m_LFCameraPoseInvMatrix * rigToWorldDx_l;
            auto RfToWorld = m_RFCameraPoseInvMatrix * rigToWorldDx_r;

            XMFLOAT4X4 LfToWorldMat, RfToWorldMat;
            XMStoreFloat4x4(&LfToWorldMat, LfToWorld);
            XMStoreFloat4x4(&RfToWorldMat, RfToWorld);

            Eigen::Vector3f leftCamPinhole(LfToWorldMat._41, LfToWorldMat._42, LfToWorldMat._43);
            Eigen::Vector3f rightCamPinhole(RfToWorldMat._41, RfToWorldMat._42, RfToWorldMat._43);

            Eigen::Matrix3f leftCamToWorldR, rightCamToWorldR;
            leftCamToWorldR << LfToWorldMat._11, LfToWorldMat._21, LfToWorldMat._31,
                LfToWorldMat._12, LfToWorldMat._22, LfToWorldMat._32,
                LfToWorldMat._13, LfToWorldMat._23, LfToWorldMat._33;
            rightCamToWorldR << RfToWorldMat._11, RfToWorldMat._21, RfToWorldMat._31,
                RfToWorldMat._12, RfToWorldMat._22, RfToWorldMat._32,
                RfToWorldMat._13, RfToWorldMat._23, RfToWorldMat._33;

            for (size_t i = 0; i < leftArucoMarkerIds.size(); ++i)
            {
                const auto& markerCorners = leftArucoMarkers[i];
                if (markerCorners.size() != 4) continue;

                for (size_t j = 0; j < markerCorners.size(); ++j)
                {
                    DetectedMarker detectedMarker;
                    detectedMarker.markerId = leftArucoMarkerIds[i] * 4 + j;
                    detectedMarker.x = static_cast<int>(markerCorners[j].x);
                    detectedMarker.y = static_cast<int>(markerCorners[j].y);
                    detectedMarker.point = leftCamPinhole;

                    float xy[2] = { 0, 0 };
                    float uv[2] = { markerCorners[j].x, markerCorners[j].y };

                    winrt::check_hresult(m_LFCameraSensor->MapImagePointToCameraUnitPlane(uv, xy));
                    Eigen::Vector3f dirCam(xy[0], xy[1], 1);

                    detectedMarker.dir = leftCamToWorldR * dirCam;

                    leftDetectedMarkers[detectedMarker.markerId] = detectedMarker;
                }
            }
            for (size_t i = 0; i < rightArucoMarkerIds.size(); ++i)
            {
                const auto& markerCorners = rightArucoMarkers[i];
                if (markerCorners.size() != 4) continue;

                for (size_t j = 0; j < markerCorners.size(); ++j)
                {
                    DetectedMarker detectedMarker;
                    detectedMarker.markerId = rightArucoMarkerIds[i] * 4 + j;
                    detectedMarker.x = static_cast<int>(markerCorners[j].x);
                    detectedMarker.y = static_cast<int>(markerCorners[j].y);
                    detectedMarker.point = rightCamPinhole;

                    float xy[2] = { 0, 0 };
                    float uv[2] = { markerCorners[j].x, markerCorners[j].y };

                    winrt::check_hresult(m_RFCameraSensor->MapImagePointToCameraUnitPlane(uv, xy));
                    Eigen::Vector3f dirCam(xy[0], xy[1], 1);

                    detectedMarker.dir = rightCamToWorldR * dirCam;

                    rightDetectedMarkers[detectedMarker.markerId] = detectedMarker;
                }
            }

            for (const auto& leftDetectionIterator : leftDetectedMarkers)
            {
                const auto& leftDetection = leftDetectionIterator.second;

                if (rightDetectedMarkers.count(leftDetection.markerId))
                {
                    const auto& rightDetection = rightDetectedMarkers[leftDetection.markerId];

                    auto a = leftDetection.point;
                    auto b = leftDetection.dir;
                    auto c = rightDetection.point;
                    auto d = rightDetection.dir;

                    Eigen::Matrix<float, 3, 2> A;
                    A.col(0) = b;
                    A.col(1) = -d;
                    Eigen::Vector3f y = (c - a);
                    Eigen::Vector2f x = (A.transpose() * A).inverse() * (A.transpose() * y);
                    Eigen::Vector3f triangulatedPoint = (a + b * x[0] + c + d * x[1]) * 0.5f;

                    int markerId = leftDetection.markerId / 4;
                    int cornerPosition = leftDetection.markerId % 4;

                    auto trackedMarkerIter = trackedMarkers.find(markerId);
                    if (trackedMarkerIter != trackedMarkers.end())
                    {
                        trackedMarkerIter->second.corners[3 * cornerPosition] = triangulatedPoint[0];
                        trackedMarkerIter->second.corners[3 * cornerPosition + 1] = triangulatedPoint[1];
                        trackedMarkerIter->second.corners[3 * cornerPosition + 2] = -triangulatedPoint[2];
                    }
                    else
                    {
                        TrackedMarkerStereo triangulatedMarkerCorner;
                        triangulatedMarkerCorner.markerId = markerId;
                        triangulatedMarkerCorner.corners[3 * cornerPosition] = triangulatedPoint[0];
                        triangulatedMarkerCorner.corners[3 * cornerPosition + 1] = triangulatedPoint[1];
                        triangulatedMarkerCorner.corners[3 * cornerPosition + 2] = -triangulatedPoint[2];
                        trackedMarkers[markerId] = triangulatedMarkerCorner;
                    }
                }
            }
        }
        if (pLeftVLCFrame) pLeftVLCFrame->Release();
        if (pRightVLCFrame) pRightVLCFrame->Release();
        return trackedMarkers;
    }

    void HL2ResearchMode::CamAccessOnComplete(ResearchModeSensorConsent consent)
    {
        camAccessCheck = consent;
        SetEvent(camConsentGiven);
    }

    void HL2ResearchMode::ImuAccessOnComplete(ResearchModeSensorConsent consent)
    {
        imuAccessCheck = consent;
        SetEvent(imuConsentGiven);
    }

    inline INT64 HL2ResearchMode::GetPVTimestamp() { return m_latestPVTimestamp; }

    inline INT64 HL2ResearchMode::GetLongDepthTimestamp() { return m_latestLongDepthTimestamp; }

    inline INT64 HL2ResearchMode::GetShortDepthTimestamp() { return m_latestShortDepthTimestamp; }

    inline INT64 HL2ResearchMode::GetIMUTimestamp() { return m_latestIMUTimestamp; }

    inline UINT16 HL2ResearchMode::GetCenterDepth() {return m_centerDepth;}

    inline int HL2ResearchMode::GetDepthBufferSize() { return m_depthBufferSize; }

    inline bool HL2ResearchMode::DepthMapTextureUpdated() { return m_depthMapTextureUpdated; }

    inline bool HL2ResearchMode::DepthMapUpdated() { return m_depthMapUpdated; }

    inline bool HL2ResearchMode::PointCloudUpdated() { return m_pointCloudUpdated; }

    inline int HL2ResearchMode::GetLongDepthBufferSize() { return m_longDepthBufferSize; }

    inline bool HL2ResearchMode::LongDepthMapTextureUpdated() { return m_longDepthMapTextureUpdated; }

    inline bool HL2ResearchMode::LongDepthMapUpdated() { return m_longDepthMapUpdated; }

	inline bool HL2ResearchMode::LFImageUpdated() { return m_LFImageUpdated; }

	inline bool HL2ResearchMode::RFImageUpdated() { return m_RFImageUpdated; }

    inline bool HL2ResearchMode::LLImageUpdated() { return m_LLImageUpdated; }

    inline bool HL2ResearchMode::RRImageUpdated() { return m_RRImageUpdated; }

    inline bool HL2ResearchMode::PVImageUpdated() { return m_PVImageUpdated; }

    inline bool HL2ResearchMode::LongThrowLUTUpdated() { return m_LUTGenerated_long; }

    inline bool HL2ResearchMode::ShortThrowLUTUpdated() { return m_LUTGenerated_short; }

    inline bool HL2ResearchMode::IMUSampleUpdated() { return m_imuSampleUpdated; }

    hstring HL2ResearchMode::PrintDepthResolution()
    {
        std::string res_c_ctr = std::to_string(m_depthResolution.Height) + "x" + std::to_string(m_depthResolution.Width) + "x" + std::to_string(m_depthResolution.BytesPerPixel);
        return winrt::to_hstring(res_c_ctr);
    }

    hstring HL2ResearchMode::PrintDepthExtrinsics()
    {
        std::stringstream ss;
        ss << MatrixToString(m_depthCameraPose);
        std::string msg = ss.str();
        std::wstring widemsg = std::wstring(msg.begin(), msg.end());
        //OutputDebugString(widemsg.c_str());
        return winrt::to_hstring(msg);
    }

    hstring HL2ResearchMode::PrintLongDepthExtrinsics()
    {
        std::stringstream ss;
        ss << MatrixToString(m_longDepthCameraPose);
        std::string msg = ss.str();
        std::wstring widemsg = std::wstring(msg.begin(), msg.end());
        //OutputDebugString(widemsg.c_str());
        return winrt::to_hstring(msg);
    }

    hstring HL2ResearchMode::PrintPVIntrinsics()
    {
        std::string msg = m_PVCameraIntrinsics.str();
        std::wstring widemsg = std::wstring(msg.begin(), msg.end());
        //OutputDebugString(widemsg.c_str());
        return winrt::to_hstring(msg);
    }

	hstring HL2ResearchMode::PrintSpatialCameraResolution()
	{
		std::string res_c_ctr = std::to_string(m_SpatialCameraResolution.Height) + "x" + std::to_string(m_SpatialCameraResolution.Width) + "x" + std::to_string(m_SpatialCameraResolution.BytesPerPixel);
		return winrt::to_hstring(res_c_ctr);
	}

	hstring HL2ResearchMode::PrintLFExtrinsics()
	{
		std::stringstream ss;
		ss << MatrixToString(m_LFCameraPose);
		std::string msg = ss.str();
		std::wstring widemsg = std::wstring(msg.begin(), msg.end());
		//OutputDebugString(widemsg.c_str());
		return winrt::to_hstring(msg);
	}

	hstring HL2ResearchMode::PrintRFExtrinsics()
	{
		std::stringstream ss;
		ss << MatrixToString(m_RFCameraPose);
		std::string msg = ss.str();
		std::wstring widemsg = std::wstring(msg.begin(), msg.end());
		//OutputDebugString(widemsg.c_str());
		return winrt::to_hstring(msg);
	}

    hstring HL2ResearchMode::PrintLLExtrinsics()
    {
        std::stringstream ss;
        ss << MatrixToString(m_LLCameraPose);
        std::string msg = ss.str();
        std::wstring widemsg = std::wstring(msg.begin(), msg.end());
        //OutputDebugString(widemsg.c_str());
        return winrt::to_hstring(msg);
    }

    hstring HL2ResearchMode::PrintRRExtrinsics()
    {
        std::stringstream ss;
        ss << MatrixToString(m_RRCameraPose);
        std::string msg = ss.str();
        std::wstring widemsg = std::wstring(msg.begin(), msg.end());
        //OutputDebugString(widemsg.c_str());
        return winrt::to_hstring(msg);
    }

    std::string HL2ResearchMode::MatrixToString(DirectX::XMFLOAT4X4 mat)
    {
        std::stringstream ss;
        for (size_t i = 0; i < 4; i++)
        {
            for (size_t j = 0; j < 4; j++)
            {
                ss << mat(i, j) << ",";
            }
            ss << "\n";
        }
        return ss.str();
    }
    
    // Stop the sensor loop and release buffer space.
    // Sensor object should be released at the end of the loop function
    winrt::Windows::Foundation::IAsyncAction HL2ResearchMode::StopAllSensorDevice()
    {
        m_depthSensorLoopStarted = false;
        //m_pDepthUpdateThread->join();
        if (m_depthMap) 
        {
            delete[] m_depthMap;
            m_depthMap = nullptr;
        }
        if (m_depthMapTexture) 
        {
            delete[] m_depthMapTexture;
            m_depthMapTexture = nullptr;
        }
        if (m_pointCloud) 
        {
            m_pointcloudLength = 0;
            delete[] m_pointCloud;
            m_pointCloud = nullptr;
        }
        OutputDebugString(L"Stopped depth loop...\n");
        m_longDepthSensorLoopStarted = false;
        if (m_longDepthMap)
        {
            delete[] m_longDepthMap;
            m_longDepthMap = nullptr;
        }
        if (m_longDepthMapTexture)
        {
            delete[] m_longDepthMapTexture;
            m_longDepthMapTexture = nullptr;
        }
        OutputDebugString(L"Stopped long depth loop...\n");

        m_imuSensorLoopStarted = false;
        if (m_imuSample)
        {
            delete[] m_imuSample;
            m_imuSample = nullptr;
        }
        OutputDebugString(L"Stopped IMU loop...\n");
        
        m_spatialCamerasFrontLoopStarted = false;
        if (m_LFImage) 
        {
            delete[] m_LFImage;
            m_LFImage = nullptr;
        }
        if (m_RFImage)
        {
            delete[] m_RFImage;
            m_RFImage = nullptr;
        }
        if (m_LLImage)
        {
            delete[] m_LLImage;
            m_LLImage = nullptr;
        }
        if (m_RRImage)
        {
            delete[] m_RRImage;
            m_RRImage = nullptr;
        }
        OutputDebugString(L"Stopped Spatial camera loop...\n");

        if (mediaCapture != nullptr && mediaCapture.CameraStreamState() != CameraStreamState::Shutdown)
        {
            m_fExit = true;
            co_await m_mediaFrameReader.StopAsync();
            m_mediaFrameReader.Close();
            mediaCapture.Close();
            mediaCapture = nullptr;
        }
        OutputDebugString(L"Stopped PV loop...\n");

		m_pSensorDevice->Release();
        OutputDebugString(L"Release Sensor...\n");
		m_pSensorDevice = nullptr;
		m_pSensorDeviceConsent->Release();
        OutputDebugString(L"Release Consent...\n");
		m_pSensorDeviceConsent = nullptr;
    }

    com_array<uint8_t> HL2ResearchMode::GetDepthMapBuffer()
    {
        std::shared_lock<std::shared_mutex> l(mu);
        if (!m_depthMap)
        {
            return com_array<uint8_t>();
        }
        com_array<UINT8> tempBuffer = com_array<UINT8>(m_depthMap, m_depthMap + m_depthBufferSize);
        
        return tempBuffer;
    }

    com_array<uint8_t> HL2ResearchMode::GetShortAbImageBuffer()
    {
        std::shared_lock<std::shared_mutex> l(mu);
        if (!m_shortAbImage)
        {
            return com_array<uint8_t>();
        }
        com_array<UINT8> tempBuffer = com_array<UINT8>(m_shortAbImage, m_shortAbImage + m_shortAbImageBufferSize);

        return tempBuffer;
    }

    // Get depth map texture buffer. (For visualization purpose)
    com_array<uint8_t> HL2ResearchMode::GetDepthMapTextureBuffer()
    {
        std::shared_lock<std::shared_mutex> l(mu);
        if (!m_depthMapTexture) 
        {
            return com_array<UINT8>();
        }
        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_depthMapTexture), std::move_iterator(m_depthMapTexture + m_depthBufferSize));

        m_depthMapTextureUpdated = false;
        return tempBuffer;
    }

    com_array<uint8_t> HL2ResearchMode::GetLongDepthMapBuffer()
    {
        std::shared_lock<std::shared_mutex> l(mu);
        if (!m_longDepthMap)
        {
            return com_array<uint8_t>();
        }
        com_array<UINT8> tempBuffer = com_array<UINT8>(m_longDepthMap, m_longDepthMap + m_longDepthBufferSize);
        m_longDepthMapUpdated = false;
        return tempBuffer;
    }

    com_array<float> HL2ResearchMode::GetIMUSample()
    {
        std::shared_lock<std::shared_mutex> l(mu);
        if (!m_imuSample)
        {
            return com_array<float>(6);
        }
        com_array<float> tempBuffer = com_array<float>(std::move_iterator(m_imuSample), std::move_iterator(m_imuSample + 6));
        m_imuSampleUpdated = false;
        return tempBuffer;
    }

    com_array<uint8_t> HL2ResearchMode::GetLongDepthMapTextureBuffer()
    {
        std::shared_lock<std::shared_mutex> l(mu);
        if (!m_longDepthMapTexture)
        {
            return com_array<UINT8>();
        }
        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_longDepthMapTexture), std::move_iterator(m_longDepthMapTexture + m_longDepthBufferSize));

        m_longDepthMapTextureUpdated = false;
        return tempBuffer;
    }

	com_array<uint8_t> HL2ResearchMode::GetLFCameraBuffer()
	{
		std::shared_lock<std::shared_mutex> l(mu);
		if (!m_LFImage)
		{
			return com_array<UINT8>();
		}
		com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_LFImage), std::move_iterator(m_LFImage + m_spatialBufferSize));

		m_LFImageUpdated = false;
		return tempBuffer;
	}

	com_array<uint8_t> HL2ResearchMode::GetRFCameraBuffer()
	{
		std::shared_lock<std::shared_mutex> l(mu);
		if (!m_RFImage)
		{
			return com_array<UINT8>();
		}
		com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_RFImage), std::move_iterator(m_RFImage + m_spatialBufferSize));

		m_RFImageUpdated = false;
		return tempBuffer;
	}

    com_array<uint8_t> HL2ResearchMode::GetLLCameraBuffer()
    {
        std::shared_lock<std::shared_mutex> l(mu);
        if (!m_LLImage)
        {
            return com_array<UINT8>();
        }
        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_LLImage), std::move_iterator(m_LLImage + m_spatialBufferSize));

        m_LLImageUpdated = false;
        return tempBuffer;
    }

    com_array<uint8_t> HL2ResearchMode::GetRRCameraBuffer()
    {
        std::shared_lock<std::shared_mutex> l(mu);
        if (!m_RRImage)
        {
            return com_array<UINT8>();
        }
        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_RRImage), std::move_iterator(m_RRImage + m_spatialBufferSize));

        m_RRImageUpdated = false;
        return tempBuffer;
    }

    com_array<uint8_t> HL2ResearchMode::GetPVCameraBuffer()
    {
        std::shared_lock<std::shared_mutex> l(mu);
        if (!m_PVImage)
        {
            return com_array<UINT8>();
        }
        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_PVImage), std::move_iterator(m_PVImage + m_PVbufferSize));

        m_PVImageUpdated = false;
        return tempBuffer;
    }


    // Get the buffer for point cloud in the form of float array.
    // There will be 3n elements in the array where the 3i, 3i+1, 3i+2 element correspond to x, y, z component of the i'th point. (i->[0,n-1])
    com_array<float> HL2ResearchMode::GetPointCloudBuffer()
    {
        std::shared_lock<std::shared_mutex> l(mu);
        if (!m_reconstructShortThrowPointCloud || m_pointcloudLength == 0)
        {
            return com_array<float>();
        }
        com_array<float> tempBuffer = com_array<float>(std::move_iterator(m_pointCloud), std::move_iterator(m_pointCloud + m_pointcloudLength));
        m_pointCloudUpdated = false;
        return tempBuffer;
    }

    // Get the 3D point (float[3]) of center point in depth map. Can be used to render depth cursor.
    com_array<float> HL2ResearchMode::GetCenterPoint()
    {
        std::shared_lock<std::shared_mutex> l(mu);
        com_array<float> centerPoint = com_array<float>(std::move_iterator(m_centerPoint), std::move_iterator(m_centerPoint + 3));

        return centerPoint;
    }

    com_array<float> HL2ResearchMode::GetShortThrowLUT()
    {
        std::shared_lock<std::shared_mutex> l(mu);
        com_array<float> lut = com_array<float>(std::move_iterator(m_lut_short), std::move_iterator(m_lut_short + m_lutLength_short));

        return lut;
    }

    com_array<float> HL2ResearchMode::GetLongThrowLUT()
    {
        std::shared_lock<std::shared_mutex> l(mu);
        com_array<float> lut = com_array<float>(std::move_iterator(m_lut_long), std::move_iterator(m_lut_long + m_lutLength_long));

        return lut;
    }

    com_array<float> HL2ResearchMode::GetPVFloatBuffer()
    {
        std::shared_lock<std::shared_mutex> l(mu);
        if (m_containerLength == 0)
        {
            return com_array<float>();
        }
        com_array<float> tempBuffer = com_array<float>(std::move_iterator(m_floatContainer), std::move_iterator(m_floatContainer + m_containerLength));
        return tempBuffer;
    }

    com_array<float> HL2ResearchMode::GetRigTransformBuffer()
    {
        std::shared_lock<std::shared_mutex> l(mu);
        if (m_transformLength == 0)
        {
            return com_array<float>();
        }
        com_array<float> tempBuffer = com_array<float>(std::move_iterator(m_rigtoWorldTransform), std::move_iterator(m_rigtoWorldTransform + m_transformLength));
        return tempBuffer;
    }

    com_array<float> HL2ResearchMode::GetAhatTransformBuffer()
    {
        std::shared_lock<std::shared_mutex> l(mu);
        if (m_transformLength == 0)
        {
            return com_array<float>();
        }
        com_array<float> tempBuffer = com_array<float>(std::move_iterator(m_ahatTransform), std::move_iterator(m_ahatTransform + m_transformLength));
        return tempBuffer;
    }

    com_array<float> HL2ResearchMode::GetArucoCornersStereo(int id)
    {
        std::shared_lock<std::shared_mutex> l(m_arucoMutex);
        auto markerIter = m_arucoResult.find(id);
        if (markerIter != m_arucoResult.end())
            return com_array<float>(markerIter->second.corners);
        else
            return com_array<float>();
    }

    // Set the reference coordinate system. Need to be set before the sensor loop starts; otherwise, default coordinate will be used.
    void HL2ResearchMode::SetReferenceCoordinateSystem(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem refCoord)
    {
        std::unique_lock<std::shared_mutex> l(mu);
        m_refFrame = refCoord;
    }

    void HL2ResearchMode::SetPointCloudRoiInSpace(float centerX, float centerY, float centerZ, float boundX, float boundY, float boundZ)
    {
        std::unique_lock<std::shared_mutex> l(mu);

        m_useRoiFilter = true;
        m_roiCenter[0] = centerX;
        m_roiCenter[1] = centerY;
        m_roiCenter[2] = -centerZ;

        m_roiBound[0] = boundX;
        m_roiBound[1] = boundY;
        m_roiBound[2] = boundZ;
    }

    void HL2ResearchMode::SetPointCloudDepthOffset(uint16_t offset)
    {
        m_depthOffset = offset;
    }

    //long long HL2ResearchMode::checkAndConvertUnsigned(UINT64 val)
    //{
    //    assert(val <= kMaxLongLong);
    //    return static_cast<long long>(val);
    //}

    XMMATRIX HL2ResearchMode::SpatialLocationToDxMatrix(SpatialLocation location) {
        auto rot = location.Orientation();
        auto quatInDx = XMFLOAT4(rot.x, rot.y, rot.z, rot.w);
        auto rotMat = XMMatrixRotationQuaternion(XMLoadFloat4(&quatInDx));
        auto pos = location.Position();
        auto posMat = XMMatrixTranslation(pos.x, pos.y, pos.z);
        return rotMat * posMat;
    }

}
