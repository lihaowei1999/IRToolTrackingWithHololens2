using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Text;
using System.Runtime.InteropServices;
using PubSub;

#if ENABLE_WINMD_SUPPORT
using System.Threading.Tasks;
using HL2UnityPlugin;
#endif

public class ResearchModeVideoStream : MonoBehaviour
{
#if ENABLE_WINMD_SUPPORT
    HL2ResearchMode researchMode;
#endif
    [SerializeField] private Publisher publisher;
    
    public GameObject shortDepthPreviewPlane = null;
    private Material shortDepthMediaMaterial = null;
    private Texture2D shortDepthMediaTexture = null;

    public GameObject shortAbPreviewPlane = null;
    private Material shortAbMediaMaterial = null;
    private Texture2D shortAbMediaTexture = null;

    private byte[] shortDepthTransformData = null;

    private bool sensorStarted = false;
    private bool startToolTracking = false;

    void Start()
    {
        shortDepthMediaMaterial = shortDepthPreviewPlane.GetComponent<MeshRenderer>().material;
        shortDepthMediaTexture = new Texture2D(512, 512, TextureFormat.R16, false);
        shortDepthMediaMaterial.mainTexture = shortDepthMediaTexture;

        shortAbMediaMaterial = shortAbPreviewPlane.GetComponent<MeshRenderer>().material;
        shortAbMediaTexture = new Texture2D(512, 512, TextureFormat.R16, false);
        shortAbMediaMaterial.mainTexture = shortAbMediaTexture;

        StartSensors();
    }

    bool startRealtimePreview = false;
    public void TogglePreview()
    {
        startRealtimePreview = !shortDepthPreviewPlane.activeInHierarchy;

        shortDepthPreviewPlane.SetActive(startRealtimePreview);
        shortAbPreviewPlane.SetActive(startRealtimePreview);
    }
    void LateUpdate()
    {
        if (!sensorStarted) return;

#if ENABLE_WINMD_SUPPORT
        // -------------------- Ahat camera --------------------
        // Show preview
        if (startRealtimePreview && researchMode.DepthMapUpdated())
        {
            shortDepthMediaTexture.LoadRawTextureData(researchMode.GetDepthMapBuffer());
            shortDepthMediaTexture.Apply();

            shortAbMediaTexture.LoadRawTextureData(researchMode.GetShortAbImageBuffer());
            shortAbMediaTexture.Apply();
        }

        // Tool track data
        if (researchMode.IRToolUpdated())
        {
            float[] shortTransformFloat = researchMode.GetAhatTransformBuffer();
            long depthTs = researchMode.GetShortDepthTimestamp();
            float[] irToolCenters = researchMode.GetIRToolCenters();

            if (irToolCenters.Length == 0) return;

            // Send Lookup table and depth images for tool tracking
            if (startToolTracking)
            {
                if (shortDepthTransformData == null)
                {
                    shortDepthTransformData = new byte[shortTransformFloat.Length * 4];
                }
                Buffer.BlockCopy(shortTransformFloat, 0, shortDepthTransformData, 0, shortDepthTransformData.Length);

                byte[] irCentersBytes = new byte[irToolCenters.Length * 4];
                Buffer.BlockCopy(irToolCenters, 0, irCentersBytes, 0, irCentersBytes.Length);

                NetMQ.NetMQMessage m = new NetMQ.NetMQMessage();
                m.Append("depth_frame");
                m.Append(depthTs);
                m.Append(shortDepthTransformData);
                m.Append(irCentersBytes);

                publisher.PublishMultipart(m);
            }
        }
#endif
    }

    public void StartToolTracking()
    {
        startToolTracking = true;

        if (!sensorStarted) StartSensors();
    }

    public void StopToolTracking()
    {
        startToolTracking = false;
    }

    private void StartSensors()
    {
#if ENABLE_WINMD_SUPPORT
        // Get Unity Origin Coordinate
#if UNITY_2020_1_OR_NEWER // note: Unity 2021.2 and later not supported
        IntPtr WorldOriginPtr = UnityEngine.XR.WindowsMR.WindowsMREnvironment.OriginSpatialCoordinateSystem;
        var unityWorldOrigin = Marshal.GetObjectForIUnknown(WorldOriginPtr) as Windows.Perception.Spatial.SpatialCoordinateSystem;
#else
        IntPtr WorldOriginPtr = UnityEngine.XR.WSA.WorldManager.GetNativeISpatialCoordinateSystemPtr();
        var unityWorldOrigin = Marshal.GetObjectForIUnknown(WorldOriginPtr) as Windows.Perception.Spatial.SpatialCoordinateSystem;
#endif
        if (researchMode == null)
        {
            researchMode = new HL2ResearchMode();
        }
        // Set Unity Origin Coordinate
        researchMode.SetReferenceCoordinateSystem(unityWorldOrigin);
        // Start Ahat camera
        researchMode.InitializeDepthSensor();

        // false: do not reconstruct point cloud; true: start IR sphere filtering
        researchMode.StartDepthSensorLoop(false, true); 
#endif
        // Set preview options
        startRealtimePreview = true;
        shortDepthPreviewPlane.SetActive(true);
        shortAbPreviewPlane.SetActive(true);

        sensorStarted = true;
    }

    public async void StopSensorsEvent()
    {
        startRealtimePreview = false;
        startToolTracking = false;

        shortDepthPreviewPlane.SetActive(false);
        shortAbPreviewPlane.SetActive(false);

#if ENABLE_WINMD_SUPPORT
        await researchMode.StopAllSensorDevice();
        researchMode = null;
        sensorStarted = false;
#endif
    }
    public void ExitApplication()
    {
        StopSensorsEvent();
        Application.Quit();
    }
}