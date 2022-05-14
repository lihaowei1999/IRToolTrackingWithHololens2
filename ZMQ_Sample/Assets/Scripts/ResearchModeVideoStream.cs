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
    [SerializeField] private ReqRep.Client reqClient;
    [SerializeField] private Publisher publisher;
    [SerializeField] private bool enableIMU = false;
    
    public GameObject shortDepthPreviewPlane = null;
    private Material shortDepthMediaMaterial = null;
    private Texture2D shortDepthMediaTexture = null;

    public GameObject shortAbPreviewPlane = null;
    private Material shortAbMediaMaterial = null;
    private Texture2D shortAbMediaTexture = null;

    private byte[] shortDepthTransformData = null;

    public GameObject PVPreviewPlane = null;
    private Material PVMediaMaterial = null;
    private Texture2D PVMediaTexture = null;

    private bool running = false;
    private bool lutSent = false;
    void Start()
    {
        shortDepthMediaMaterial = shortDepthPreviewPlane.GetComponent<MeshRenderer>().material;
        shortDepthMediaTexture = new Texture2D(512, 512, TextureFormat.R16, false);
        shortDepthMediaMaterial.mainTexture = shortDepthMediaTexture;

        shortAbMediaMaterial = shortAbPreviewPlane.GetComponent<MeshRenderer>().material;
        shortAbMediaTexture = new Texture2D(512, 512, TextureFormat.R16, false);
        shortAbMediaMaterial.mainTexture = shortAbMediaTexture;

        //PVMediaMaterial = PVPreviewPlane.GetComponent<MeshRenderer>().material;
        //PVMediaTexture = new Texture2D(960, 540, TextureFormat.Alpha8, false);
        //PVMediaMaterial.mainTexture = PVMediaTexture;
    }

    private void SetLutFlag(string msg)
    {
        DebugConsole.Log(msg);
        lutSent = true;
    }
    bool startRealtimePreview = false;
    public void TogglePreview()
    {
        startRealtimePreview = !startRealtimePreview;

        shortDepthPreviewPlane.SetActive(startRealtimePreview);
        shortAbPreviewPlane.SetActive(startRealtimePreview);
        //PVPreviewPlane.SetActive(startRealtimePreview);
    }
    void LateUpdate()
    {
        if (!running) return;

#if ENABLE_WINMD_SUPPORT
        // Update LUT
        if (researchMode.ShortThrowLUTUpdated() && !lutSent)
        {
            float[] lut = researchMode.GetShortThrowLUT();
            if (lut.Length > 0)
            {
                var lutData = new byte[lut.Length * 4];
                Buffer.BlockCopy(lut, 0, lutData, 0, lutData.Length);

                // TODO: make it async?
                reqClient.SendRequest("depth_lut", lutData, SetLutFlag); 
            }
        }

        // Update short depth map texture
        if (researchMode.DepthMapUpdated())
        {
            byte[] depthMap = researchMode.GetDepthMapBuffer();
            byte[] shortAbImage = researchMode.GetShortAbImageBuffer();

            if (depthMap.Length == 0 || shortAbImage.Length == 0) return;

            if (lutSent)
            {
                long tsData = researchMode.GetShortDepthTimestamp();
                //byte[] timestamp = BitConverter.GetBytes(tsData);
                float[] shortTransformFloat = researchMode.GetAhatTransformBuffer();

                if (shortDepthTransformData == null)
                {
                    shortDepthTransformData = new byte[shortTransformFloat.Length * 4];
                }
                Buffer.BlockCopy(shortTransformFloat, 0, shortDepthTransformData, 0, shortDepthTransformData.Length);

                NetMQ.NetMQMessage m = new NetMQ.NetMQMessage();
                m.Append("depth_frame");
                m.Append(tsData);
                m.Append(shortDepthTransformData);
                m.Append(depthMap);
                m.Append(shortAbImage);

                publisher.PublishMultipart(m);
            }

            // Show preview
            if (startRealtimePreview)
            {
                shortDepthMediaTexture.LoadRawTextureData(depthMap);
                shortDepthMediaTexture.Apply();

                shortAbMediaTexture.LoadRawTextureData(shortAbImage);
                shortAbMediaTexture.Apply();
            }
        }

        // update PV camera texture
        if (researchMode.PVImageUpdated())
        {
            byte[] frameTexture = researchMode.GetPVCameraBuffer();
            if (frameTexture.Length == 0) return;
            
            // apply pv data to preview plane
            if (startRealtimePreview)
            {
                PVMediaTexture.LoadRawTextureData(frameTexture);
                PVMediaTexture.Apply();
            }
        }
#endif

    }

    public async void StartSensorsEvent()
    {
#if ENABLE_WINMD_SUPPORT
        IntPtr WorldOriginPtr = UnityEngine.XR.WSA.WorldManager.GetNativeISpatialCoordinateSystemPtr();
        var unityWorldOrigin = Marshal.GetObjectForIUnknown(WorldOriginPtr) as Windows.Perception.Spatial.SpatialCoordinateSystem;

        if (researchMode == null)
        {
            researchMode = new HL2ResearchMode();
        }
        researchMode.SetReferenceCoordinateSystem(unityWorldOrigin);
        researchMode.InitializeDepthSensor();
        if (enableIMU)
        {
            researchMode.InitializeIMUSensor();
        }

        researchMode.StartDepthSensorLoop(false);
        //await researchMode.InitializePVCamera();

        if (enableIMU)
        {
            researchMode.StartIMUSensorLoop();
        }
#endif
        DebugConsole.Log("Sensors enabled");
        startRealtimePreview = true;
        shortDepthPreviewPlane.SetActive(true);
        shortAbPreviewPlane.SetActive(true);
        //PVPreviewPlane.SetActive(true);

        running = true;
    }
    public async void StopSensorsEvent()
    {
        startRealtimePreview = false;
        running = false;
        lutSent = false;

        shortDepthPreviewPlane.SetActive(false);
        shortAbPreviewPlane.SetActive(false);
        PVPreviewPlane.SetActive(false);

#if ENABLE_WINMD_SUPPORT
        await researchMode.StopAllSensorDevice();
        researchMode = null;
#endif
    }
    public void ExitApplication()
    {
        StopSensorsEvent();
        Application.Quit();
    }

    private void OnApplicationFocus(bool focus)
    {
        if (!focus) StopSensorsEvent();
    }
}