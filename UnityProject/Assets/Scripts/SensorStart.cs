using System;
using System.Runtime.InteropServices;
using UnityEngine;

public class SensorStart : MonoBehaviour
{
#if ENABLE_WINMD_SUPPORT
    [DllImport("HL2RmStreamUnityPlugin", EntryPoint = "StartStreaming", CallingConvention = CallingConvention.Cdecl)]
    public static extern void StartStreaming();
#endif

	// Start is called before the first frame update
	void Start()
	{


	}

	public void StartSensorStreaming()
	{
		print("Start");
#if ENABLE_WINMD_SUPPORT
        StartStreaming();
#endif
		print("EndStarting");
	}
}
