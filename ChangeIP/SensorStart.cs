using System;
using System.Runtime.InteropServices;
using UnityEngine;

public class SensorStart : MonoBehaviour
{
#if ENABLE_WINMD_SUPPORT
    [DllImport("HL2RmStreamUnityPlugin", EntryPoint = "StartStreaming", CallingConvention = CallingConvention.Cdecl)]
    public static extern void StartStreaming(string IP, int Port_1, int Port_2, int Port_3, int Port_4, int Port_5);
#endif

	// Start is called before the first frame update
	void Start()
	{

		
	}
	// Update is called once per frame
	void Update()
	{
		
	}

	public void StartSensorStreaming()
	{
		print("Start");
#if ENABLE_WINMD_SUPPORT
        StartStreaming("192.168.68.105",3001,3002,3003,3004,3005);
#endif
		print("EndStarting");
	}
}
