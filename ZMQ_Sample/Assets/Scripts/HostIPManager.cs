using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;
using System.IO;
using System;

#if WINDOWS_UWP
using Windows.Storage;
#endif

public class HostIPManager : MonoBehaviour
{
    private static string _ip = "";
    private static bool _initialized = false;

    public static string GetHostIP()
    {
        return _ip;
    }

    public static void SetHostIP(string ip)
    {
        _ip = ip;
        _initialized = true;
    }
}
