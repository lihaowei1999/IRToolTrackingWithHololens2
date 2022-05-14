using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class DebugConsole : MonoBehaviour
{
    private static Queue<string> msgs = new Queue<string>();
    private static TextMeshPro _msgBox;
    private static int _maxMsgCount = 10;

    public static void InitMessageBox(TextMeshPro msgBox)
    {
        _msgBox = msgBox;
    }
    public static void Log(string msg)
    {
        if (_msgBox == null) return;
        while (msgs.Count > _maxMsgCount - 1)
        {
            msgs.Dequeue();
        }
        msgs.Enqueue(msg);

        string temp_str = "";
        foreach (string m in msgs)
        {
            temp_str = temp_str + "> " + m + "\n";
        }

        _msgBox.text = temp_str;
    }
}
