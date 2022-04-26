using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System;
using System.IO;
using System.Xml;
using System.Threading;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using UnityEngine.UI;

using System.Linq;

public class TrackingObjectList : MonoBehaviour
{
	public List<SingleTrackingObject> TrackingTargets = new List<SingleTrackingObject>();

	public TrackingObjectList(String XMLFilePath)
	{

		//if (!File.Exists(XMLFilePath))
		//{
		//	Debug.LogError("XML File Path not exist");
		//	return;
		//}
		//// Read In XML File
		//XmlDocument xmldoc = new XmlDocument();
		//FileStream fileStream = new FileStream(XMLFilePath, FileMode.Open, FileAccess.Read, FileShare.Read);
		//// 读取文件的 byte[]
		//byte[] bytes = new byte[fileStream.Length];
		//fileStream.Read(bytes, 0, bytes.Length);
		////fileStream.Close()
		//// 把 byte[] 转换成 Stream
		//Stream stream = new MemoryStream(bytes);
		//xmldoc.Load((stream));
		////xmldoc.Load((XMLFilePath));
		//XmlNodeList node2 = xmldoc.GetElementsByTagName("item");
		//XmlNode x2 = node2[0];
		////print(node2.Count);
		//// XmlNode node2s = node2.Count();
		////XmlNodeList node = xmldoc.SelectSingleNode("item").ChildNodes;
		//XmlNodeList node = x2.ChildNodes;
		//foreach (XmlElement SingleObj in node)
		//{
		//	SingleTrackingObject tempObj = new SingleTrackingObject();
		//	foreach (XmlElement Property in SingleObj.ChildNodes)
		//	{
		//		print(Property.InnerText);
		//		switch (Property.Name)
		//		{

		//			case "Name":
		//				tempObj.Name = Property.InnerText;
		//				break;
		//			case "TargetName":
		//				tempObj.TargetName = Property.InnerText;
		//				break;
		//			case "InitiateStatus":
		//				if (!(Convert.ToInt16(Property.InnerText) == 0)) tempObj.Activity = true;
		//				break;
		//			case "InitiatePosition":
		//				foreach (XmlElement axis in Property.ChildNodes)
		//				{
		//					switch (axis.Name)
		//					{
		//						case "x":
		//							tempObj.Position.x = (float)Convert.ToDouble(axis.InnerText);
		//							break;
		//						case "y":
		//							tempObj.Position.y = (float)Convert.ToDouble(axis.InnerText);
		//							break;
		//						case "z":
		//							tempObj.Position.z = (float)Convert.ToDouble(axis.InnerText);
		//							break;
		//					}
		//				}
		//				break;
		//			case "RegistrationSetUp":
		//				// do nothing for registration setup in Unity  
		//				break;
		//		}
		//	}
		//	TrackingTargets.Add(tempObj);
		//}


		// Manual Loading
		// print("Initializing1");
		//SingleTrackingObject tempObj1 = new SingleTrackingObject();
		////tempObj1.Name = "Spine";
		////tempObj1.TargetName = "DisplaySpine";

		//tempObj1.Name = "NDIBall";
		//tempObj1.TargetName = "ToolExample";

		//tempObj1.Activity = true;
		//tempObj1.Position = new Vector3(0, 0, (float)0.6);
		//TrackingTargets.Add(tempObj1);
		//SingleTrackingObject tempObj2 = new SingleTrackingObject();
		//tempObj2.Name = "TrackingTarget";
		//tempObj2.TargetName = "Marker";
		//tempObj2.Activity = true;
		//tempObj2.Position = new Vector3(0, (float)0.5, 4);
		//TrackingTargets.Add(tempObj2);
		SingleTrackingObject tempObj2 = new SingleTrackingObject();

		tempObj2.Name = "ToolTip";
		tempObj2.TargetName = "DrillTip";

		tempObj2.Activity = true;
		tempObj2.Position = new Vector3((float)0.1, 0, (float)1.9);
		TrackingTargets.Add(tempObj2);

		SingleTrackingObject HandEyeTestModel_1 = new SingleTrackingObject();
		HandEyeTestModel_1.Name = "HandEyeTestModel_1";
		HandEyeTestModel_1.Activity = true;
		HandEyeTestModel_1.TargetName = "HandEyeTestModel_1";
		HandEyeTestModel_1.Position = new Vector3(0, 0, (float)3);
		TrackingTargets.Add(HandEyeTestModel_1);

		SingleTrackingObject HandEyeTestModel_2 = new SingleTrackingObject();
		HandEyeTestModel_2.Name = "HandEyeTestModel_2";
		HandEyeTestModel_2.Activity = true;
		HandEyeTestModel_2.TargetName = "HandEyeTestModel_2";
		HandEyeTestModel_2.Position = new Vector3(0, 0, (float)3);
		TrackingTargets.Add(HandEyeTestModel_2);

		SingleTrackingObject HandEyeTestModel_3 = new SingleTrackingObject();
		HandEyeTestModel_3.Name = "HandEyeTestModel_3";
		HandEyeTestModel_3.Activity = true;
		HandEyeTestModel_3.TargetName = "HandEyeTestModel_3";
		HandEyeTestModel_3.Position = new Vector3(0, 0, (float)3);
		TrackingTargets.Add(HandEyeTestModel_3);

		SingleTrackingObject HandEyeTestSpine = new SingleTrackingObject();
		HandEyeTestSpine.Name = "HandEyeTestSpine";
		HandEyeTestSpine.TargetName = "HandEyeTestSpine";
		HandEyeTestSpine.Activity = true;
		HandEyeTestSpine.Position = new Vector3(0, 0, (float)5);
		TrackingTargets.Add(HandEyeTestSpine);

		SingleTrackingObject DrillNeedle = new SingleTrackingObject();
		DrillNeedle.Name = "Needle";
		DrillNeedle.TargetName = "Needle";
		DrillNeedle.Activity = true;
		DrillNeedle.Position = new Vector3(0, 0, (float)1.5);
		TrackingTargets.Add(DrillNeedle);

		SingleTrackingObject RegistrationTarget = new SingleTrackingObject();
		RegistrationTarget.Name = "RegistrationHelper";
		RegistrationTarget.TargetName = "RegistrationHelper";
		RegistrationTarget.Activity = true;
		RegistrationTarget.Position = new Vector3(0, 0, (float)1.5);
		TrackingTargets.Add(RegistrationTarget);
	}



	public void GenerateGameObject()
	{
		foreach (SingleTrackingObject TrackingTarget in TrackingTargets)
		{

			TrackingTarget.GenerateGameObject();
		}
	}

	public void SetObjTR(int id, Vector3 shift, Quaternion Quat)
	{
		if (id < 0 || id > TrackingTargets.Count)
		{
			return;
		}
		TrackingTargets[id].SetTR(shift, Quat);
	}

	public void SetObjsTR(List<Vector3> shifts, List<Quaternion> quats, List<bool> status)
	{
		for (int i = 0; i < status.Count; i++)
		{
			if (status[i])
			{
				SetObjTR(i, shifts[i], quats[i]);
			}
		}
	}


}