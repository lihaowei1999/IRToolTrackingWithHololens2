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

public class NetworkServer : MonoBehaviour
{
	// Start is called before the first frame update

	public string IPAddress;

	private Netwk_socket up_socket = new Netwk_socket(3000);
	private Netwk_socket down_socket = new Netwk_socket(4000);
	private bool Recv_mess = false;
	private Vector3 ad_po;
	private bool has_ad_po = true;
	private bool has_set_position_6d = true;
	private Vector3 position_model_shift;
	private Quaternion position_model_quaternion;

	Quaternion self_rot_r;
	Vector3 t_hololens_r;

	
	List<Vector3> ObjectPositions = new List<Vector3>();
	List<Quaternion> ObjectRotation = new List<Quaternion>();
	List<bool> ObjectHasMoved = new List<bool>();

	List<Vector3> ObjectTargetPosition = new List<Vector3>();
	List<Quaternion> ObjectTargetRotation = new List<Quaternion>();

	List<Quaternion> CameraQuaternionCache = new List<Quaternion>();
	List<Vector3> CameraShiftCache = new List<Vector3>();
	int CameraDelayMaxFrameNum;
	int CameraDelayFrameNum = 0;

	float quaternionslerppercentage = (float)0.8;
	float shift_inter_percent = (float)0.9;

	TrackingObjectList TrackingList;


	void Start()
	{
        //up_socket = new Netwk_socket(IPAddress, "3000");
        //down_socket = new Netwk_socket(IPAddress, "4000");

        //// cannnot find xml file here
        //string TestJsonPath = "/Resources/LoadObjectList.xml";
        //TrackingList = new TrackingObjectList(Application.dataPath + TestJsonPath);
        //TrackingList.GenerateGameObject();
        //for (int i = 0; i < TrackingList.TrackingTargets.Count; i++)
        //{
        //	ObjectPositions.Add(TrackingList.TrackingTargets[i].Position);
        //	// print(TrackingList.TrackingTargets[i].Position);
        //	//ObjectPositions.Add(new Vector3(0, 0, 0));
        //	ObjectRotation.Add(new Quaternion());
        //	ObjectTargetPosition.Add(TrackingList.TrackingTargets[i].Position);
        //	ObjectTargetRotation.Add(new Quaternion());
        //	ObjectHasMoved.Add(true);
        //}
        Vector3 tempcamerashift = Camera.main.transform.position;
        Quaternion tempcameraquat = Camera.main.transform.rotation;

        CameraDelayMaxFrameNum = 60;
        CameraDelayFrameNum = 5;

        for (int i = 0; i < CameraDelayMaxFrameNum; i++)
        {
            CameraQuaternionCache.Add(tempcameraquat);
            CameraShiftCache.Add(tempcamerashift);
        }


    }

	public void Connecting()
	{
		print("In connecting");
		up_socket.Connect(IPAddress);
		down_socket.Connect(IPAddress);
		Recv_mess = true;
		print("success");

	}

	// Update is called once per frame
	void Update()
	{

		//for (int i = 0; i < ObjectPositions.Count; i++)
		//{
		//	ObjectRotation[i] = Quaternion.Slerp(ObjectRotation[i], ObjectTargetRotation[i], quaternionslerppercentage);
		//	ObjectPositions[i] = shift_inter_percent * ObjectTargetPosition[i] + (1 - shift_inter_percent) * ObjectPositions[i];
		//}

		//TrackingList.SetObjsTR(ObjectPositions, ObjectRotation, ObjectHasMoved);

		this.self_rot_r = Camera.main.transform.rotation;
		this.t_hololens_r = Camera.main.transform.position;

		CameraQuaternionCache.Add(self_rot_r);
		CameraQuaternionCache.RemoveAt(0);
		CameraShiftCache.Add(t_hololens_r);
		CameraShiftCache.RemoveAt(0);




		if (Recv_mess == true)
		{
			Run_message();
			Recv_mess = false;
			print("I'm out, checkpoint_1");
		}

		if (!this.has_ad_po)
		{
			print("setting axis");
			print(this.ad_po);
			this.has_ad_po = !this.has_ad_po;
		}
		if (!this.has_set_position_6d)
		{
			this.has_set_position_6d = !this.has_set_position_6d;
		}

	}


	public async void Run_message()
	{
		print("in running mess");
		while (true)
		{
			string mess = this.up_socket.Recv();
			if (mess.Length == 1024)
			{
				var this_task = Dl_mess(mess);
				bool res = await this_task;
			}

			await Task.Delay(1);
		}
		print("jump out");

	}


	private Task<bool> Dl_mess(string mess)
	{
		var task = Task.Run(() =>
		{
			print("in await");

			if (mess.Length == 1024)
			{
				print("detect a signal");
				Code_decode my_decode = new Code_decode(mess);
				my_decode.Decode(true);
				if (my_decode.control_command == Control_enum.create_axis)
				{
					Vector3 add_pos = new Vector3((float)my_decode.x, (float)my_decode.y, (float)my_decode.z);
					print(add_pos);
					this.ad_po = add_pos;
					this.has_ad_po = false;

				}
				else if (my_decode.control_command == Control_enum.create_axis_feedback)
				{
					Vector3 ad_p = new Vector3((float)my_decode.x, (float)my_decode.y, (float)my_decode.z);
					Quaternion self_rot = this.self_rot_r;
					Vector3 t_hololens = this.t_hololens_r;
					Matrix4x4 transform_hololens_camera = new Matrix4x4();
					transform_hololens_camera.SetTRS(t_hololens, self_rot, new Vector3(1, 1, 1));
					double x = transform_hololens_camera[0, 0] * ad_p[0] + transform_hololens_camera[0, 1] * ad_p[1] + transform_hololens_camera[0, 2] * ad_p[2] + transform_hololens_camera[0, 3];
					double y = transform_hololens_camera[1, 0] * ad_p[0] + transform_hololens_camera[1, 1] * ad_p[1] + transform_hololens_camera[1, 2] * ad_p[2] + transform_hololens_camera[1, 3];
					double z = transform_hololens_camera[2, 0] * ad_p[0] + transform_hololens_camera[2, 1] * ad_p[1] + transform_hololens_camera[2, 2] * ad_p[2] + transform_hololens_camera[2, 3];
					double k = transform_hololens_camera[3, 0] * ad_p[0] + transform_hololens_camera[3, 1] * ad_p[1] + transform_hololens_camera[3, 2] * ad_p[2] + transform_hololens_camera[3, 3];
					x = x / k;
					y = y / k;
					z = z / k;
					Vector3 ppp = new Vector3((float)x, (float)y, (float)z);
					this.ad_po = ppp;
					this.has_ad_po = false;

					Code_code new_Code = new Code_code(Control_enum.return_pos);
					new_Code.Init_pos(ppp[0], ppp[1], ppp[2]);
					string mess_ret = new_Code.Code();
					this.down_socket.send(mess_ret);


				}
				else if (my_decode.control_command == Control_enum.get_hololens_pos)
				{
					Vector3 pos = this.t_hololens_r;
					Quaternion quat = this.self_rot_r;
					print("getting");
					print(pos);
					print(quat.w);
					print(quat.x);
					print(quat.y);
					print(quat.z);
					Code_code new_Code = new Code_code(Control_enum.return_hololens_pos);
					new_Code.Init_point6d((float)pos[0], (float)pos[1], (float)pos[2], (float)quat.w, (float)quat.x, (float)quat.y, (float)quat.z);
					string mess_ret = new_Code.Code();
					this.down_socket.send(mess_ret);
				}
				else if (my_decode.control_command == Control_enum.set_position_6d)
				{
					Vector3 shift_this_frame = new Vector3((float)my_decode.x, (float)my_decode.y, (float)my_decode.z);
					Quaternion quat_this_frame = new Quaternion();
					quat_this_frame.w = (float)my_decode.qw;
					quat_this_frame.x = (float)my_decode.qx;
					quat_this_frame.y = (float)my_decode.qy;
					quat_this_frame.z = (float)my_decode.qz;
					this.position_model_shift = shift_this_frame;
					this.position_model_quaternion = quat_this_frame;
					this.has_set_position_6d = false;
				}
				else if (my_decode.control_command == Control_enum.set_single_object)
				{
					int target_id = my_decode.target_id;
					Vector3 shift_this_frame = new Vector3((float)my_decode.x, (float)my_decode.y, (float)my_decode.z);
					Quaternion quat_this_frame = new Quaternion();
					quat_this_frame.w = (float)my_decode.qw;
					quat_this_frame.x = (float)my_decode.qx;
					quat_this_frame.y = (float)my_decode.qy;
					quat_this_frame.z = (float)my_decode.qz;

					//ObjectPositions[target_id] = shift_this_frame;
					//ObjectRotation[target_id] = quat_this_frame;
					ObjectTargetPosition[target_id] = shift_this_frame;
					ObjectTargetRotation[target_id] = quat_this_frame;

					ObjectHasMoved[target_id] = true;
					print(shift_this_frame);

				}
				else if (my_decode.control_command == Control_enum.set_single_object_to_head)
				{
					int target_id = my_decode.target_id;
					Vector3 shift_this_frame = new Vector3((float)my_decode.x, (float)my_decode.y, (float)my_decode.z);
					Quaternion quat_this_frame = new Quaternion();
					quat_this_frame.w = (float)my_decode.qw;
					quat_this_frame.x = (float)my_decode.qx;
					quat_this_frame.y = (float)my_decode.qy;
					quat_this_frame.z = (float)my_decode.qz;


					Matrix4x4 CameraMatrix = new Matrix4x4();
					CameraMatrix.SetTRS(CameraShiftCache[CameraDelayMaxFrameNum-1-CameraDelayFrameNum], CameraQuaternionCache[CameraDelayMaxFrameNum - 1 - CameraDelayFrameNum], new Vector3(1, 1, 1));
					//CameraMatrix.SetTRS(t_hololens_r, self_rot_r, new Vector3(1, 1, 1));
					Matrix4x4 RelativeMatrix = new Matrix4x4();
					RelativeMatrix.SetTRS(shift_this_frame, quat_this_frame, new Vector3(1, 1, 1));
					Matrix4x4 TargetMatrix = new Matrix4x4();
					TargetMatrix = CameraMatrix * RelativeMatrix;
					Vector4 vy = TargetMatrix.GetColumn(1);
					Vector4 vz = TargetMatrix.GetColumn(2);
					Quaternion newQ = Quaternion.LookRotation(new Vector3(vz.x, vz.y, vz.z), new Vector3(vy.x, vy.y, vy.z));

					Vector3 RealPostion = new Vector3(TargetMatrix[0, 3], TargetMatrix[1, 3], TargetMatrix[2, 3]);
					Quaternion RealRotation = newQ;

					ObjectTargetPosition[target_id] = RealPostion;
					ObjectTargetRotation[target_id] = RealRotation;

					//ObjectPositions[target_id] = RealPostion;
					//ObjectRotation[target_id] = RealRotation;
					//ObjectHasMoved[target_id] = true;
					print(shift_this_frame);

				}
				else if (my_decode.control_command == Control_enum.set_unity_frame_delay)
				{
					CameraDelayFrameNum = my_decode.unity_delay_frames;
				}
				else if (my_decode.control_command == Control_enum.set_slerp_percentage)
				{
					quaternionslerppercentage = my_decode.slerp_percentage;
				}
				else if (my_decode.control_command == Control_enum.set_interplote_percentage)
				{
					shift_inter_percent = my_decode.interplot_percentage;
				}
			}
			return true;
		});
		return task;
	}



}