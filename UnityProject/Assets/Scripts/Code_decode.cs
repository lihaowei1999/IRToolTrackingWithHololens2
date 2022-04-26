using System;
using System.IO;
using System.Xml;
using System.Threading;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

public class Code_decode : MonoBehaviour
{
	public string message;
	public int target_id;
	public double x, y, z;
	public double qw, qx, qy, qz;
	public int unity_delay_frames;
	public Control_enum control_command;
	public float slerp_percentage, interplot_percentage;


	public Code_decode(string mess)
	{
		this.message = mess;
	}

	public void Decode(bool display = false)
	{
		this.control_command = (Control_enum)string2int(this.message.Substring(0, 4), 4);
		if (this.control_command == Control_enum.create_axis || this.control_command == Control_enum.create_axis_feedback || this.control_command == Control_enum.return_pos)
		{
			this.x = string2double(this.message.Substring(4, 33), 16, 16);
			this.y = string2double(this.message.Substring(37, 33), 16, 16);
			this.z = string2double(this.message.Substring(70, 33), 16, 16);
			if (display)
			{
				print(this.control_command);
				print(this.x);
				print(this.y);
				print(this.z);
			}
		}
		else if (this.control_command == Control_enum.get_hololens_pos)
		{

		}
		else if (this.control_command == Control_enum.set_position_6d)
		{
			this.x = string2double(this.message.Substring(4, 33), 16, 16);
			this.y = string2double(this.message.Substring(37, 33), 16, 16);
			this.z = string2double(this.message.Substring(70, 33), 16, 16);
			this.qw = string2double(this.message.Substring(103, 33), 16, 16);
			this.qx = string2double(this.message.Substring(136, 33), 16, 16);
			this.qy = string2double(this.message.Substring(169, 33), 16, 16);
			this.qz = string2double(this.message.Substring(202, 33), 16, 16);
			if (display)
			{
				print(this.x);
				print(this.y);
				print(this.z);
				print(this.qw);
				print(this.qx);
				print(this.qy);
				print(this.qz);
			}
		}
		else if (this.control_command == Control_enum.set_single_object)
		{
			this.target_id = string2int(this.message.Substring(4, 4), 4);
			this.x = string2double(this.message.Substring(8, 33), 16, 16);
			this.y = string2double(this.message.Substring(41, 33), 16, 16);
			this.z = string2double(this.message.Substring(74, 33), 16, 16);
			this.qw = string2double(this.message.Substring(107, 33), 16, 16);
			this.qx = string2double(this.message.Substring(140, 33), 16, 16);
			this.qy = string2double(this.message.Substring(173, 33), 16, 16);
			this.qz = string2double(this.message.Substring(206, 33), 16, 16);
			if (display)
			{
				print(this.target_id);
				print(this.x);
				print(this.y);
				print(this.z);
				print(this.qw);
				print(this.qx);
				print(this.qy);
				print(this.qz);
			}
		}
		else if (this.control_command == Control_enum.set_single_object_to_head)
		{
			this.target_id = string2int(this.message.Substring(4, 4), 4);
			this.x = string2double(this.message.Substring(8, 33), 16, 16);
			this.y = string2double(this.message.Substring(41, 33), 16, 16);
			this.z = string2double(this.message.Substring(74, 33), 16, 16);
			this.qw = string2double(this.message.Substring(107, 33), 16, 16);
			this.qx = string2double(this.message.Substring(140, 33), 16, 16);
			this.qy = string2double(this.message.Substring(173, 33), 16, 16);
			this.qz = string2double(this.message.Substring(206, 33), 16, 16);
			if (display)
			{
				print(this.target_id);
				print(this.x);
				print(this.y);
				print(this.z);
				print(this.qw);
				print(this.qx);
				print(this.qy);
				print(this.qz);
			}
		}
		else if (this.control_command == Control_enum.set_unity_frame_delay)
		{
			this.unity_delay_frames = string2int(this.message.Substring(4, 4), 4);
		}
		else if (this.control_command == Control_enum.set_slerp_percentage)
		{
			this.slerp_percentage = (float)string2double(this.message.Substring(4, 33), 16, 16);
		}
		else if (this.control_command == Control_enum.set_interplote_percentage)
		{
			this.interplot_percentage = (float)string2double(this.message.Substring(4, 33), 16, 16);
		}
	}

	public int string2int(string substring, int int_)
	{
		return (int)Convert.ToInt64(substring.Substring(0, 4));
	}

	public double string2double(string substring, int int_, int deci_)
	{
		string sign_string = substring.Substring(0, 1);
		int sign_;
		if (sign_string == "0")
		{
			sign_ = -1;
		}
		else
		{
			sign_ = 1;
		}
		string int_string = substring.Substring(1, 16);
		string deci_string = substring.Substring(17, 16);
		double int_part = (double)Convert.ToInt64(int_string);
		double deci_part = (double)Convert.ToInt64(deci_string);
		double num = sign_ * (int_part + deci_part * Math.Pow(10, -deci_));
		return num;
	}

}