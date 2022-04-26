using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Code_code : MonoBehaviour
{
	public Control_enum control_seq;
	public double x, y, z;
	public double qw, qx, qy, qz;
	public string mess;

	public Code_code(Control_enum ct)
	{
		this.control_seq = ct;
	}

	public void Init_pos(double x_, double y_, double z_)
	{
		this.x = x_;
		this.y = y_;
		this.z = z_;
	}

	public void Init_point6d(double x_, double y_, double z_, double qw_, double qx_, double qy_, double qz_)
	{
		this.x = x_;
		this.y = y_;
		this.z = z_;

		this.qw = qw_;
		this.qx = qx_;
		this.qy = qy_;
		this.qz = qz_;

	}

	public string Code()
	{
		print("coding");
		if (this.control_seq == Control_enum.return_pos)
		{
			string append = new string('0', 1024);
			string mess_ = string.Format("{0:D4}", (int)this.control_seq) + Double2string(this.x) + Double2string(this.y) + Double2string(this.z) + append;
			this.mess = mess_.Substring(0, 1024);
		}
		else if (this.control_seq == Control_enum.return_hololens_pos)
		{
			string append = new string('0', 1024);
			print(Double2string(this.x));
			print(Double2string(this.y));
			print(Double2string(this.z));
			print(Double2string(this.qw));
			print(Double2string(this.qx));
			print(Double2string(this.qy));
			print(Double2string(this.qz));


			string mess_ = string.Format("{0:D4}", (int)this.control_seq) + Double2string(this.x) + Double2string(this.y) + Double2string(this.z) + Double2string(this.qw) + Double2string(this.qx) + Double2string(this.qy) + Double2string(this.qz) + append;
			this.mess = mess_.Substring(0, 1024);
		}

		return this.mess;
	}

	public string Double2string(double va)
	{
		string tp;
		double dt;
		if (va >= 0)
		{
			tp = "1";
			dt = va;
		}
		else
		{
			tp = "0";
			dt = -va;
		}
		return tp + string.Format("{0:D16}{1:D16}", (long)System.Math.Floor(dt), (long)System.Math.Floor((dt - System.Math.Floor(dt)) * 1e16));

	}

}