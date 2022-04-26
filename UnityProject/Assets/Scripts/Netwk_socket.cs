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

public class Netwk_socket : MonoBehaviour
{
	public string IP;
	public int port;
	private Socket socket;
	private Socket connfd;
	IPAddress ip_address;
	IPEndPoint ip_endpoint;

	public Netwk_socket(int port)
	{
		this.port = port;
		this.socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

	}
	public void Connect(string ip)
	{
		this.IP = ip;
		this.ip_address = IPAddress.Parse(IP);
		this.ip_endpoint = new IPEndPoint(this.ip_address, this.port);
		print("Listening information");
		print("ip     :     " + this.IP);
		print("port   :     " + this.port);
		this.socket.Bind(this.ip_endpoint);
		print("Starting to listening");
		this.socket.Listen(1);
		this.connfd = socket.Accept();
		print("connection established");
		this.connfd.ReceiveTimeout = 5;
	}

	public void send(string msg)
	{
		byte[] byteArray = System.Text.Encoding.UTF8.GetBytes(msg);
		this.connfd.Send(byteArray);
	}
	//返回1024的值
	public string Recv()
	{
		try
		{
			byte[] readbuff = new byte[1024];
			int count = this.connfd.Receive(readbuff);
			string str = System.Text.Encoding.UTF8.GetString(readbuff, 0, count);
			return str;
		}
		catch (Exception ex)
		{

		}
		return "";
	}
}


public enum Control_enum
{
	create_axis = 5,
	create_axis_feedback = 6,
	return_pos = 15,
	get_hololens_pos = 20,
	return_hololens_pos = 25,
	set_position_6d = 30,
	set_single_object = 50,
	set_single_object_to_head=60,
	set_unity_frame_delay=70,
	set_slerp_percentage=71,
	set_interplote_percentage=72
};