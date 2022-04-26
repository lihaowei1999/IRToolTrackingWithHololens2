using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SingleTrackingObject : MonoBehaviour
{
	public string Name;
	public string TargetName;
	public bool Activity = false;
	public Vector3 Position = new Vector3(0, 0, 4);
	public Quaternion rotation = new Quaternion();
	public GameObject _GameObject;
	public void GenerateGameObject()
	{
		UnityEngine.Object cr = Resources.Load(TargetName, typeof(GameObject));
		_GameObject = Instantiate(cr) as GameObject;
		_GameObject.transform.position = Position;

	}

	public void SetTR(Vector3 shift, Quaternion Quat)
	{
		_GameObject.transform.position = shift;
		_GameObject.transform.rotation = Quat;
	}
}