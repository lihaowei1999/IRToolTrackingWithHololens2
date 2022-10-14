﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace IRToolTrack
{
    public class IRToolController : MonoBehaviour
    {
        private string host;
        [SerializeField] private string port;
        private SubListener _listener;
        private Vector3 targetPosition;
        private Quaternion targetRotation;

        public enum Status
        {
            Inactive,
            Active
        }
        private Status _subStatus = Status.Inactive;

        public void StartTracking()
        {
            if (_subStatus == Status.Active)
            {
                DebugConsole.Log("Tool tracking already started.");
                return;
            }
            _listener.Start();
            DebugConsole.Log("Tool track started");
            _subStatus = Status.Active;
        }

        public void StopTracking()
        {
            if (_subStatus == Status.Inactive)
            {
                DebugConsole.Log("Tool tracking already stopped.");
                return;
            }
            _listener.Stop();
            DebugConsole.Log("Tool tracking stopped.");
            _subStatus = Status.Inactive;
        }
        void Start()
        {
            host = HostIPManager.GetHostIP();
            _listener = new SubListener(host, port);
            DebugConsole.Log($"Listener listening to {host}:{port}");
        }
        void Update()
        {
            if (_subStatus == Status.Active && _listener.ToolPoseUpdated)
            {
                float[] transFromWorld = _listener.ToolTrans;
                float[] quatFromWorld = _listener.ToolQuat;

                targetPosition = new Vector3(transFromWorld[0], transFromWorld[1], transFromWorld[2]);
                targetRotation = new Quaternion(quatFromWorld[0], quatFromWorld[1], quatFromWorld[2], quatFromWorld[3]);

                _listener.ToolPoseUpdated = false;
            }

            if (_subStatus == Status.Active)
            {
                transform.position = Vector3.Lerp(transform.position, targetPosition, 0.97f);
                transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, 0.97f);
            }
        }
    }
}