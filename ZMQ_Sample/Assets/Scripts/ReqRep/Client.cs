using UnityEngine;
using NetMQ;
using NetMQ.Sockets;
using System;
using System.Threading.Tasks;

namespace ReqRep
{
    public class Client : MonoBehaviour
    {
        private string host;
        [SerializeField] private string port;
        private RequestSocket _requestSocket;
        private TimeSpan timeout = new TimeSpan(0, 0, 0, 0, 800);
        public enum Status
        {
            Inactive,
            Active
        }
        private Status _status = Status.Inactive;

        private void Start()
        {
            host = HostIPManager.GetHostIP();
        }

        public void StartClient()
        {
            AsyncIO.ForceDotNet.Force();
            _requestSocket = new RequestSocket();
            _requestSocket.Connect($"tcp://{host}:{port}");
            DebugConsole.Log($"Request Connected with {host}:{port}");

            _requestSocket.Options.Correlate = true;
            _requestSocket.Options.Relaxed = true;

            _status = Status.Active;
        }

        public void StopClient()
        {
            _requestSocket.Close();
            _requestSocket = null;

            _status = Status.Inactive;
        }

        public void SendRequest(string message, Action<string> callback)
        {
            if (_status == Status.Inactive) return;

            NetMQMessage m = new NetMQMessage();
            m.Append(message);
            _requestSocket.SendMultipartMessage(m);
            while (!ReceiveMessage(callback))
            {
                _requestSocket.SendMultipartMessage(m);
            }
        }
        public void SendRequest(string topic, byte[] data, Action<string> callback)
        {
            if (_status == Status.Inactive) return;

            NetMQMessage m = new NetMQMessage();
            m.Append(topic);
            m.Append(data);
            _requestSocket.SendMultipartMessage(m);
            ReceiveMessage(callback);
            //while (!ReceiveMessage(callback))
            //{
            //    _requestSocket.SendMultipartMessage(m);
            //}
        }

        private bool ReceiveMessage(Action<string> callback)
        {
            var message = "";
            bool messageReceived = _requestSocket.TryReceiveFrameString(timeout, out message);
            if (messageReceived) callback?.Invoke(message);
            return messageReceived;
        }

    }
}

