using UnityEngine;

namespace PubSub
{
    public class Subscriber : MonoBehaviour
    {
        [SerializeField] private string host;
        [SerializeField] private string port;
        private SubListener _listener;
        
        public enum Status
        {
            Inactive,
            Active
        }
        private Status _subStatus = Status.Inactive;

        private void Start()
        {
            _listener = new SubListener(host, port);
            DebugConsole.Log("Sub Listener initialized");
        }

        private void Update()
        {
            if (_subStatus == Status.Active)
            {
                if (_listener.TryGetLatestMessage(out var msg))
                {
                    DebugConsole.Log("Subsribed: " + msg);
                }
            }
        }

        public void StartSubscriber()
        {
            if (_subStatus == Status.Active)
            {
                DebugConsole.Log("Subscriber already started.");
                return;
            }
            DebugConsole.Log("Starting subscriber...");
            _listener.Start();
            DebugConsole.Log("Subscriber started");
            _subStatus = Status.Active;
        }

        public void StopSubscriber()
        {
            if (_subStatus == Status.Inactive)
            {
                DebugConsole.Log("Subscriber already stopped.");
                return;
            }
            DebugConsole.Log("Stopping subscriber...");
            _listener.Stop();
            DebugConsole.Log("Subscriber stopped");
            _subStatus = Status.Inactive;
        }

        private void HandleMessage(string message)
        {
            DebugConsole.Log("Received: " + message);
        }
    }
}

