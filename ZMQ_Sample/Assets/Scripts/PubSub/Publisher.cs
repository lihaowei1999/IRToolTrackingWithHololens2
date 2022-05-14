using UnityEngine;
using System;
using System.Collections.Concurrent;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;

namespace PubSub
{
    public class Publisher : MonoBehaviour
    {
        [SerializeField] private string port;
        private PublisherSocket dataPubSocket;

        public enum Status
        {
            Inactive,
            Active
        }
        private Status _pubStatus = Status.Inactive;

        public void StartDataPublisher()
        {
            AsyncIO.ForceDotNet.Force();
            dataPubSocket = new PublisherSocket();
            dataPubSocket.Options.SendHighWatermark = 3;
            dataPubSocket.Bind($"tcp://*:{port}");
            _pubStatus = Status.Active;
        }

        public void StopDataPublisher()
        {
            dataPubSocket.Close();
            //NetMQConfig.Cleanup();
            dataPubSocket = null;
            _pubStatus = Status.Inactive;
        }
        public bool DataPublisherWork(string topic, byte[] data)
        {
            if (dataPubSocket == null || _pubStatus == Status.Inactive) return false;
            
            try
            {
#if WINDOWS_UWP
                dataPubSocket?.SendMoreFrame(topic).SendFrame(data);
#endif
                return true;
            }
            catch (Exception ex)
            {
                DebugConsole.Log("[Pub] Exception: " + ex.Message);
            }
            return false;
        }

        public bool PublishMultipart(NetMQMessage m)
        {
            if (dataPubSocket == null || _pubStatus == Status.Inactive) return false;

            try
            {
#if WINDOWS_UWP
                dataPubSocket?.SendMultipartMessage(m);
#endif
                return true;
            }
            catch (Exception ex)
            {
                DebugConsole.Log("[Pub] Exception: " + ex.Message);
            }
            return false;
        }


#if WINDOWS_UWP
        private Windows.Perception.PerceptionTimestamp GetCurrentTimestamp()
        {
            // Get the current time, in order to create a PerceptionTimestamp. 
            Windows.Globalization.Calendar c = new Windows.Globalization.Calendar();
            c.SetToNow();
            Windows.Perception.PerceptionTimestamp ts = Windows.Perception.PerceptionTimestampHelper.FromHistoricalTargetTime(c.GetDateTime());
            return ts;
        }
#endif
    }
}