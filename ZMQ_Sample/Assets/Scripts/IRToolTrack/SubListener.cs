using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;

namespace IRToolTrack
{
    public class SubListener
    {
        private Thread _subThread;
        private readonly string _host;
        private readonly string _port;
        private bool _listenerOn;

        
        public float[] ToolTrans = new float[3];
        public float[] ToolQuat= new float[4];
        public bool ToolPoseUpdated = false;

        public string receivedTopic = "";
        public int msgSize = -1;

        private readonly ConcurrentQueue<Tuple<string, byte[]>> _messageQueue = new ConcurrentQueue<Tuple<string, byte[]>>();

        public SubListener(string host, string port)
        {
            _host = host;
            _port = port;
        }

        public void Start()
        {
            _listenerOn = true;
            _subThread = new Thread(ListenerWork);
            _subThread.Start();
        }

        public void Stop()
        {
            _listenerOn = false;
            _subThread?.Join();
            _subThread = null;
        }

        private void ListenerWork()
        {
            AsyncIO.ForceDotNet.Force();
            using (var subSocket = new SubscriberSocket())
            {
                subSocket.Options.ReceiveHighWatermark = 5;
                subSocket.Connect($"tcp://{_host}:{_port}");
                //subSocket.Subscribe("points");
                subSocket.SubscribeToAnyTopic();
                while (_listenerOn)
                {
                    List<byte[]> frames = new List<byte[]>();
                    if (!subSocket.TryReceiveMultipartBytes(ref frames)) continue;

                    string topic = System.Text.Encoding.UTF8.GetString(frames[0], 0, frames[0].Length);
                    receivedTopic = topic;
                    msgSize = frames.Count;
                    if (topic == "ir_tool")
                    {
                        Buffer.BlockCopy(frames[1], 0, ToolTrans, 0, 12);
                        Buffer.BlockCopy(frames[2], 0, ToolQuat, 0, 16);
                        ToolPoseUpdated = true;
                    }
                    
                }
                subSocket.Close();
            }
            NetMQConfig.Cleanup();
        }

        public bool TryGetLatestMessage(out byte[] msg)
        {
            if (!_messageQueue.IsEmpty)
            {
                Tuple<string, byte[]> temp;
                while (_messageQueue.TryDequeue(out temp))
                {
                    if (_messageQueue.IsEmpty) break;
                }
                msg = temp.Item2;
                return true;
            }
            msg = new byte[] { };
            return false;
        }

    }
}
