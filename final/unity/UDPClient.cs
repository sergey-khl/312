/*
    Author(s):  Long Qian
    Created on: 2019-03-29
    (C) Copyright 2015-2018 Johns Hopkins University (JHU), All Rights Reserved.

    --- begin cisst license - do not edit ---
    This software is provided "as is" under an open source license, with
    no warranty.  The complete license can be found in license.txt and
    http://www.cisst.org/cisst/license.txt.
    --- end cisst license ---
*/

// updated by sergey and jerrica to allow 2 sockets at the same time and retrieval of multiple messages at once.
// this file also stores the controlled robot and controllable robot as we only have 1 copy of this file in our scene.
using UnityEngine;
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Collections.Generic;
using UnityEngine.tvOS;
using UnityEditor.VersionControl;

using System.Net;
using System.Net.Sockets;
using System.Threading;


namespace DVRK {

    public class UDPClient : MonoBehaviour {

        public int port_cp = 48054; // for position and orientation data
        public int port_extra = 48055; // for sending wrench and recieving button data
        public string remote_ip = "127.0.0.1"; // use phantom ip here
        private Queue<string> receivedCPData = new Queue<string>();
        private Queue<string> receivedButtonData = new Queue<string>();
        private readonly object queueLock = new object();
        private static Socket socket_cp;
         private static Socket socket_extra;
        private static EndPoint remote_cp;
        private static EndPoint remote_extra;
        private static EndPoint remote_extra_write;
         private static byte[] data;

      public int currentRobot;
      public List<URDFRobot> controllableRobots;




      public List<string> GetLatestCPPacket(int message_count) {
         lock (queueLock)
         {
            List<string> messages = new List<string>();

            while (receivedCPData.Count > message_count)
            {
               receivedCPData.Dequeue();
            }
            while (receivedCPData.Count > 0)
            {
               messages.Add(receivedCPData.Dequeue());
            }

            return messages;
         }
        }

      public List<string> GetLatestButtonPacket(int message_count)
      {
         List<string> messages = new List<string>();
         while (receivedButtonData.Count > message_count)
         {
            receivedButtonData.Dequeue();
         }
         while (receivedButtonData.Count > 0)
         {
            messages.Add(receivedButtonData.Dequeue());
         }
         return messages;
      }

      private string objectName;

        private void Awake() {
            objectName = name;
        }



        Thread receiveThreadCp;
        Thread receiveThreadExtra;
        UdpClient udpClient;

        private bool shouldTerminate = true;
        
        public void Start() {
            Debug.Log(objectName + ": Starting UDP");
            shouldTerminate = false;

            // setup socket
            remote_cp = new IPEndPoint(IPAddress.Any, port_cp);
            socket_cp = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);

            socket_cp.Bind(remote_cp);

            remote_extra = new IPEndPoint(IPAddress.Any, port_extra);
            socket_extra = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            socket_extra.Bind(remote_extra);
            remote_extra_write = new IPEndPoint(IPAddress.Parse(remote_ip), port_extra);


            receiveThreadCp = new Thread(new ThreadStart(() => ReceiveData(false)));
            receiveThreadCp.IsBackground = true;
            receiveThreadCp.Start();
            receiveThreadExtra = new Thread(new ThreadStart(() => ReceiveData(true)));
            receiveThreadExtra.IsBackground = true;
            receiveThreadExtra.Start();
      }

        
        void OnDestroy() {
            shouldTerminate = true;
            socket_cp.Close(); // trigger the exception in thread
            socket_extra.Close(); // trigger the exception in thread
            receiveThreadCp.Abort();
            while (receiveThreadCp.IsAlive) {
                // Debug.Log(objectName + ": Still alive");
            }
            receiveThreadExtra.Abort();
            while (receiveThreadExtra.IsAlive)
            {
               // Debug.Log(objectName + ": Still alive");
            }
            Debug.Log(objectName + ": Receive thread cp stopped");
            Debug.Log(objectName + ": Receive thread extra stopped");
        }
        


        // receive thread
        private void ReceiveData(bool is_extra) {
            //client = new UdpClient(port);
            Debug.Log(objectName + ": Receive thread starts");
            while (!shouldTerminate) {
                try {
                  byte[] data = new byte[1024];

                  if (is_extra)
                  {
                     int receivedBytes = socket_cp.ReceiveFrom(data, ref remote_cp);
                  } else
                  {
                     int receivedBytes = socket_extra.ReceiveFrom(data, ref remote_extra);
                  }

                  string text = Encoding.UTF8.GetString(data);
                     ;
                  //Debug.Log("EditorUDPClient: Packet >> " + is_extra + " " + text);
                  lock (queueLock)
                  {
                     if (is_extra)
                     {
                        receivedCPData.Enqueue(text);
                     }
                     else
                     {
                        receivedButtonData.Enqueue(text);
                     }
                  }
               }
                catch (Exception err) {
                    Debug.Log(err.ToString());
                }
            }
        }

         // used to send wrench data
         public void SendData(string message)
         {
            byte[] send_msg = Encoding.UTF8.GetBytes(message);
            socket_extra.SendTo(send_msg, remote_extra_write);
         }
   }

}

