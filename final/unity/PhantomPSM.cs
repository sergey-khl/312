/*
Group Members: Sergey Khlynovskiy, Jerrica Yang

Date: 2024-12-09

Final Project

File Summary:

   controlls the joints of the psm arm according to the phantom.
   specific controls can be found in our report but basically we use the
   x, y, z movement to determine joint movment. Pressing down button moves the tip.
   pressing both button activates clutch. Clicking top button switches arms.

Used Resources/Collaborators:
	N/A

I/we hereby certify that I/we have produced the following solution 
using only the resources listed above in accordance with the 
CMPUT 312 collaboration policy.
*/
using Newtonsoft.Json;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.VersionControl;
using UnityEngine;

namespace DVRK
{
   public class PhantomPSM : MonoBehaviour
   {
      private URDFRobot currentRobot;

      private bool upButtonPressed = false;
      private bool downButtonPressed = false;

      public float deltaAngle = 5.0f;
      public float deltaInsertion = 0.005f; // 0-0.24
      public float clickTimeout = 0.5f; // how long the button click can be to switch between robots
      public float pressTime = 0;
      public bool clicked = false;

      private UDPClient udpClient;
      public class MeasuredCPData
      {
         public MeasuredCP measured_cp;
      }

      [System.Serializable]
      public class MeasuredCP
      {
         public bool AutomaticTimestamp;
         public string MovingFrame;
         public Position Position;
         public string ReferenceFrame;
         public double Timestamp;
         public bool Valid;
      }
      public class Position
      {
         public float[][] Rotation;
         public float[] Translation;
      }
      // Start is called before the first frame update
      void Start()
      {
         InitializePhantom();
         Debug.Log("RESETTING POSITION");
         transform.position = new Vector3(100f, 1000f, 100f);
      }

      // Update is called once per frame
      void Update()
      {
         // switch robots if detected a top button click
         if (clicked)
         {
            Vector3 old_translation = currentRobot.previousTranslation;
            Quaternion old_rotation = currentRobot.previousRotation;

            udpClient.currentRobot = (udpClient.currentRobot + 1) % udpClient.controllableRobots.Count;
            currentRobot = getCurrentRobot();

            // have to update the past robots old translation and update to match the current position of the phantom
            currentRobot.previousTranslation = old_translation;
            currentRobot.previousRotation = old_rotation;

            // reset click
            clicked = false;
            pressTime = 0;
         }

         // get the latest phantom position and orientation
         List<string> messages;
         messages = udpClient.GetLatestCPPacket(1);

         foreach (string msg in messages)
         { ReadPhantomData(msg); }
         
         // get phantom latest button data
         messages = udpClient.GetLatestButtonPacket(1);

         foreach (string msg in messages)
         { ReadButtonData(msg); }
      }

      URDFRobot getCurrentRobot()
      {
         // finds the currently controller psm arm
         if (udpClient == null)
         {
            Debug.Log("udp client not defined");
            return null;
         }
         if (udpClient.controllableRobots.Count == 0)
         {
            Debug.LogError("no controllable robots");
            return null;
         }
         return udpClient.controllableRobots[udpClient.currentRobot];
      }

      private void InitializePhantom()
      {
         udpClient = GetComponent<UDPClient>();
         currentRobot = getCurrentRobot();

         // initialize the udp client for each robot and initialize joints
         foreach (URDFRobot robot in udpClient.controllableRobots)
         {
            robot.udpClient = udpClient;
            currentRobot.previousRotation = Quaternion.identity;
            currentRobot.previousTranslation = Vector3.zero;
         }

         Debug.Log("Initializing phantom device...");
      }

      private void updateJointAngle(Vector3 translation, Quaternion rotation)
      {
         if (currentRobot.independentJoints != null)
         {
            for (int i = 0; i < currentRobot.independentJoints.Count; i++)
            {
               // clutch. move the phantom freely without affecting the simulation
               if (downButtonPressed && upButtonPressed)
               {
                  continue;
               }
               // tip movements
               if (downButtonPressed)
               {
                  // outer roll joint: just by rotations - we can update this whenever
                  if (i == 3)
                  {
                     updateJointIfChanged(translation[2] - currentRobot.previousTranslation[2], i);
                  }
                  // outer wrist pitch: hold upbutton and move in z-axis
                  if (i == 4)
                  {
                     updateJointIfChanged(translation[1] - currentRobot.previousTranslation[1], i);
                  }
                  // outer wrist yaw: hold downbutton and move in x-axis
                  if (i == 5)
                  {
                     updateJointIfChanged(translation[0] - currentRobot.previousTranslation[0], i);
                  }
               } else
               // whole psm movements
               {
                  if (i == 0 )
                  {
                     updateJointIfChanged(translation[i] - currentRobot.previousTranslation[i], i);
                  }
                  if (i == 1)
                  {
                     updateJointIfChanged(translation[2] - currentRobot.previousTranslation[2], 1);
                  }
                  if (i == 2)
                  {
                     updateJointIfChanged(-(translation[1] - currentRobot.previousTranslation[1]), 2);
                  }
               }
            }
         }
      }

      private void updateJointIfChanged(float delta_joint, int index) {
         int s = 500; // for scaling, we can redo this part if we want to hold and move further from the point
         if (Mathf.Abs(delta_joint) > 0)
         {
            // scale factor needs to be less for insertion
            if (index == 2)
            {
               s = 1;
            }

            currentRobot.independentJoints[index].SetJointValue(currentRobot.independentJoints[index].currentJointValue + delta_joint*s);
         }
      }
      private void ReadPhantomData(string message)
      {
         if (message.Contains("measured_cp"))
         {
            // Deserialize as MeasuredCP
            MeasuredCPData data = JsonConvert.DeserializeObject<MeasuredCPData>(message);

            // Extract the translation and rotation data
            Vector3 translation = new Vector3(
                data.measured_cp.Position.Translation[0],
                data.measured_cp.Position.Translation[1],
                data.measured_cp.Position.Translation[2]
            );

            // Extract the rotation matrix and convert it to a quaternion
            Matrix4x4 rotationMatrix = new Matrix4x4(
                new Vector4(data.measured_cp.Position.Rotation[0][0], data.measured_cp.Position.Rotation[0][1], data.measured_cp.Position.Rotation[0][2], 0),
                new Vector4(data.measured_cp.Position.Rotation[1][0], data.measured_cp.Position.Rotation[1][1], data.measured_cp.Position.Rotation[1][2], 0),
                new Vector4(data.measured_cp.Position.Rotation[2][0], data.measured_cp.Position.Rotation[2][1], data.measured_cp.Position.Rotation[2][2], 0),
                new Vector4(0, 0, 0, 1)
            );

            Quaternion rotation = rotationMatrix.rotation;


            if (currentRobot.previousTranslation != translation || currentRobot.previousRotation != rotation)
            {
               updateJointAngle(translation, rotation);
            }

            // we really only use the translation
            currentRobot.previousRotation = rotation;
            currentRobot.previousTranslation = translation;
         }
         else
         {
            Debug.LogError("Unknown JSON format.");
         }
      }
      private void ReadButtonData(string message)
      {

         // 1 if pressed, 0 otherewise
         string[] result = message.Split(',');
         if (result.Length > 1)
         {
            if (result[0] == "2")
            {
               // for some reason if statemnts dont work so need contains
               upButtonPressed = result[1].Contains("1");

               // logic for detecting click
               if (upButtonPressed)
               {
                  pressTime = Time.time;
               }
               if (!upButtonPressed && Time.time - pressTime <= clickTimeout)
               {
                  clicked = true;
               }
               
            } else
            {
               downButtonPressed = result[1].Contains("1");
            }
         }
      }
   }




}
