/*
Group Members: Sergey Khlynovskiy, Jerrica Yang

Date: 2024-12-09

Final Project

File Summary:

   attatched to the psm tips to detect when a collision happens and sends a udp message
   with the experienced force direction and magnitude using f=m*a

Used Resources/Collaborators:
	N/A

I/we hereby certify that I/we have produced the following solution 
using only the resources listed above in accordance with the 
CMPUT 312 collaboration policy.
*/
using System.Collections;
using System.Collections.Generic;
using System.Runtime.ExceptionServices;
using UnityEditor.VersionControl;
using UnityEngine;

namespace DVRK
{
   public class PSMCollider : MonoBehaviour
   {
      public URDFRobot controlledRobot;
      private UDPClient udpClient;

      private Dictionary<string, Vector3> knownVelocities = new Dictionary<string, Vector3>
    {
        { "table", new Vector3(0, -1, 0) }
    };


      void Start()
      {
         if (controlledRobot == null)
         {
            Debug.Log("controlled robot not assigned");
         }
      }

      void Update()
      {
         // send a message of the collision for the current selected arm
         if (udpClient != null && controlledRobot == udpClient.controllableRobots[udpClient.currentRobot])
         {
            string message = controlledRobot.collisionDirection.ToString() + "_" + controlledRobot.collisionMagnitude.ToString();

            udpClient.SendData(message);
         } else
         {
            // phantom psm will update this on startup
            udpClient = controlledRobot.udpClient;
         }
      }

      void OnCollisionEnter(Collision collision)
      {
         checkForces(collision, true);
      }

      void OnCollisionStay(Collision collision)
      {
         checkForces(collision);
      }

      void checkForces(Collision collision, bool first = false)
      {
         Rigidbody otherRigidbody = collision.rigidbody;
         if (otherRigidbody != null)
         {
            // Get the relative velocity at the collision point
            Vector3 relativeVelocity = collision.relativeVelocity;
            // use a known velocity for buggy objects like tables
            if (knownVelocities.ContainsKey(otherRigidbody.name))
            {
               relativeVelocity = knownVelocities[otherRigidbody.name];
            }

            // Calculate the collision force (F = m * a, approximated by velocity change)
            float otherMass = otherRigidbody.mass;
            Vector3 collisionForce = otherMass * relativeVelocity / Time.fixedDeltaTime;

            // average the collision force magnitude and directions while we are colliding to smoothen effects
            if (first)
            {
               controlledRobot.totalForce = collisionForce;
               controlledRobot.collisionDirection = collisionForce.normalized;
               controlledRobot.collisionMagnitude = collisionForce.magnitude;
               controlledRobot.collisionCount = 1;
            }
            else
            {
               controlledRobot.totalForce += collisionForce;
               controlledRobot.collisionDirection = Vector3.Normalize(controlledRobot.totalForce);
               controlledRobot.collisionMagnitude = controlledRobot.totalForce.magnitude / controlledRobot.collisionCount;
               controlledRobot.collisionCount++;
            }
            // Debug.Log(otherRigidbody.name + " " + controlledRobot.collisionDirection + " " + controlledRobot.collisionMagnitude);

         }
         else
         {
            Debug.LogWarning("The other object does not have a Rigidbody.");
         }
      }

      void OnCollisionExit(Collision collision)
      {
         // 0 out forces when we stop colliding
         controlledRobot.collisionDirection = Vector3.zero;
         controlledRobot.collisionMagnitude = 0.0f;
      }
   }

}
