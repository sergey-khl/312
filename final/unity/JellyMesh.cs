/*
Group Members: Sergey Khlynovskiy, Jerrica Yang

Date: 2024-12-09

Final Project

File Summary:

     Add jelly physics to an object. Used for the green block in our scene

Used Resources/Collaborators:
	https://www.youtube.com/watch?v=Kwh4TkQqqf8

I/we hereby certify that I/we have produced the following solution 
using only the resources listed above in accordance with the 
CMPUT 312 collaboration policy.
*/
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace DVRK
{
   public class JellyMesh : MonoBehaviour
   {
      public float Intensity = 1.5f;
      public float Mass = 0.5f;
      public float stiffness = 5f;
      public float damping = 0.6f;
      private Mesh OriginalMesh, MeshClone;
      private MeshRenderer renderer;
      private JellyVertex[] jv;
      private Vector3[] vertexArray;


      //citation: https://www.youtube.com/watch?v=Kwh4TkQqqf8&t=20s

      // Start is called before the first frame update
      void Start()
      {
         OriginalMesh = GetComponent<MeshFilter>().sharedMesh;
         MeshClone = Instantiate(OriginalMesh);
         GetComponent<MeshFilter>().sharedMesh = MeshClone;
         renderer = GetComponent<MeshRenderer>();

         jv = new JellyVertex[MeshClone.vertices.Length];
         for (int i = 0; i < MeshClone.vertices.Length; i++)
         {
            jv[i] = new JellyVertex(i, transform.TransformPoint(MeshClone.vertices[i]));
         }
      }

      // Update is called once per frame
      void FixedUpdate()
      {
         vertexArray = OriginalMesh.vertices;
         for (int i = 0; i < jv.Length; i++)
         {
            Vector3 target = transform.TransformPoint(vertexArray[jv[i].ID]);
            float intensity = (1 - (renderer.bounds.max.y - target.y) / renderer.bounds.size.y) * Intensity;
            jv[i].Shake(target, Mass, stiffness, damping);
            target = transform.InverseTransformPoint(jv[i].Position);
            vertexArray[jv[i].ID] = Vector3.Lerp(vertexArray[jv[i].ID], target, intensity);
         }
         MeshClone.vertices = vertexArray;
      }

      public class JellyVertex
      {
         public int ID;
         public Vector3 Position;
         public Vector3 velocity, Force;

         public JellyVertex(int _id, Vector3 _pos)
         {
            ID = _id;
            Position = _pos;
         }

         public void Shake(Vector3 target, float m, float s, float d)
         {
            Force = (target - Position) * s;
            velocity = (velocity + Force / m) * d;
            Position += velocity;
            if ((velocity + Force + Force / m).magnitude < 0.001f)
            {
               Position = target;
            }
         }
      }


   }
}