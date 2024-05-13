using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Octree
{
    public static class CustMesh
    {
        public static Vector3[] vertices;
        public static Vector3 minPos;
        public static Vector3 maxPos;

        // Find mesh vertices
        public static void Getvertices(Vector3[] pos)
        {
            minPos = pos[0];
            maxPos = pos[0];


            for (int i = 0; i < pos.Length; i++)
            {
                Vector3 positions = pos[i];
                minPos.x = Mathf.Min(minPos.x, positions.x);
                minPos.y = Mathf.Min(minPos.y, positions.y);
                minPos.z = Mathf.Min(minPos.z, positions.z);

                maxPos.x = Mathf.Max(maxPos.x, positions.x);
                maxPos.y = Mathf.Max(maxPos.y, positions.y);
                maxPos.z = Mathf.Max(maxPos.z, positions.z);
            }

        }
    }
}
