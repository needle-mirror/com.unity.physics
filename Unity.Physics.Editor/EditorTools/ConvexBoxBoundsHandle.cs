using System;
using Unity.Mathematics;
using UnityEditor;
using UnityEditor.IMGUI.Controls;
using UnityEngine;

namespace Unity.Physics.Editor
{
    class ConvexBoxBoundsHandle : BoxBoundsHandle
    {
        public float ConvexRadius { get; set; }

        protected override void DrawWireframe()
        {
            if (ConvexRadius <= 0f)
            {
                base.DrawWireframe();
                return;
            }

            var center = (float3)this.center;
            var size = (float3)this.size;
            DrawFace(center, size * new float3( 1f,  1f,  1f), 0, 1, 2);
            DrawFace(center, size * new float3(-1f,  1f,  1f), 0, 1, 2);
            DrawFace(center, size * new float3( 1f,  1f,  1f), 1, 0, 2);
            DrawFace(center, size * new float3( 1f, -1f,  1f), 1, 0, 2);
            DrawFace(center, size * new float3( 1f,  1f,  1f), 2, 0, 1);
            DrawFace(center, size * new float3( 1f,  1f, -1f), 2, 0, 1);

            var corner = 0.5f * size - new float3(1f) * ConvexRadius;
            var rgt = new float3(1f, 0f, 0f);
            var up = new float3(0f, 1f, 0f);
            var fwd = new float3(0f, 0f, 1f);
            DrawCorner(center + corner * new float3( 1f,  1f,  1f), quaternion.LookRotation( fwd,  up));
            DrawCorner(center + corner * new float3(-1f,  1f,  1f), quaternion.LookRotation(-rgt,  up));
            DrawCorner(center + corner * new float3( 1f, -1f,  1f), quaternion.LookRotation( rgt, -up));
            DrawCorner(center + corner * new float3( 1f,  1f, -1f), quaternion.LookRotation(-fwd, rgt));
            DrawCorner(center + corner * new float3(-1f, -1f,  1f), quaternion.LookRotation( fwd, -up));
            DrawCorner(center + corner * new float3(-1f,  1f, -1f), quaternion.LookRotation(-fwd,  up));
            DrawCorner(center + corner * new float3( 1f, -1f, -1f), quaternion.LookRotation(-fwd, -up));
            DrawCorner(center + corner * new float3(-1f, -1f, -1f), quaternion.LookRotation(-rgt, -up));
        }

        static Vector3[] s_FacePoints = new Vector3[8];

        void DrawFace(float3 center, float3 size, int a, int b, int c)
        {
            size *= 0.5f;
            var ctr = center + new float3 { [a] = size[a] };
            var i = 0;
            size -= new float3(ConvexRadius);
            s_FacePoints[i++] = ctr + new float3 { [b] =  size[b], [c] =  size[c] };
            s_FacePoints[i++] = ctr + new float3 { [b] = -size[b], [c] =  size[c] };
            s_FacePoints[i++] = ctr + new float3 { [b] = -size[b], [c] =  size[c] };
            s_FacePoints[i++] = ctr + new float3 { [b] = -size[b], [c] = -size[c] };
            s_FacePoints[i++] = ctr + new float3 { [b] = -size[b], [c] = -size[c] };
            s_FacePoints[i++] = ctr + new float3 { [b] =  size[b], [c] = -size[c] };
            s_FacePoints[i++] = ctr + new float3 { [b] =  size[b], [c] = -size[c] };
            s_FacePoints[i++] = ctr + new float3 { [b] =  size[b], [c] =  size[c] };
            Handles.DrawLines(s_FacePoints);
        }

        void DrawCorner(float3 point, quaternion orientation)
        {
            var rgt = math.mul(orientation, new float3(1f, 0f, 0f));
            var up = math.mul(orientation, new float3(0f, 1f, 0f));
            var fwd = math.mul(orientation, new float3(0f, 0f, 1f));
            Handles.DrawWireArc(point, fwd, rgt, 90f, ConvexRadius);
            Handles.DrawWireArc(point, rgt, up, 90f, ConvexRadius);
            Handles.DrawWireArc(point, up, fwd, 90f, ConvexRadius);
        }
    }
}