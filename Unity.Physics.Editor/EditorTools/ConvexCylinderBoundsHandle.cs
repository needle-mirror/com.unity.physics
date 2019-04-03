using System;
using Unity.Mathematics;
using UnityEditor;
using UnityEditor.IMGUI.Controls;
using UnityEngine;

namespace Unity.Physics.Editor
{
    class ConvexCylinderBoundsHandle : PrimitiveBoundsHandle
    {
        public float ConvexRadius { get; set; }

        public float Height
        {
            get => GetSize().z;
            set
            {
                var size = GetSize();
                size.z = math.max(0f, value);
                SetSize(size);
            }
        }

        public float Radius
        {
            get => GetSize().x;
            set
            {
                var size = GetSize();
                size.x = size.y = math.max(0f, value);
                SetSize(size);
            }
        }

        const int k_NumSpans = ConvexConvexManifoldQueries.Manifold.k_MaxNumContacts;
        static readonly Vector3[] s_Points = new Vector3[k_NumSpans * 6];
        static readonly Vector3[] s_PointsWithRadius = new Vector3[k_NumSpans * 10];

        protected override void DrawWireframe()
        {
            var halfHeight = new float3(0f, 0f, Height * 0.5f);
            var t = 2f * 31 / k_NumSpans;
            var prevXY = new float3((float)math.cos(math.PI * t), (float)math.sin(math.PI * t), 0f) * Radius;
            int step;
            Vector3[] points;
            if (ConvexRadius > 0f)
            {
                points = s_PointsWithRadius;
                step = 10;
            }
            else
            {
                points = s_Points;
                step = 6;
            }
            for (var i = 0; i < k_NumSpans; ++i)
            {
                t = 2f * i / k_NumSpans;
                var xy = new float3((float)math.cos(math.PI * t), (float)math.sin(math.PI * t), 0f) * Radius;
                var xyCvx = math.normalizesafe(xy) * ConvexRadius;
                var idx = i * step;
                // height
                points[idx++] = xy + halfHeight - new float3 { z = ConvexRadius };
                points[idx++] = xy - halfHeight + new float3 { z = ConvexRadius };
                // top
                points[idx++] = prevXY + halfHeight - xyCvx;
                points[idx++] = xy + halfHeight - xyCvx;
                // bottom
                points[idx++] = prevXY - halfHeight - xyCvx;
                points[idx++] = xy - halfHeight - xyCvx;
                // convex
                if (ConvexRadius > 0f)
                {
                    // top
                    points[idx++] = prevXY + halfHeight - new float3 { z = ConvexRadius };
                    points[idx++] = xy + halfHeight - new float3 { z = ConvexRadius };
                    // bottom
                    points[idx++] = prevXY - halfHeight + new float3 { z = ConvexRadius };
                    points[idx++] = xy - halfHeight + new float3 { z = ConvexRadius };
                    // corners
                    var normal = math.cross(new float3(0f, 0f, 1f), xy);
                    var p = new float3(xy.x, xy.y, halfHeight.z) - new float3(xyCvx.x, xyCvx.y, ConvexRadius);
                    Handles.DrawWireArc(p, normal, xy, -90f, ConvexRadius);
                    p *= new float3(1f, 1f, -1f);
                    Handles.DrawWireArc(p, normal, xy, 90f, ConvexRadius);
                }
                prevXY = xy;
            }
            Handles.DrawLines(points);
        }
    }
}