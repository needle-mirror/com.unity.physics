using System;
using Unity.Mathematics;
using Unity.Physics.Authoring;
using UnityEditor;
using UnityEditor.IMGUI.Controls;
using UnityEngine;

namespace Unity.Physics.Editor
{
    class ConvexCylinderBoundsHandle : PrimitiveBoundsHandle
    {
        public ConvexCylinderBoundsHandle() => midpointHandleDrawFunction = DoMidpointHandle;

        void DoMidpointHandle(int controlID, Vector3 position, Quaternion rotation, float size, EventType eventType)
        {
            var direction = (HandleDirection)(controlID - m_FirstControlID);
            switch (direction)
            {
                case HandleDirection.NegativeX:
                case HandleDirection.PositiveX:
                    break;
                case HandleDirection.NegativeY:
                case HandleDirection.PositiveY:
                    break;
                case HandleDirection.NegativeZ:
                case HandleDirection.PositiveZ:
                    break;
                default:
                    Debug.LogException(
                        new NotImplementedException(
                            $"Unknown handle direction {direction}. " +
                            $"Did you forget to call {nameof(DrawHandle)}() during {EventType.Layout} phase?"
                        )
                    );
                    break;
            }
            Handles.DotHandleCap(controlID, position, rotation, size, eventType);
        }

        public float ConvexRadius
        {
            get => m_ConvexRadius;
            set
            {
                m_ConvexRadius = math.max(0f, value);
                Height = math.max(Height, ConvexRadius * 2f);
                Radius = math.max(Radius, ConvexRadius);
            }
        }

        float m_ConvexRadius = PhysicsShape.k_DefaultConvexRadius;

        public float Height
        {
            get => GetSize().z;
            set
            {
                var size = GetSize();
                size.z = math.max(math.max(0f, 2f * ConvexRadius), value);
                SetSize(size);
            }
        }

        public float Radius
        {
            get => GetSize().x * 0.5f;
            set
            {
                var size = GetSize();
                size.x = size.y = math.max(0f, math.max(value, ConvexRadius) * 2f);
                SetSize(size);
            }
        }

        const int k_NumSpans = ConvexConvexManifoldQueries.Manifold.k_MaxNumContacts;
        static readonly Vector3[] s_Points = new Vector3[k_NumSpans * 6];
        static readonly Vector3[] s_PointsWithRadius = new Vector3[k_NumSpans * 10];
        int m_FirstControlID;

        protected override void DrawWireframe()
        {
            m_FirstControlID = GUIUtility.GetControlID(GetHashCode(), FocusType.Passive) - 6;

            var halfHeight = new float3(0f, 0f, Height * 0.5f);
            var t = 2f * 31 / k_NumSpans;
            var prevXY = new float3(math.cos(math.PI * t), math.sin(math.PI * t), 0f) * Radius;
            var prevXYCvx = math.normalizesafe(prevXY) * ConvexRadius;
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
                var xy = new float3(math.cos(math.PI * t), math.sin(math.PI * t), 0f) * Radius;
                var xyCvx = math.normalizesafe(xy) * ConvexRadius;
                var idx = i * step;
                var ctr = (float3)center;
                // height
                points[idx++] = ctr + xy + halfHeight - new float3 { z = ConvexRadius };
                points[idx++] = ctr + xy - halfHeight + new float3 { z = ConvexRadius };
                // top
                points[idx++] = prevXY + halfHeight - prevXYCvx;
                points[idx++] = xy + halfHeight - xyCvx;
                // bottom
                points[idx++] = prevXY - halfHeight - prevXYCvx;
                points[idx++] = xy - halfHeight - xyCvx;
                // convex
                if (ConvexRadius > 0f)
                {
                    // top
                    points[idx++] = ctr + prevXY + halfHeight - new float3 { z = ConvexRadius };
                    points[idx++] = ctr + xy + halfHeight - new float3 { z = ConvexRadius };
                    // bottom
                    points[idx++] = ctr + prevXY - halfHeight + new float3 { z = ConvexRadius };
                    points[idx++] = ctr + xy - halfHeight + new float3 { z = ConvexRadius };
                    // corners
                    var normal = math.cross(new float3(0f, 0f, 1f), xy);
                    var p = new float3(xy.x, xy.y, halfHeight.z) - new float3(xyCvx.x, xyCvx.y, ConvexRadius);
                    Handles.DrawWireArc(ctr + p, normal, xy, -90f, ConvexRadius);
                    p *= new float3(1f, 1f, -1f);
                    Handles.DrawWireArc(ctr + p, normal, xy, 90f, ConvexRadius);
                }
                prevXY = xy;
                prevXYCvx = xyCvx;
            }
            Handles.DrawLines(points);
        }

        protected override Bounds OnHandleChanged(HandleDirection handle, Bounds boundsOnClick, Bounds newBounds)
        {
            const int k_DirectionX = 0;
            const int k_DirectionY = 1;
            const int k_DirectionZ = 2;

            var changedAxis = k_DirectionX;
            var otherRadiusAxis = k_DirectionY;
            switch (handle)
            {
                case HandleDirection.NegativeY:
                case HandleDirection.PositiveY:
                    changedAxis = k_DirectionY;
                    otherRadiusAxis = k_DirectionX;
                    break;
                case HandleDirection.NegativeZ:
                case HandleDirection.PositiveZ:
                    changedAxis = k_DirectionZ;
                    break;
            }

            var upperBound = newBounds.max;
            var lowerBound = newBounds.min;

            var convexDiameter = 2f * ConvexRadius;

            // ensure changed dimension cannot be made less than convex diameter
            if (upperBound[changedAxis] - lowerBound[changedAxis] < convexDiameter)
            {
                switch (handle)
                {
                    case HandleDirection.PositiveX:
                    case HandleDirection.PositiveY:
                    case HandleDirection.PositiveZ:
                        upperBound[changedAxis] = lowerBound[changedAxis] + convexDiameter;
                        break;
                    default:
                        lowerBound[changedAxis] = upperBound[changedAxis] - convexDiameter;
                        break;
                }
            }

            // ensure radius changes uniformly
            if (changedAxis != k_DirectionZ)
            {
                var rad = 0.5f * (upperBound[changedAxis] - lowerBound[changedAxis]);

                lowerBound[otherRadiusAxis] = center[otherRadiusAxis] - rad;
                upperBound[otherRadiusAxis] = center[otherRadiusAxis] + rad;
            }

            return new Bounds((upperBound + lowerBound) * 0.5f, upperBound - lowerBound);
        }
    }
}