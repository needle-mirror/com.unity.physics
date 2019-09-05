using System;
using Unity.Mathematics;
using UnityEditor;
using UnityEditor.IMGUI.Controls;
using UnityEngine;

namespace Unity.Physics.Editor
{
    class BeveledCylinderBoundsHandle : PrimitiveBoundsHandle
    {
        public BeveledCylinderBoundsHandle() => midpointHandleDrawFunction = DoMidpointHandle;

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

        public float BevelRadius
        {
            get => m_BevelRadius;
            set
            {
                m_BevelRadius = math.max(0f, value);
                Height = math.max(Height, BevelRadius * 2f);
                Radius = math.max(Radius, BevelRadius);
            }
        }

        float m_BevelRadius = ConvexHullGenerationParameters.Default.BevelRadius;

        public float Height
        {
            get => GetSize().z;
            set
            {
                var size = GetSize();
                size.z = math.max(math.max(0f, 2f * BevelRadius), value);
                SetSize(size);
            }
        }

        public float Radius
        {
            get => GetSize().x * 0.5f;
            set
            {
                var size = GetSize();
                size.x = size.y = math.max(0f, math.max(value, BevelRadius) * 2f);
                SetSize(size);
            }
        }

        public int SideCount
        {
            get => m_SideCount;
            set
            {
                if (value == m_SideCount)
                    return;

                m_SideCount = value;

                Array.Resize(ref m_Points, m_SideCount * 6);
                Array.Resize(ref m_PointsWithRadius, m_SideCount * 10);
            }
        }
        int m_SideCount;

        Vector3[] m_Points = Array.Empty<Vector3>();
        Vector3[] m_PointsWithRadius = Array.Empty<Vector3>();
        int m_FirstControlID;

        protected override void DrawWireframe()
        {
            m_FirstControlID = GUIUtility.GetControlID(GetHashCode(), FocusType.Passive) - 6;

            var halfHeight = new float3(0f, 0f, Height * 0.5f);
            var t = 2f * (m_SideCount - 1) / m_SideCount;
            var prevXY = new float3(math.cos(math.PI * t), math.sin(math.PI * t), 0f) * Radius;
            var prevXYCvx = math.normalizesafe(prevXY) * BevelRadius;
            int step;
            Vector3[] points;
            if (BevelRadius > 0f)
            {
                points = m_PointsWithRadius;
                step = 10;
            }
            else
            {
                points = m_Points;
                step = 6;
            }
            for (var i = 0; i < m_SideCount; ++i)
            {
                t = 2f * i / m_SideCount;
                var xy = new float3(math.cos(math.PI * t), math.sin(math.PI * t), 0f) * Radius;
                var xyCvx = math.normalizesafe(xy) * BevelRadius;
                var idx = i * step;
                var ctr = (float3)center;
                // height
                points[idx++] = ctr + xy + halfHeight - new float3 { z = BevelRadius };
                points[idx++] = ctr + xy - halfHeight + new float3 { z = BevelRadius };
                // top
                points[idx++] = prevXY + halfHeight - prevXYCvx;
                points[idx++] = xy + halfHeight - xyCvx;
                // bottom
                points[idx++] = prevXY - halfHeight - prevXYCvx;
                points[idx++] = xy - halfHeight - xyCvx;
                // convex
                if (BevelRadius > 0f)
                {
                    // top
                    points[idx++] = ctr + prevXY + halfHeight - new float3 { z = BevelRadius };
                    points[idx++] = ctr + xy + halfHeight - new float3 { z = BevelRadius };
                    // bottom
                    points[idx++] = ctr + prevXY - halfHeight + new float3 { z = BevelRadius };
                    points[idx++] = ctr + xy - halfHeight + new float3 { z = BevelRadius };
                    // corners
                    var normal = math.cross(new float3(0f, 0f, 1f), xy);
                    var p = new float3(xy.x, xy.y, halfHeight.z) - new float3(xyCvx.x, xyCvx.y, BevelRadius);
                    Handles.DrawWireArc(ctr + p, normal, xy, -90f, BevelRadius);
                    p *= new float3(1f, 1f, -1f);
                    Handles.DrawWireArc(ctr + p, normal, xy, 90f, BevelRadius);
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

            var convexDiameter = 2f * BevelRadius;

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