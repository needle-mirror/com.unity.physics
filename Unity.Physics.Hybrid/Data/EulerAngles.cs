using System;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    [Serializable]
    struct EulerAngles : IEquatable<EulerAngles>
    {
        public static EulerAngles Default => new EulerAngles { RotationOrder = math.RotationOrder.ZXY };

        public float3 Value;
        [HideInInspector]
        public math.RotationOrder RotationOrder;

        internal void SetValue(quaternion value) => Value = ToEulerAngles(value, RotationOrder);

        static readonly float[] k_PitchScalars = { -1f,  1f,  1f, -1f, -1f,  1f };
        static readonly int[] k_PitchAxes      = {   1,   2,   0,   2,   0,   1 };
        static readonly int[] k_RollAxes       = {   2,   1,   2,   0,   1,   0 };
        static readonly int[] k_YawAxes        = {   0,   0,   1,   1,   2,   2 };

        static float3 ToEulerAngles(quaternion q, math.RotationOrder order)
        {
            var iOrder = (int)order;
            
            var rotationMatrix = new float3x3(math.normalizesafe(q));
            var pitchAngle = math.asin(math.clamp(
                k_PitchScalars[iOrder] * rotationMatrix[k_YawAxes[iOrder]][k_RollAxes[iOrder]], -1f, 1f
            ));
            float rollAngle, yawAngle;
            const float kPiOver2 = math.PI * 0.5f;
            if (pitchAngle < kPiOver2)
            {
                if (pitchAngle > -kPiOver2)
                {
                    yawAngle = math.atan2(
                        -k_PitchScalars[iOrder] * rotationMatrix[k_PitchAxes[iOrder]][k_RollAxes[iOrder]],
                        rotationMatrix[k_RollAxes[iOrder]][k_RollAxes[iOrder]]
                    );
                    rollAngle = math.atan2(
                        -k_PitchScalars[iOrder] * rotationMatrix[k_YawAxes[iOrder]][k_PitchAxes[iOrder]],
                        rotationMatrix[k_YawAxes[iOrder]][k_YawAxes[iOrder]]
                    );
                }
                else // non-unique solution
                {
                    rollAngle = 0f;
                    yawAngle = rollAngle - math.atan2(
                        k_PitchScalars[iOrder] * rotationMatrix[k_PitchAxes[iOrder]][k_YawAxes[iOrder]],
                        rotationMatrix[k_PitchAxes[iOrder]][k_PitchAxes[iOrder]]
                    );
                }
            }
            else // non-unique solution
            {
                rollAngle = 0f;
                yawAngle = math.atan2(
                    k_PitchScalars[iOrder] * rotationMatrix[k_PitchAxes[iOrder]][k_YawAxes[iOrder]],
                    rotationMatrix[k_PitchAxes[iOrder]][k_PitchAxes[iOrder]]
                ) - rollAngle;
            }
            // pack the angles into a vector and return the result
            return math.degrees(new float3
            {
                [k_RollAxes[iOrder]] = rollAngle,
                [k_YawAxes[iOrder]] = yawAngle,
                [k_PitchAxes[iOrder]] = pitchAngle
            });
        }

        public static implicit operator quaternion(EulerAngles euler) =>
            quaternion.Euler(math.radians(euler.Value), euler.RotationOrder);

        public bool Equals(EulerAngles other) => Value.Equals(other.Value) && RotationOrder == other.RotationOrder;

        public override bool Equals(object obj) => obj is EulerAngles other && Equals(other);

        public override int GetHashCode() => unchecked((int)math.hash(new float4(Value, (float)RotationOrder)));
    }
}
