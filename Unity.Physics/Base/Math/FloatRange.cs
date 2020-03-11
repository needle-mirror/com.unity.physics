using System;
using Unity.Mathematics;

namespace Unity.Physics
{
    public static partial class Math
    {
        /// <summary>
        /// Range of possible values for some constrained parameter.
        /// </summary>
        public struct FloatRange : IEquatable<FloatRange>
        {
            public float Min;
            public float Max;

            public FloatRange(float min, float max)
            {
                Min = min;
                Max = max;
            }

            public bool Equals(FloatRange other) => Min.Equals(other.Min) && Max.Equals(other.Max);

            public override bool Equals(object obj) => obj is FloatRange other && Equals(other);

            public override int GetHashCode() => unchecked((int)math.hash(new float2(Min, Max)));

            public static implicit operator float2 (FloatRange range) => new float2(range.Min, range.Max);

            public static implicit operator FloatRange(float2 f) => new FloatRange { Min = f.x, Max = f.y };

            public override string ToString() => $"FloatRange {{ Min = {Min}, Max = {Max} }}";
        }
    }
}
