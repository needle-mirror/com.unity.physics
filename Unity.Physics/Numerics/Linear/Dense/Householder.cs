using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

namespace Unity.Numerics.Linear.Dense.Primitives
{
    [BurstCompile]
    [GenerateTestsForBurstCompatibility]
    /// <summary>
    /// Householder reflectors
    /// </summary>
    internal struct Householder
    {
        /// <summary>
        /// This computes an elementary reflector H, such that
        ///
        ///   H * [alpha;x] = [beta;0] and H^T H = I,
        ///
        /// where alpha and beta are scalars, and x is an n-1 element vector.
        /// H is represented in the form
        ///
        ///   H = I - tau [1;v] [1;v^T],
        ///
        /// where tau is a scalar and v is an n-1 element vector.  If all elements
        /// of x are zero, then tau=0 and H=I, otherwise 1 <= tau <= 2.
        ///
        /// On return, the elements of this vector are replaced by x.
        /// </summary>
        /// <param name="v"></param>
        /// <param name="alpha"></param>
        /// <param name="beta"></param>
        /// <param name="tau"></param>
        /// <remarks>LAPACK equivalent: SLARFG</remarks>
        public static void ComputeReflector(in Vector v, float alpha, out float beta, out float tau)
        {
            if (v.Dimension < 1)
            {
                beta = alpha;
                tau = 0;
                return;
            }

            var xnorm = v.Norm();
            if (xnorm == 0)
            {
                beta = alpha;
                tau = 0;
                return;
            }

            beta = -Utility.CopySign(Utility.Magnitude(new float2(alpha, xnorm)), alpha);

            float sfmin = Utility.SafeMin;
            int iterations = 0;
            if (math.abs(beta) < sfmin)
            {
                // xnorm and beta may be inaccurate
                while (math.abs(beta) < sfmin && iterations++ < 20)
                {
                    var rsfmin = 1.0f / sfmin;
                    v.Scale(rsfmin);
                    beta *= rsfmin;
                    alpha *= rsfmin;
                }

                xnorm = v.Norm();
                beta = -Utility.CopySign(Utility.Magnitude(new float2(alpha, xnorm)), alpha);
            }

            tau = (beta - alpha) / beta;
            v.Scale(1.0f / (alpha - beta));

            // if beta is subnormal (see above) it may lose accuracy
            for (int i = 0; i < iterations; i++)
            {
                beta *= sfmin;
            }
        }

        /// <summary>
        /// replaces matrix C with either H C or C H depending on the value of 'side',
        /// where
        ///
        ///   H = I - tau v v^T
        /// </summary>
        /// <param name="C"></param>
        /// <param name="side"></param>
        /// <param name="v"></param>
        /// <param name="tau"></param>
        /// <remarks>LAPACK equivalent: SLARF</remarks>
        public static void ApplyReflector(in Matrix C, Side side, in Vector v, float tau)
        {
            int lastV = 0;
            int lastC = 0;

            if (tau != 0)
            {
                lastV = (side == Side.Left ? C.NumRows : C.NumCols) - 1;

                // look for the last nonzero element of v
                while (lastV >= 0 && v[lastV] == 0)
                {
                    lastV--;
                }

                if (side == Side.Left)
                {
                    // scan for the last nonzero column in M(0:lastV,*)
                    lastC = C.LastNZColumnIndex(0, 0, lastV + 1, C.NumCols);
                }
                else
                {
                    // scan for the last nonzero row in M(*,0:lastV)
                    lastC = C.LastNZRowIndex(0, 0, C.NumRows, lastV + 1);
                }

                lastV++;
                lastC++;
            }

            if (side == Side.Left)
            {
                // H * C
                if (lastV > 0)
                {
                    var work = v.Heap.Vector(lastC);
                    var subC = C.Submatrix(0, 0, lastV, lastC);
                    var subv = v.Subvector(0, lastV);
                    work.ScaleAndAddProduct(
                        subC,
                        Op.Transpose,
                        subv,
                        1.0f, 0.0f);
                    subC.AddOuterProduct(subv, work, -tau);
                    subC.Dispose();
                    work.Dispose();
                }
            }
            else
            {
                // C * H
                if (lastV > 0)
                {
                    var work = v.Heap.Vector(lastC);
                    var subC = C.Submatrix(0, 0, lastC, lastV);
                    var subv = v.Subvector(0, lastV);
                    work.ScaleAndAddProduct(
                        subC,
                        Op.None,
                        subv,
                        1.0f, 0.0f);
                    subC.AddOuterProduct(work, subv, -tau);
                    subC.Dispose();
                    work.Dispose();
                }
            }
        }
    }
}
