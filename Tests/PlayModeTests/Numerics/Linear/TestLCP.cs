using System;
using NUnit.Framework;
using Unity.Numerics.Linear.Dense.Primitives;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Numerics.Memory;

namespace Unity.Numerics.Linear.Tests
{
    public unsafe class TestLCP
    {
        static float k_LCPEpsilon = 1e-4f;

        [BurstCompile(CompileSynchronously = true)]
        [GenerateTestsForBurstCompatibility]
        struct LCPSolveJob : IJob
        {
            public MemoryManager heap;
            [ReadOnly] public Matrix M;
            [ReadOnly] public Vector q;
            public Vector z;
            public Vector w;
            public NativeArray<LCP.LCPIndexFlag> indexSetArray;
            public NativeReference<bool> success;

            public void Execute()
            {
                success.Value = LCP.SolveLCP(M, q, ref indexSetArray, ref w, ref z);
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        [GenerateTestsForBurstCompatibility]
        struct MLCPSolveJob : IJob
        {
            public MemoryManager heap;
            [ReadOnly] public Matrix M;
            [ReadOnly] public Vector q;
            [ReadOnly] public Vector l;
            [ReadOnly] public Vector u;
            public Vector z;
            public Vector w;
            public NativeArray<LCP.MLCPIndexFlag> indexSetArray;
            public NativeReference<bool> success;

            public void Execute()
            {
                success.Value = LCP.SolveMLCP(M, q, l, u, ref indexSetArray, ref w, ref z);
            }
        }

        bool AreEqualTolerance(float a, float b, float eps = math.EPSILON)
        {
            var t = a - b;
            return math.abs(t) <= eps;
        }

        public enum IndexSetMode
        {
            Free,
            Tight,
            Mixed
        }

        [TestCase(IndexSetMode.Free)]
        [TestCase(IndexSetMode.Tight)]
        [TestCase(IndexSetMode.Mixed)]
        public void LCP_Job_10x10(IndexSetMode initialIndexSetMode)
        {
            /*
                Solve the following LCP:

                    w = q + Mz, w >= 0, z >=0, w^t * z = 0

                The LCP is constructed in a way such that
                    for q components that correspond to -1, the corresponding LCP variable is free (i.e., z > 0 && w = 0 ), and
                    for q components that correspond to 1, the corresponding LCP variable is tight (i.e., z = 0 && w > 0).
                In addition, for LCP variables which we are forcing to be free (LCP.IndexFlag.ForceFree),
                    we set the corresponding q component to 2 to induce a z result of -2 at this component.

                The above behavior is achieved by setting M to identity.
            */

            using (var heap = MemoryManager.Create(16384, Allocator.Temp))
            {
                var M = Matrix.Create(heap, 10, 10);
                M.Clear();
                for (int i = 0; i < 10; ++i)
                {
                    M.Cols[i][i] = 1.0f;
                }

                // create rhs vector q
                var q = Vector.Create(heap, 10);

                // free vs. tight flags
                // also, force some as free
                var expectedIndexSetArray = new[] {LCP.LCPIndexFlag.Free, LCP.LCPIndexFlag.Tight, LCP.LCPIndexFlag.ForcedFree, LCP.LCPIndexFlag.Tight, LCP.LCPIndexFlag.Free, LCP.LCPIndexFlag.ForcedFree, LCP.LCPIndexFlag.Tight, LCP.LCPIndexFlag.Free, LCP.LCPIndexFlag.Free, LCP.LCPIndexFlag.Tight};

                // initialize q to induce the expected outcome of free and tight variables
                for (int i = 0; i < 10; ++i)
                {
                    // set q to -1 to induce free variable and 1 to induce tight variable
                    // Note: here we also set q to 2 when we have a forced free variable. This way, in the result vector we should
                    // get a negative z value (-2) which would otherwise not be possible for free variables in an LCP.
                    switch (expectedIndexSetArray[i])
                    {
                        case LCP.LCPIndexFlag.Free:
                        {
                            q[i] = -1.0f;
                            break;
                        }
                        case  LCP.LCPIndexFlag.Tight:
                        {
                            q[i] = 1.0f;
                            break;
                        }
                        case  LCP.LCPIndexFlag.ForcedFree:
                        {
                            q[i] = 2.0f;
                            break;
                        }
                    }
                }

                var indexSetArray = new NativeArray<LCP.LCPIndexFlag>(10, Allocator.TempJob);
                if (initialIndexSetMode == IndexSetMode.Free || initialIndexSetMode == IndexSetMode.Tight)
                {
                    var allFree = initialIndexSetMode == IndexSetMode.Free;
                    // create initial tight & free set array, with 1 denoting a tight variable and 0 denoting a free variable
                    for (int i = 0; i < 10; ++i)
                    {
                        var flag = expectedIndexSetArray[i] == LCP.LCPIndexFlag.ForcedFree ? LCP.LCPIndexFlag.ForcedFree : (allFree ? LCP.LCPIndexFlag.Free : LCP.LCPIndexFlag.Tight);
                        indexSetArray[i] = flag;
                    }
                }
                else // mixed
                {
                    for (int i = 0; i < 10; ++i)
                    {
                        var indexFree = i % 2 == 0;
                        var flag = expectedIndexSetArray[i] == LCP.LCPIndexFlag.ForcedFree ? LCP.LCPIndexFlag.ForcedFree : (indexFree ? LCP.LCPIndexFlag.Free : LCP.LCPIndexFlag.Tight);
                        indexSetArray[i] = flag;
                    }
                }

                // create slack and result vectors
                var w = Vector.Create(heap, 10);
                w.Clear();
                var z = Vector.Create(heap, 10);
                z.Clear();

                var success = new NativeReference<bool>(false, Allocator.TempJob);

                try
                {
                    var lcpJob = new LCPSolveJob()
                    {
                        heap = heap,
                        M = M,
                        q = q,
                        z = z,
                        w = w,
                        indexSetArray = indexSetArray,
                        success = success
                    };
                    lcpJob.Run();

                    Assert.IsTrue(lcpJob.success.Value);

                    // check results:

                    // first check for any infeasibilities (w and z values which violate the LCP)

                    for (int i = 0; i < 10; ++i)
                    {
                        var zVal = z[i];
                        var wVal = w[i];
                        // both z and w must be >= 0, unless the index is forced to be free.
                        // In the latter case, we expect w to be 0 and z could take any value.
                        if (indexSetArray[i] != LCP.LCPIndexFlag.ForcedFree)
                        {
                            Assert.GreaterOrEqual(zVal, -k_LCPEpsilon);
                            Assert.GreaterOrEqual(wVal, -k_LCPEpsilon);

                            if (AreEqualTolerance(zVal, wVal, k_LCPEpsilon))
                            {
                                // if both are equal, they must be both zero
                                Assert.IsTrue(AreEqualTolerance(zVal, 0, k_LCPEpsilon));
                                Assert.IsTrue(AreEqualTolerance(wVal, 0, k_LCPEpsilon));
                            }
                            else
                            {
                                // if they are not equal, then exactly one of the two variables must be zero:
                                var zZero = AreEqualTolerance(zVal, 0, k_LCPEpsilon);
                                var wZero = AreEqualTolerance(wVal, 0, k_LCPEpsilon);

                                Assert.IsTrue(zZero ^ wZero);

                                if (zZero)
                                {
                                    Assert.Greater(wVal, 0);
                                }

                                if (wZero)
                                {
                                    Assert.Greater(zVal, 0);
                                }
                            }
                        }
                    }

                    for (int i = 0; i < 10; ++i)
                    {
                        // expected state for i'th variable after solving LCP
                        switch (expectedIndexSetArray[i])
                        {
                            case LCP.LCPIndexFlag.Free:
                            {
                                // for free variables we expect zero slack (w)
                                Assert.IsTrue(AreEqualTolerance(w[i], 0, k_LCPEpsilon));
                                break;
                            }
                            case LCP.LCPIndexFlag.Tight:
                            {
                                // for tight variables we expect zero in the result vector (z)
                                Assert.IsTrue(AreEqualTolerance(z[i], 0, k_LCPEpsilon));
                                break;
                            }
                            case LCP.LCPIndexFlag.ForcedFree:
                            {
                                // for variables that are forced free, we also expect zero slack as for any other free variable.
                                Assert.IsTrue(AreEqualTolerance(w[i], 0, k_LCPEpsilon));

                                // also, the way this toy example is constructed, we expect z to be -2 for forced free variables (see explanation above).
                                Assert.IsTrue(AreEqualTolerance(z[i], -2.0f, k_LCPEpsilon));

                                break;
                            }
                        }

                        // check if we get the expected tight set end result
                        Assert.AreEqual(expectedIndexSetArray[i], indexSetArray[i]);
                    }
                }
                finally
                {
                    M.Dispose();
                    q.Dispose();
                    z.Dispose();
                    w.Dispose();
                    indexSetArray.Dispose();
                    success.Dispose();
                }
            }
        }

        [Flags] public enum VariableModeFlags
        {
            None = 0x00,
            Free = 0x01,
            LowerTight = 0x02,
            UpperTight = 0x04,
            Forced = 0x08,
            GreaterZero = 0x10,
        }

        [TestCase(IndexSetMode.Free)]
        [TestCase(IndexSetMode.Tight)]
        [TestCase(IndexSetMode.Mixed)]
        public void MLCP_Job_10x10(IndexSetMode initialIndexSetMode)
        {
            /*
                Solve the following MLCP:

                    w = w_+ - w_- = q + Mz,
                    l <= z <= u,
                    w_+ >= 0, w_- >= 0,
                    w_+^t * (z - l) = 0,
                    w_-^t * (u - z) = 0,
                    w_+^t * w_- = 0

                We construct an MLCP with an expected result vector z as follows.
                The lead matrix M is set to identity and the upper and lower bounds are set to u = 2 and l = -2 respectively.
                This leads to the following behavior:
                    - for q components that are set to -1, the corresponding MLCP variable is free (in bounds)
                      and will be greater than zero, i.e., -2=l < 0 < z=1 < u=2 && w_- = 0.
                    - for q components that are set to +1, the corresponding MLCP variable is free (in bounds)
                      and will be smaller than zero, i.e., -2=l < -1=z < 0 < u=2 && w_+ = 0.
                    - for q components that are set to -3, the corresponding MLCP variable is upper tight (at upper
                      bounds), i.e., z=u=2 && w_- > 0.
                    - for q components that are set to +3, the corresponding MLCP variable is lower tight (at lower
                      bounds), i.e., -2=l=z && w_+ > 0.

                Accordingly, all q vector components can be specified in such a way as to enforce specific desired results in the z vector.

                In addition, we are forcing a set of MLCP variables (z components) to be unbounded by setting their corresponding
                l and u components to -infinity and +infinity. For these variables we also set the q component to +3 or -3 to
                enforce a z value of -3 or +3 respectively.
            */

            using (var heap = MemoryManager.Create(16384, Allocator.Temp))
            {
                var M = Matrix.Create(heap, 10, 10);
                M.Clear();
                for (int i = 0; i < 10; ++i)
                {
                    M.Cols[i][i] = 1.0f;
                }

                // create rhs vector q
                var q = Vector.Create(heap, 10);

                // create lower and uppers bounds vectors l and u
                var l = Vector.Create(heap, 10);
                l.Clear(0, -1, -2.0f);
                var u = Vector.Create(heap, 10);
                u.Clear(0, -1, 2.0f);

                // free vs. tight flags
                // also, force some as free
                var expectedResultArray = new[]
                {
                    VariableModeFlags.Free | VariableModeFlags.GreaterZero | VariableModeFlags.Forced, VariableModeFlags.LowerTight,
                    VariableModeFlags.UpperTight, VariableModeFlags.Free | VariableModeFlags.GreaterZero,
                    VariableModeFlags.Free | VariableModeFlags.GreaterZero | VariableModeFlags.Forced, VariableModeFlags.UpperTight,
                    VariableModeFlags.LowerTight, VariableModeFlags.Free | VariableModeFlags.Forced,
                    VariableModeFlags.Free, VariableModeFlags.LowerTight
                };

                // initialize q to induce the expected outcome of free and tight variables
                for (int i = 0; i < 10; ++i)
                {
                    // set q to -1 to induce free variable and 1 to induce tight variable
                    // Note: here we also set q to 2 when we have a forced free variable. This way, in the result vector we should
                    // get a negative z value (-2) which would otherwise not be possible for free variables in an LCP.
                    var flags = expectedResultArray[i];
                    if (flags.HasFlag(VariableModeFlags.Free))
                    {
                        bool forced = flags.HasFlag(VariableModeFlags.Forced);
                        if (forced)
                        {
                            // override bounds: set bounds to infinity to force free unbounded variable
                            l[i] = float.MinValue;
                            u[i] = float.MaxValue;
                        }

                        if (flags.HasFlag(VariableModeFlags.GreaterZero))
                        {
                            q[i] = math.select(-1.0f, -3.0f, forced);
                        }
                        else
                        {
                            q[i] = math.select(1.0f, 3.0f, forced);
                        }
                    }
                    else if (flags.HasFlag(VariableModeFlags.UpperTight))
                    {
                        q[i] = -3.0f;
                    }
                    else if (flags.HasFlag(VariableModeFlags.LowerTight))
                    {
                        q[i] = 3.0f;
                    }
                }

                var indexSetArray = new NativeArray<LCP.MLCPIndexFlag>(10, Allocator.TempJob);
                if (initialIndexSetMode == IndexSetMode.Free || initialIndexSetMode == IndexSetMode.Tight)
                {
                    var allFree = initialIndexSetMode == IndexSetMode.Free;
                    // create initial tight or free index set array, with alternating lower and upper tight if initially tight.
                    for (int i = 0; i < 10; ++i)
                    {
                        var flag = allFree ? LCP.MLCPIndexFlag.Free : (i % 2 == 0 ? LCP.MLCPIndexFlag.LowerTight : LCP.MLCPIndexFlag.UpperTight);
                        indexSetArray[i] = flag;
                    }
                }
                else // mixed
                {
                    for (int i = 0; i < 10; ++i)
                    {
                        // alternate free and tight: free for even variable indices, tight for odd, and set every 3rd variable as lower tight if not already free
                        var indexFree = i % 2 == 0;
                        var flag = indexFree ? LCP.MLCPIndexFlag.Free : (i % 3 == 0 ? LCP.MLCPIndexFlag.LowerTight : LCP.MLCPIndexFlag.UpperTight);
                        indexSetArray[i] = flag;
                    }
                }

                // create slack and result vectors
                var w = Vector.Create(heap, 10);
                w.Clear();
                var z = Vector.Create(heap, 10);
                z.Clear();

                var success = new NativeReference<bool>(false, Allocator.TempJob);
                try
                {
                    var lcpJob = new MLCPSolveJob()
                    {
                        heap = heap,
                        M = M,
                        q = q,
                        l = l,
                        u = u,
                        z = z,
                        w = w,
                        indexSetArray = indexSetArray,
                        success = success
                    };
                    lcpJob.Run();

                    Assert.IsTrue(lcpJob.success.Value);

                    // check results:

                    // first check for any infeasibilities (w and z values which violate the LCP)

                    for (int i = 0; i < 10; ++i)
                    {
                        var zVal = z[i];
                        var wVal = w[i];

                        if (indexSetArray[i] == LCP.MLCPIndexFlag.LowerTight)
                        {
                            // need z on the lower bound (z == l)
                            Assert.IsTrue(AreEqualTolerance(zVal, l[i], k_LCPEpsilon));
                            // need positive slack (w >= 0)
                            Assert.GreaterOrEqual(wVal, -k_LCPEpsilon);
                        }
                        else if (indexSetArray[i] == LCP.MLCPIndexFlag.UpperTight)
                        {
                            // need z on the upper bound (z == u)
                            Assert.IsTrue(AreEqualTolerance(zVal, u[i], k_LCPEpsilon));
                            // need negative slack (w <= 0)
                            Assert.LessOrEqual(wVal, k_LCPEpsilon);
                        }
                        else // LCP.MLCPIndexFlag.Free
                        {
                            // check if l <= z <= u
                            if (l[i] > float.MinValue)
                            {
                                Assert.GreaterOrEqual(zVal, l[i] - k_LCPEpsilon);
                            }

                            if (u[i] < float.MaxValue)
                            {
                                Assert.LessOrEqual(zVal, u[i] + k_LCPEpsilon);
                            }

                            // check that zero slack
                            Assert.IsTrue(AreEqualTolerance(wVal, 0, k_LCPEpsilon));
                        }
                    }

                    for (int i = 0; i < 10; ++i)
                    {
                        // expected state for i'th variable after solving MLCP
                        var flags = expectedResultArray[i];
                        if (flags.HasFlag(VariableModeFlags.Free))
                        {
                            var sign = math.select(-1.0f, 1.0f, flags.HasFlag(VariableModeFlags.GreaterZero));
                            var expectedValue = sign * math.select(1, 3, flags.HasFlag(VariableModeFlags.Forced));
                            Assert.IsTrue(AreEqualTolerance(z[i], expectedValue, k_LCPEpsilon));

                            // expect the correct index set value
                            Assert.AreEqual(LCP.MLCPIndexFlag.Free, indexSetArray[i]);
                        }
                        else if (flags.HasFlag(VariableModeFlags.LowerTight))
                        {
                            var expectedValue = -2.0f;
                            Assert.IsTrue(AreEqualTolerance(z[i], expectedValue));

                            // expect the correct index set value
                            Assert.AreEqual(LCP.MLCPIndexFlag.LowerTight, indexSetArray[i]);
                        }
                        else if (flags.HasFlag(VariableModeFlags.UpperTight))
                        {
                            var expectedValue = 2.0f;
                            Assert.IsTrue(AreEqualTolerance(z[i], expectedValue));

                            // expect the correct index set value
                            Assert.AreEqual(LCP.MLCPIndexFlag.UpperTight, indexSetArray[i]);
                        }
                    }
                }
                finally
                {
                    M.Dispose();
                    q.Dispose();
                    l.Dispose();
                    u.Dispose();
                    z.Dispose();
                    w.Dispose();
                    indexSetArray.Dispose();
                    success.Dispose();
                }
            }
        }
    }
}
