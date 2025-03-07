using System.Threading;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

namespace Unity.Numerics.Linear.Dense.Primitives
{
    internal unsafe struct LCP
    {
        // Index flags for Linear Complementarity Problems (LCP)
        public enum LCPIndexFlag : byte
        {
            Free = 0,
            Tight = 1,
            ForcedFree = 2
        }

        // Index flags for Mixed Linear Complementarity Problems (MLCP)
        public enum MLCPIndexFlag : byte
        {
            Free = 0,
            LowerTight = 1,
            UpperTight = 2
        }

        /// <summary>
        /// Solve the strictly monotone Linear Complementarity Problem (LCP) with
        ///
        ///     w = q + Mz,
        ///     w >= 0,
        ///     z >=0,
        ///     w^t * z = 0,
        ///
        /// where M is a P matrix. That is, all the principal minors of M are positive.
        /// The method is based on
        ///      Judice, Joaquim and Fernanda M. Pires. “A block principal pivoting algorithm for large-scale
        ///      strictly monotone linear complementarity problems.” Comput. Oper. Res. 21 (1994): 587-596.
        ///
        /// </summary>
        /// <param name="indexSetArray">Array indicating the initial guess for tight and free sets, as well as which variables
        /// are forced to be in the free set as part of the solution to the LCP.
        ///     Free: variable with corresponding index is in the free set
        ///     Tight: variable with corresponding index is in the tight set
        ///     ForcedFree: variable with corresponding index is forced to be in the free set as part of the solution
        /// Forcing a variable to be in the free set allows this variable to have a negative solution value (z_i < 0), while the remaining
        /// LCP is still respected.
        /// </param>
        /// <returns>true if solution was found within the given maximum of iterations. Otherwise false.</returns>
        [BurstCompile]
        [GenerateTestsForBurstCompatibility]
        public static bool SolveLCP(in Matrix M, in Vector q, ref NativeArray<LCPIndexFlag> indexSetArray,
            [WriteOnly] ref Vector w, [WriteOnly] ref Vector z, in float eps = 1e-4f, in uint maxNumIterations = 100,
            in uint maxNumBlockIterWithoutInfeasibilityReduction = 3)
        {
            if (M.NumCols != M.NumRows)
            {
                UnityEngine.Debug.LogError("Input matrix must be square.");
                return false;
            }

            int dim = M.MinDimension;

            if (q.Dimension != dim)
            {
                UnityEngine.Debug.LogError("Invalid input vector size.");
                return false;
            }

            if (w.Dimension != dim || z.Dimension != dim)
            {
                UnityEngine.Debug.LogError("Invalid output vector size.");
                return false;
            }

            if (indexSetArray.Length < dim)
            {
                UnityEngine.Debug.LogError("Insufficient space in the provided index set array.");
                return false;
            }

            // Allocate enough data for submatrix and subvector creations below. Will be reused in each iteration through
            // use of Matrix.Submatrix().
            Matrix Q_Max = Matrix.Create(M.Heap, dim, 1);
            Matrix M_Max = Matrix.Create(M.Heap, dim, dim);
            Vector w_Max = Vector.Create(M.Heap, dim);
            bool done = false;

            try
            {
                // assume initially we have a maximal number of infeasibilities
                int minNumInfeasibilities = dim;
                var T = new NativeArray<int>(dim, Allocator.Temp);
                var F = new NativeArray<int>(dim, Allocator.Temp);
                var pivots = new NativeArray<int>(dim, Allocator.Temp);
                Vector z_F = new Vector();
                Vector w_T = new Vector();

                bool blockMode = true;
                int blockIter = 0;
                uint iter = 0;
                while (!done && iter++ < maxNumIterations)
                {
                    // 1. Create new tight and free index sets
                    int numTightIndices = 0;
                    int numFreeIndices = 0;
                    for (int i = 0; i < dim; ++i)
                    {
                        bool tight = indexSetArray[i] == LCPIndexFlag.Tight;
                        if (tight)
                        {
                            T[numTightIndices++] = i;
                        }
                        else // free (either via IndexFlag.Free or forced via IndexFlag.ForcedFree)
                        {
                            F[numFreeIndices++] = i;
                        }
                    }

                    ////
                    // 1. Build M_FF from free set unless it is empty.

                    if (numFreeIndices > 0)
                    {
                        var M_FF = M_Max.Submatrix(0, 0, numFreeIndices, numFreeIndices);
                        if (numFreeIndices == dim)
                        {
                            // optimized path for M_FF == M
                            M.CopyTo(M_FF);
                        }
                        else
                        {
                            M.CreateSubmatrix(M_FF, F, numFreeIndices, F, numFreeIndices);
                        }

                        ////
                        // 2. Solve the linear system M_FF * z_F = -q_F for z_F
                        // under the assumption that the current tight and free index sets F and T are accurate.

                        // Compute LU factorization of M_FF with M_FF = LU.
                        LU.Factor(M_FF, ref pivots, out var singularRow);
                        if (singularRow != -1)
                        {
                            UnityEngine.Debug.LogError(
                                "Principal submatrix M_FF is singular and could not be factorized. Aborting.");
                            return false;
                        }

                        // quickly obtain a matrix with the right dimension for the solve (data is shared through use of Submatrix function)
                        Matrix Q_F = Q_Max.Submatrix(0, 0, numFreeIndices);
                        q.CreateSubvector(Q_F.Cols[0], F, numFreeIndices);
                        // Q_F.Cols[0] corresponds to q_F at this point

                        // @todo: add a linear system solver to Numerics which uses LU factorization, for use here and in Solver.cs

                        // With M_FF = LU, solve linear system LU * z_F = -q_F via forward and backward substitution as follows:
                        //      1. Define y := U * z_F
                        //      2. Solve L * y = -q_F for y
                        //      3. Then, solve U * z_F = y for z_F

                        // Note: we need to use the pivot array from the factorization above to permute the input rhs
                        // vector in order to solve the correct system.
                        var pivot = false;
                        for (int i = 0; i < numFreeIndices; ++i)
                        {
                            pivot |= pivots[i] != i;
                        }

                        if (pivot)
                        {
                            Q_F.InterchangeRows(pivots, 0, numFreeIndices - 1);
                        }

                        // Solve L * y = -q_F for y
                        // Note: Q_F.Col[0] = q_F, so we need alpha set to -1.0f below.
                        var alpha = -1.0f;
                        Q_F.SolveGeneralizedTriangular(Side.Left, TriangularType.Lower, Op.None, DiagonalType.Unit,
                            alpha, M_FF);
                        // Note: at this point, Q_F contains y

                        // Solve U * z_F = y for z_F
                        Q_F.SolveGeneralizedTriangular(Side.Left, TriangularType.Upper, Op.None, DiagonalType.Explicit,
                            1.0f, M_FF);
                        // quickly extract result from matrix Q_F where Q_F.Cols[0] = z_F (data is shared through use of Subvector function)
                        z_F = Q_F.Cols[0].Subvector(0);

                        M_FF.Dispose();
                        Q_F.Dispose();
                    }
                    // else: z_F has dimension 0

                    ////
                    // 3. Build M_TF and calculate slack vector for tight indices as w_T = q_T + M_TF * z_F
                    if (numTightIndices > 0)
                    {
                        w_T = w_Max.Subvector(0, numTightIndices);
                        // Note: w_T is initialized to q_T
                        q.CreateSubvector(w_T, T, numTightIndices);

                        if (numFreeIndices > 0)
                        {
                            var M_TF = M_Max.Submatrix(0, 0, numTightIndices, numFreeIndices);
                            M.CreateSubmatrix(M_TF, T, numTightIndices, F, numFreeIndices);
                            // add M_TF * z_F to q_T to complete calculation of w_T
                            // Note: rhs.Cols[0] corresponds to z_F at this point
                            w_T.ScaleAndAddProduct(M_TF, Op.None, z_F, 1.0f, 1.0f);
                            M_TF.Dispose();
                        }
                        // else: M_TF is zero matrix. So w_T = q_T and we are done.
                    }
                    // else: w_T has dimension 0

                    ////
                    // 4. Check for infeasibilities and apply pivoting strategy:
                    // If # of infeasibilities has reduced below the past minimum # of infeasibilities thus far encountered, stay in block pivoting and
                    // update all index sets with the infeasibilities. Otherwise, continue block pivoting maximum p times while attempting
                    // to further reduce the # of infeasibilities beyond the minimum. If this does not succeed, switch to single pivoting, updating only the first
                    // encountered infeasibility, until the minimum has reduced. Once this is the case, switch back to block pivoting and reset the block
                    // pivoting counter.

                    // count infeasibilities
                    int freeToTight = 0;
                    int tightToFree = 0;

                    for (int i = 0; i < numFreeIndices; ++i)
                    {
                        freeToTight += math.select(0, 1,
                            indexSetArray[F[i]] != LCPIndexFlag.ForcedFree && z_F[i] < -eps);
                    }

                    for (int i = 0; i < numTightIndices; ++i)
                    {
                        tightToFree += math.select(0, 1, w_T[i] < -eps);
                    }

                    // check if number of infeasibilities has reduced
                    int numInf = freeToTight + tightToFree;
                    if (numInf == 0)
                    {
                        // we are done. Copy over results.

                        // Approach: first reset result vectors to zero, and then copy z_F and w_T into result vectors
                        // according to final tight and free index sets.
                        z.Clear();
                        w.Clear();

                        for (int i = 0; i < numFreeIndices; ++i)
                        {
                            z[F[i]] = z_F[i];
                        }

                        for (int i = 0; i < numTightIndices; ++i)
                        {
                            w[T[i]] = w_T[i];
                        }

                        done = true;
                    }
                    else
                    {
                        if (minNumInfeasibilities > numInf)
                        {
                            // if this is the case store the new min infeasibility count
                            minNumInfeasibilities = numInf;
                            // go into block mode (or ensure we stay in it)
                            blockMode = true;
                            // reset block iteration count
                            blockIter = 0;
                        }
                        else if (blockMode)
                        {
                            // increase block iter in this case, if we are in block mode
                            ++blockIter;
                        }

                        // check if we need to switch to single pivoting,
                        // which is the case when we have done "p" block pivoting iterations (see paper) in
                        // which the number of infeasibilities hasn't reduced.
                        if (blockMode && blockIter > maxNumBlockIterWithoutInfeasibilityReduction)
                        {
                            blockMode = false;
                        }

                        if (!blockMode)
                        {
                            // find first infeasibility among free or tight index sets and update index sets
                            // @todo here we should pick the one with the smallest index, but for now we just take the first one that
                            // we encounter
                            bool found = false;
                            if (freeToTight > 0)
                            {
                                for (int i = 0; i < numFreeIndices; ++i)
                                {
                                    if (z_F[i] < -eps && indexSetArray[F[i]] != LCPIndexFlag.ForcedFree)
                                    {
                                        // remove from free set and put in tight set
                                        indexSetArray[F[i]] = LCPIndexFlag.Tight;
                                        found = true;
                                        break;
                                    }
                                }
                            }

                            if (tightToFree > 0 && !found)
                            {
                                for (int i = 0; i < numTightIndices; ++i)
                                {
                                    if (w_T[i] < -eps)
                                    {
                                        // remove from tight set and put into free set
                                        indexSetArray[T[i]] = LCPIndexFlag.Free;
                                        break;
                                    }
                                }
                            }
                        }
                        else
                        {
                            // find all infeasibilities and update index sets
                            if (freeToTight > 0)
                            {
                                for (int i = 0; i < numFreeIndices; ++i)
                                {
                                    var index = F[i];
                                    if (z_F[i] < -eps && indexSetArray[F[i]] != LCPIndexFlag.ForcedFree)
                                    {
                                        // remove from free set and put in tight set
                                        indexSetArray[F[i]] = LCPIndexFlag.Tight;
                                    }
                                }
                            }

                            if (tightToFree > 0)
                            {
                                for (int i = 0; i < numTightIndices; ++i)
                                {
                                    if (w_T[i] < -eps)
                                    {
                                        // remove from tight set and put into free set
                                        indexSetArray[T[i]] = LCPIndexFlag.Free;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            finally
            {
                Q_Max.Dispose();
                M_Max.Dispose();
                w_Max.Dispose();
            }

            return done;
        }

        /// <summary>
        /// Solve the strictly monotone Mixed Linear Complementarity Problem (MLCP) with
        ///
        ///     w = w_+ - w_- = q + Mz,
        ///     w_+ >= 0, w_- >= 0,
        ///     z - l >= 0, u - z >= 0,
        ///     w_+^t * (z - l) = 0,
        ///     w_-^t * (u - z) = 0,
        ///     w_+^t * w_- = 0
        ///
        /// where and l and u are the lower and upper bounds of the admissible solution space and M is a P matrix. That is, all the
        /// principal minors of M are positive.
        /// The method is based on
        ///      Judice, Joaquim and Fernanda M. Pires. “A block principal pivoting algorithm for large-scale
        ///      strictly monotone linear complementarity problems.” Comput. Oper. Res. 21 (1994): 587-596.
        ///
        /// </summary>
        /// <param name="indexSetArray">Array indicating the initial guess for lower tight, upper tight and free sets, as well as which variables
        /// are forced to be in the free set as part of the solution to the LCP.
        ///     Free: variable with corresponding index is in the free set
        ///     Lower Tight: variable with corresponding index is in the lower tight set
        ///     Upper Tight: variable with corresponding index is in the upper tight set
        ///     ForcedFree: variable with corresponding index is forced to be in the free set as part of the solution
        /// Forcing a variable to be in the free set is equivalent to setting this variable's lower and upper bounds to -infinity and +infinity respectively (-infinity < z_i < infinity), while the remaining
        /// LCP is still respected.
        /// </param>
        /// <returns>true if solution was found within the given maximum of iterations. Otherwise false.</returns>
        [BurstCompile]
        [GenerateTestsForBurstCompatibility]
        public static bool SolveMLCP(in Matrix M, in Vector q, in Vector l, in Vector u,
            ref NativeArray<MLCPIndexFlag> indexSetArray,
            [WriteOnly] ref Vector w, [WriteOnly] ref Vector z, in float eps = 1e-4f, in uint maxNumIterations = 100,
            in uint maxNumBlockIterWithoutInfeasibilityReduction = 3)
        {
            if (M.NumCols != M.NumRows)
            {
                UnityEngine.Debug.LogError("Input matrix must be square.");
                return false;
            }

            int dim = M.MinDimension;

            if (q.Dimension != dim)
            {
                UnityEngine.Debug.LogError("Invalid input vector size.");
                return false;
            }

            if (l.Dimension != dim)
            {
                UnityEngine.Debug.LogError("Invalid input vector size.");
                return false;
            }

            if (u.Dimension != dim)
            {
                UnityEngine.Debug.LogError("Invalid input vector size.");
                return false;
            }

            if (w.Dimension != dim || z.Dimension != dim)
            {
                UnityEngine.Debug.LogError("Invalid output vector size.");
                return false;
            }

            if (indexSetArray.Length < dim)
            {
                UnityEngine.Debug.LogError("Insufficient space in the provided index set array.");
                return false;
            }

            // Allocate enough data for submatrix and subvector creations below. Will be reused in each iteration through
            // use of Matrix.Submatrix().
            Matrix RHS_Max = Matrix.Create(M.Heap, dim, 1);
            // @todo: it is better to allocate a matrix of the right size on the heap whenever required and to dispose of it once done
            // this way the accesses are going to be way more cache-friendly. Would be useful to maybe allow resizing matrices?
            Matrix M_Max = Matrix.Create(M.Heap, dim, dim);
            Vector lu_Max = Vector.Create(M.Heap, dim);
            Vector w_Max = Vector.Create(M.Heap, dim);

            bool done = false;
            uint iter = 0;
            try
            {
                // assume initially we have a maximal number of infeasibilities
                int minNumInfeasibilities = dim;
                var L = new NativeArray<int>(dim, Allocator.Temp);
                var U = new NativeArray<int>(dim, Allocator.Temp);
                var F = new NativeArray<int>(dim, Allocator.Temp);
                var pivots = new NativeArray<int>(dim, Allocator.Temp);
                Vector z_F = new Vector();
                Vector l_L = new Vector();
                Vector u_U = new Vector();
                Vector w_L = new Vector();
                Vector w_U = new Vector();

                // Below, we rearrange the system of equations M * z + q = w based on the free, lower tight and
                // upper tight index sets F, L and U as follows:
                //
                //      |M_FF|M_FL|M_FU|   |z_F|   |q_F|   |w_F|
                //      |M_LF|M_LL|M_LU| * |z_L| + |q_L| = |w_L|
                //      |M_UF|M_UL|M_UU|   |z_U|   |q_U|   |w_U|
                //

                bool blockMode = true;
                int blockIter = 0;
                while (!done && iter++ < maxNumIterations)
                {
                    // 1. Create new tight and free index sets
                    int numLowerTightIndices = 0;
                    int numUpperTightIndices = 0;
                    int numFreeIndices = 0;
                    for (int i = 0; i < dim; ++i)
                    {
                        switch (indexSetArray[i])
                        {
                            case MLCPIndexFlag.Free:
                            {
                                F[numFreeIndices++] = i;
                                break;
                            }
                            case MLCPIndexFlag.LowerTight:
                            {
                                L[numLowerTightIndices++] = i;
                                break;
                            }
                            case MLCPIndexFlag.UpperTight:
                            {
                                U[numUpperTightIndices++] = i;
                                break;
                            }
                        }
                    }

                    ////
                    // 2. Solve for free variables z_F if there are any (F not empty)

                    // start by creating lower and upper limit vectors for subsequent calculations:
                    if (numLowerTightIndices > 0)
                    {
                        // Note: l_L will be reused below in step 3 during slack calculation and will occupy the first portion of lu_Max
                        // @todo: could share vector space with z_F. Combined size of z_F, l_L and u_U is dim. Note that z_F is stored inside B matrix.
                        l_L = lu_Max.Subvector(0, numLowerTightIndices);
                        l.CreateSubvector(l_L, L, numLowerTightIndices);
                    }

                    if (numUpperTightIndices > 0)
                    {
                        // Note: u_U will be reused below in step 3 during slack calculation and will occupy the remaining portion of lu_Max
                        u_U = lu_Max.Subvector(numLowerTightIndices, numUpperTightIndices);
                        u.CreateSubvector(u_U, U, numUpperTightIndices);
                    }

                    if (numFreeIndices > 0)
                    {
                        // Build M_FF from free set
                        var M_FF = M_Max.Submatrix(0, 0, numFreeIndices, numFreeIndices);
                        if (numFreeIndices == dim)
                        {
                            // optimized path for M_FF == M
                            M.CopyTo(M_FF);
                        }
                        else
                        {
                            M.CreateSubmatrix(M_FF, F, numFreeIndices, F, numFreeIndices);
                        }

                        ////
                        // Solve the linear system M_FF * z_F = -b for z_F with -b = -M_FL * l_L - M_FU * u_U - q_F,
                        // under the assumption that the current lower tight, upper tight and free index sets (U, T and F) are accurate.

                        // Compute LU factorization of M_FF with M_FF = LU.
                        LU.Factor(M_FF, ref pivots, out var singularRow);
                        if (singularRow != -1)
                        {
                            UnityEngine.Debug.LogError(
                                "Principal submatrix M_FF is singular and could not be factorized. Aborting.");
                            return false;
                        }

                        ////
                        // 2a. Form right hand side -b = -M_FL * l_L - M_FU * u_U - q_F = -(M_FL * l_L + M_FU * u_U + q_F)
                        // Note: quickly obtain a matrix with the right dimension for the rhs vector of the solve (data is shared through use of Submatrix function)
                        Matrix B = RHS_Max.Submatrix(0, 0, numFreeIndices);
                        q.CreateSubvector(B.Cols[0], F, numFreeIndices);
                        // B.Cols[0] corresponds to q_F at this point

                        ////
                        // 2b. Create M_FL and M_FU matrices and incorporate the contribution of the lower and upper tight variables to the right hand side
                        if (numLowerTightIndices > 0)
                        {
                            // add M_FL * l_L to b
                            Matrix M_FL = M_Max.Submatrix(0, numFreeIndices, numFreeIndices, numLowerTightIndices);
                            M.CreateSubmatrix(M_FL, F, numFreeIndices, L, numLowerTightIndices);

                            // add M_FL * l_L to b = q_F yielding b = M_FL * l_L + q_F
                            B.Cols[0].ScaleAndAddProduct(M_FL, Op.None, l_L, 1.0f, 1.0f);
                            M_FL.Dispose();
                        }

                        if (numUpperTightIndices > 0)
                        {
                            // add M_FU * l_U to b
                            Matrix M_FU = M_Max.Submatrix(0, numFreeIndices + numLowerTightIndices, numFreeIndices,
                                numUpperTightIndices);
                            M.CreateSubmatrix(M_FU, F, numFreeIndices, U, numUpperTightIndices);

                            // add M_FU * u_U to b = M_FL * l_L + q_F yielding b = M_FU * u_U + M_FL * l_L + q_F
                            B.Cols[0].ScaleAndAddProduct(M_FU, Op.None, u_U, 1.0f, 1.0f);
                            M_FU.Dispose();
                        }

                        // @todo: add a linear system solver to Numerics which uses LU factorization, for use here and in Solver.cs

                        ////
                        // 2.b With M_FF = LU, solve linear system LU * z_F = -b via forward and backward substitution as follows:
                        //      1. Define y := U * z_F
                        //      2. Solve L * y = -b for y
                        //      3. Then, solve U * z_F = y for z_F

                        // Note: we need to use the pivot array from the factorization above to permute the input rhs
                        // vector in order to solve the correct system.
                        var pivot = false;
                        for (int i = 0; i < numFreeIndices; ++i)
                        {
                            pivot |= pivots[i] != i;
                        }

                        if (pivot)
                        {
                            B.InterchangeRows(pivots, 0, numFreeIndices - 1);
                        }

                        // Solve L * y = -b for y
                        // Note: At this point we have B.Col[0] = b, so we need alpha set to -1 below to set the right hand side
                        // in the linear system solve correctly to -b.
                        var alpha = -1.0f;
                        B.SolveGeneralizedTriangular(Side.Left, TriangularType.Lower, Op.None, DiagonalType.Unit,
                            alpha, M_FF);
                        // Note: at this point, B contains y

                        // Solve U * z_F = y for z_F
                        B.SolveGeneralizedTriangular(Side.Left, TriangularType.Upper, Op.None, DiagonalType.Explicit,
                            1.0f, M_FF);
                        // quickly extract result from matrix B where B.Cols[0] = z_F (data is shared through use of Subvector function)
                        z_F = B.Cols[0].Subvector(0);
                        M_FF.Dispose();
                        B.Dispose();
                    }
                    // else: z_F has dimension 0

                    ////
                    // 3. Calculate slack variables, w_L and w_U, for lower and upper tight variables, respectively.

                    ////
                    // 3a. Calculate slack for lower tight variables, w_L:
                    //     Build M_LF, M_LL and M_LU and calculate slack vector for lower tight indices as
                    //          w_L = q_L + M_LF * z_F + M_LL * l_L + M_LU * u_U
                    if (numLowerTightIndices > 0)
                    {
                        w_L = w_Max.Subvector(0, numLowerTightIndices);
                        // Initialize slack vector w_L to w_L = q_L
                        q.CreateSubvector(w_L, L, numLowerTightIndices);

                        // w_L += M_LF * z_F
                        if (numFreeIndices > 0)
                        {
                            var M_LF = M_Max.Submatrix(numFreeIndices, 0, numLowerTightIndices, numFreeIndices);
                            M.CreateSubmatrix(M_LF, L, numLowerTightIndices, F, numFreeIndices);
                            w_L.ScaleAndAddProduct(M_LF, Op.None, z_F, 1.0f, 1.0f);
                            M_LF.Dispose();
                        }

                        // w_L += M_LL * l_L
                        var M_LL = M_Max.Submatrix(numFreeIndices, numFreeIndices, numLowerTightIndices,
                            numLowerTightIndices);
                        M.CreateSubmatrix(M_LL, L, numLowerTightIndices, L, numLowerTightIndices);
                        w_L.ScaleAndAddProduct(M_LL, Op.None, l_L, 1.0f, 1.0f);
                        M_LL.Dispose();

                        // w_L += M_LU * l_U
                        if (numUpperTightIndices > 0)
                        {
                            var M_LU = M_Max.Submatrix(numFreeIndices, numFreeIndices + numLowerTightIndices,
                                numLowerTightIndices, numUpperTightIndices);
                            M.CreateSubmatrix(M_LU, L, numLowerTightIndices, U, numUpperTightIndices);
                            w_L.ScaleAndAddProduct(M_LU, Op.None, u_U, 1.0f, 1.0f);
                            M_LU.Dispose();
                        }
                    }
                    // else: w_L has dimension 0

                    ////
                    // 3b. Calculate slack for upper tight variables, w_U:
                    //     Build M_UF, M_UL and M_UU and calculate slack vector for lower tight indices as
                    //          w_U = q_U + M_UF * z_F + M_UL * l_L + M_UU * u_U
                    if (numUpperTightIndices > 0)
                    {
                        w_U = w_Max.Subvector(numLowerTightIndices, numUpperTightIndices);
                        // Initialize slack vector w_U to w_U = q_U
                        q.CreateSubvector(w_U, U, numUpperTightIndices);

                        // w_U += M_UF * z_F
                        if (numFreeIndices > 0)
                        {
                            var M_UF = M_Max.Submatrix(numFreeIndices + numLowerTightIndices, 0, numUpperTightIndices,
                                numFreeIndices);
                            M.CreateSubmatrix(M_UF, U, numUpperTightIndices, F, numFreeIndices);
                            w_U.ScaleAndAddProduct(M_UF, Op.None, z_F, 1.0f, 1.0f);
                            M_UF.Dispose();
                        }

                        // w_U += M_UL * l_L
                        if (numLowerTightIndices > 0)
                        {
                            var M_UL = M_Max.Submatrix(numFreeIndices + numLowerTightIndices, numFreeIndices,
                                numUpperTightIndices, numLowerTightIndices);
                            M.CreateSubmatrix(M_UL, U, numUpperTightIndices, L, numLowerTightIndices);
                            w_U.ScaleAndAddProduct(M_UL, Op.None, l_L, 1.0f, 1.0f);
                            M_UL.Dispose();
                        }

                        // w_U += M_UU * u_U
                        var M_UU = M_Max.Submatrix(numFreeIndices + numLowerTightIndices,
                            numFreeIndices + numLowerTightIndices, numUpperTightIndices, numUpperTightIndices);
                        M.CreateSubmatrix(M_UU, U, numUpperTightIndices, U, numUpperTightIndices);
                        w_U.ScaleAndAddProduct(M_UU, Op.None, u_U, 1.0f, 1.0f);
                        M_UU.Dispose();
                    }
                    // else: w_U has dimension 0

                    ////
                    // 4. Check for infeasibilities and apply pivoting strategy:
                    // If # of infeasibilities has reduced below the past minimum # of infeasibilities thus far encountered, stay in block pivoting and
                    // update all index sets with the infeasibilities. Otherwise, continue block pivoting maximum p times while attempting
                    // to further reduce the # of infeasibilities beyond the minimum. If this does not succeed, switch to single pivoting, updating only the first
                    // encountered infeasibility, until the minimum has reduced. Once this is the case, switch back to block pivoting and reset the block
                    // pivoting counter.

                    // count infeasibilities
                    int freeToTight = 0;
                    int lowerTightToFree = 0;
                    int upperTightToFree = 0;

                    // @todo avoid accessing data and checking LCP condition twice (here for counting infeasibilities and again below)
                    // could write result as MLCPIndexFlag into a local array tmpIndexSetArray while counting the infeasibilities (same sorting as indexSetArray), and then when
                    // deciding whether we choose single or block pivoting below, copy either only the first infeasibility from tmpIndexSetArray into indexSetArray for
                    // next iteration, or memcopy the entire array over.
                    for (int i = 0; i < numFreeIndices; ++i)
                    {
                        var index = F[i];
                        var lower = l[index];
                        var upper = u[index];
                        freeToTight += math.select(0, 1, z_F[i] < lower - eps || z_F[i] > upper + eps);
                    }

                    for (int i = 0; i < numLowerTightIndices; ++i)
                    {
                        // For lower tight slack we expect w_L = w_+ >= 0.
                        // Accordingly, this condition is violated if w_L < 0.

                        lowerTightToFree += math.select(0, 1, w_L[i] < -eps);
                    }

                    for (int i = 0; i < numUpperTightIndices; ++i)
                    {
                        // For upper tight slack we expect -w_U = w_- >= 0.
                        // Accordingly, this condition is violated if -w_U < 0, i.e., w_U > 0.
                        upperTightToFree += math.select(0, 1, w_U[i] > eps);
                    }

                    // check if number of infeasibilities has reduced
                    int numInf = freeToTight + lowerTightToFree + upperTightToFree;
                    if (numInf == 0)
                    {
                        // we are done. Copy over results.

                        // Approach:
                        // - slack vector w: reset slack vector to zero, and overwrite the slack values for lower and upper tight variables with
                        //   the results in w_L and w_U respectively to correctly report the slack for the tight variables.
                        // - resultant vector z: copy z_F into z for free variables and set the result for lower and upper tight variables to the limits
                        //   defined by l and u respectively.
                        w.Clear();

                        for (int i = 0; i < numFreeIndices; ++i)
                        {
                            z[F[i]] = z_F[i];
                            // note: slack for free variables is zero, which is achieved by clearing w above
                        }

                        for (int i = 0; i < numLowerTightIndices; ++i)
                        {
                            w[L[i]] = w_L[i];
                            z[L[i]] = l_L[i];
                        }

                        for (int i = 0; i < numUpperTightIndices; ++i)
                        {
                            w[U[i]] = w_U[i];
                            z[U[i]] = u_U[i];
                        }

                        done = true;
                    }
                    else
                    {
                        if (minNumInfeasibilities > numInf)
                        {
                            // if this is the case store the new min infeasibility count
                            minNumInfeasibilities = numInf;
                            // go into block mode (or ensure we stay in it)
                            blockMode = true;
                            // reset block iteration count
                            blockIter = 0;
                        }
                        else if (blockMode)
                        {
                            // increase block iter in this case, if we are in block mode
                            ++blockIter;
                        }

                        // check if we need to switch to single pivoting,
                        // which is the case when we have done "p" block pivoting iterations (see paper) in
                        // which the number of infeasibilities hasn't reduced.
                        if (blockMode && blockIter > maxNumBlockIterWithoutInfeasibilityReduction)
                        {
                            blockMode = false;
                        }

                        if (!blockMode)
                        {
                            // find first infeasibility among free or tight index sets and update index sets
                            // @todo here we should pick the one with the smallest index among all indices in F, L and U,
                            // but for now we just take the first one that we encounter
                            bool found = false;
                            if (freeToTight > 0)
                            {
                                for (int i = 0; i < numFreeIndices; ++i)
                                {
                                    var index = F[i];
                                    var lower = l[index];
                                    var upper = u[index];

                                    if (z_F[i] < lower - eps)
                                    {
                                        // remove from free set and put in lower tight set
                                        indexSetArray[F[i]] = MLCPIndexFlag.LowerTight;
                                        found = true;
                                        break;
                                    }
                                    else if (z_F[i] > upper + eps)
                                    {
                                        // remove from free set and put in upper tight set
                                        indexSetArray[F[i]] = MLCPIndexFlag.UpperTight;
                                        found = true;
                                        break;
                                    }
                                }
                            }

                            if (!found && lowerTightToFree > 0)
                            {
                                for (int i = 0; i < numLowerTightIndices; ++i)
                                {
                                    if (w_L[i] < -eps)
                                    {
                                        // remove from lower tight set and put into free set
                                        indexSetArray[L[i]] = MLCPIndexFlag.Free;
                                        found = true;
                                        break;
                                    }
                                }
                            }

                            if (!found && upperTightToFree > 0)
                            {
                                for (int i = 0; i < numUpperTightIndices; ++i)
                                {
                                    if (w_U[i] > eps)
                                    {
                                        // remove from upper tight set and put into free set
                                        indexSetArray[U[i]] = MLCPIndexFlag.Free;
                                        break;
                                    }
                                }
                            }
                        }
                        else
                        {
                            // find all infeasibilities and update index sets
                            if (freeToTight > 0)
                            {
                                for (int i = 0; i < numFreeIndices; ++i)
                                {
                                    var index = F[i];
                                    var lower = l[index];
                                    var upper = u[index];

                                    if (z_F[i] < lower - eps)
                                    {
                                        // remove from free set and put in lower tight set
                                        indexSetArray[F[i]] = MLCPIndexFlag.LowerTight;
                                    }
                                    else if (z_F[i] > upper + eps)
                                    {
                                        // remove from free set and put in upper tight set
                                        indexSetArray[F[i]] = MLCPIndexFlag.UpperTight;
                                    }
                                }
                            }

                            if (lowerTightToFree > 0)
                            {
                                for (int i = 0; i < numLowerTightIndices; ++i)
                                {
                                    if (w_L[i] < -eps)
                                    {
                                        // remove from lower tight set and put into free set
                                        indexSetArray[L[i]] = MLCPIndexFlag.Free;
                                    }
                                }
                            }

                            if (upperTightToFree > 0)
                            {
                                for (int i = 0; i < numUpperTightIndices; ++i)
                                {
                                    if (w_U[i] > eps)
                                    {
                                        // remove from upper tight set and put into free set
                                        indexSetArray[U[i]] = MLCPIndexFlag.Free;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            finally
            {
                RHS_Max.Dispose();
                M_Max.Dispose();
                lu_Max.Dispose();
                w_Max.Dispose();
            }

            if (!done && iter >= maxNumIterations)
            {
                // @todo return best result if we ran out of attempts
                UnityEngine.Debug.LogError("Maximum number of iterations exceeded.");
            }

            return done;
        }

        /// <summary>
        /// Coupling data used in <see cref="SolveCoupledMLCP"/>.
        /// Specifies coupling between the lower and upper bounds of a given variable and a consecutive set of other LCP variables.
        /// </summary>
        public struct CouplingData
        {
            /// <summary>
            /// Index of the first variable in the set of consecutive LCP variables whose bounds are coupled.
            /// </summary>
            public int coupledVariableStartIndex;

            /// <summary>
            /// Number of variables whose bounds are coupled.
            /// </summary>
            public short coupledVariableCount;

            /// <summary>
            /// Index of the first LCP variable in the set of consecutive LCP variables used to update the bounds of the coupled variables.
            /// </summary>
            public int variableStartIndex;

            /// <summary>
            /// Number of LCP variables used to update the bounds of the coupled variables.
            /// </summary>
            public short variableCount;

            /// <summary>
            /// Linear factor applied to the value of the LCP variables for calculating the new lower and upper bounds of the entire set of coupled variables.
            /// The lower and upper bounds l, u of the coupled variable are calculated from the consecutive set of n LCP variables f_i as follows:
            ///     l = -factor * /sum_{i=0}^{n-1} f_i
            ///     u =  factor * /sum_{i=0}^{n-1} f_i
            /// </summary>
            public float factor;
        }

        /// <summary>
        /// Solve the strictly monotone Mixed Linear Complementarity Problem (MLCP) with
        ///
        ///     w = w_+ - w_- = q + Mz,
        ///     w_+ >= 0, w_- >= 0,
        ///     z - l >= 0, u - z >= 0,
        ///     w_+^t * (z - l) = 0,
        ///     w_-^t * (u - z) = 0,
        ///     w_+^t * w_- = 0
        ///
        /// where and l and u are the lower and upper bounds of the admissible solution space and M is a P matrix. That is, all the
        /// principal minors of M are positive.
        ///
        /// In addition, a coupling between the bounds of a user-specified set of LCP variables with other sets of LCP variables is performed.
        /// The lower and upper bounds l, u of the coupled variables are calculated from the respective consecutive set of n LCP variables f_i as follows:
        ///     l = -\alpha * /sum_{i=0}^{n-1} f_i
        ///     u =  \alpha * /sum_{i=0}^{n-1} f_i
        /// where \alpha is a user-specified factor.
        /// This method can be used among others to model Coulomb friction.
        ///
        /// The MLCP is solved using a block principal pivoting algorithm. For the situation in which no exact solution to the MLCP can be found,
        /// the method keeps track of the best solution found so far and returns it if the maximum number of iterations is exceeded.
        ///
        /// While the coupling method is a novel approach, the general method is based on the following paper:
        ///      Judice, Joaquim and Fernanda M. Pires. “A block principal pivoting algorithm for large-scale
        ///      strictly monotone linear complementarity problems.” Comput. Oper. Res. 21 (1994): 587-596.
        ///
        /// </summary>
        /// <param name="indexSetArray">Array indicating the initial guess for lower tight, upper tight and free sets, as well as which variables
        /// are forced to be in the free set as part of the solution to the LCP.
        ///     Free: variable with corresponding index is in the free set
        ///     Lower Tight: variable with corresponding index is in the lower tight set
        ///     Upper Tight: variable with corresponding index is in the upper tight set
        ///     ForcedFree: variable with corresponding index is forced to be in the free set as part of the solution
        /// Forcing a variable to be in the free set is equivalent to setting this variable's lower and upper bounds to -infinity and +infinity respectively (-infinity < z_i < infinity), while the remaining
        /// LCP is still respected.
        /// </param>
        /// <param name="couplingDataArray">Array specifying the coupling information between variables in this MLCP.</param>
        /// <param name="maxCouplingIterations">Maximum number of coupling iterations.</param>
        /// <param name="minNumCouplingIterations">Minimum number of coupling iterations.</param>
        /// <param name="minNumRefinementIterations">Minimum number of refinement iterations required before stopping the pivoting process during
        ///                                          which no coupling iterations are performed anymore. Ensures that bounds are no longer updated when we are getting close
        ///                                          to the maximum iteration count. This improves the quality of the result in cases a solution can not be found within the first
        ///                                          <see cref="maxNumIterations"/> - <see cref="minNumRefinementIterations"/> iterations. </param>
        /// <returns>Finite LCP error if a solution was found. Otherwise, float.PositiveInfinity.</returns>
        [BurstCompile]
        [GenerateTestsForBurstCompatibility]
        public static float SolveCoupledMLCP(in Matrix M, in Vector q, ref Vector l, ref Vector u,
            ref NativeArray<MLCPIndexFlag> indexSetArray, ref NativeList<CouplingData> couplingDataArray,
            [WriteOnly] ref Vector w, [WriteOnly] ref Vector z, in float eps = 1e-4f,
            in uint maxNumIterations = 100,
            in uint maxNumBlockIterWithoutInfeasibilityReduction = 3,
            in uint maxCouplingIterations = 10, in uint minNumCouplingIterations = 2, in uint minNumRefinementIterations = 10)
        {
            if (M.NumCols != M.NumRows)
            {
                UnityEngine.Debug.LogError("Input matrix must be square.");
                return float.PositiveInfinity;
            }

            int dim = M.MinDimension;

            if (q.Dimension != dim)
            {
                UnityEngine.Debug.LogError("Invalid input vector size.");
                return float.PositiveInfinity;
            }

            if (l.Dimension != dim)
            {
                UnityEngine.Debug.LogError("Invalid input vector size.");
                return float.PositiveInfinity;
            }

            if (u.Dimension != dim)
            {
                UnityEngine.Debug.LogError("Invalid input vector size.");
                return float.PositiveInfinity;
            }

            if (w.Dimension != dim || z.Dimension != dim)
            {
                UnityEngine.Debug.LogError("Invalid output vector size.");
                return float.PositiveInfinity;
            }

            if (indexSetArray.Length < dim)
            {
                UnityEngine.Debug.LogError("Insufficient space in the provided index set array.");
                return float.PositiveInfinity;
            }

            // Allocate enough data for submatrix and subvector creations below. Will be reused in each iteration through
            // use of Matrix.Submatrix().
            Matrix RHS_Max = Matrix.Create(M.Heap, dim, 1);
            // @todo: it is better to allocate a matrix of the right size on the heap whenever required and to dispose of it once done
            // this way the accesses are going to be way more cache-friendly. Would be useful to maybe allow resizing matrices?
            Matrix M_Max = Matrix.Create(M.Heap, dim, dim);
            Vector lu_Max = Vector.Create(M.Heap, dim);
            Vector w_Max = Vector.Create(M.Heap, dim);

            // cache of best result
            Vector z_best = Vector.Create(M.Heap, dim);
            Vector w_best = Vector.Create(M.Heap, dim);
            float minError = float.PositiveInfinity;

            bool done = false;
            uint iter = 0;
            uint couplingIter = 0;
            try
            {
                // assume initially we have a maximal number of infeasibilities
                int minNumInfeasibilities = dim;
                var L = new NativeArray<int>(dim, Allocator.Temp);
                var U = new NativeArray<int>(dim, Allocator.Temp);
                var F = new NativeArray<int>(dim, Allocator.Temp);
                var pivots = new NativeArray<int>(dim, Allocator.Temp);
                Vector z_F = new Vector();
                Vector l_L = new Vector();
                Vector u_U = new Vector();
                Vector w_L = new Vector();
                Vector w_U = new Vector();

                // Below, we rearrange the system of equations M * z + q = w based on the free, lower tight and
                // upper tight index sets F, L and U as follows:
                //
                //      |M_FF|M_FL|M_FU|   |z_F|   |q_F|   |w_F|
                //      |M_LF|M_LL|M_LU| * |z_L| + |q_L| = |w_L|
                //      |M_UF|M_UL|M_UU|   |z_U|   |q_U|   |w_U|
                //

                bool blockMode = true;
                int blockIter = 0;
                while (!done && iter++ < maxNumIterations)
                {
                    // 1. Create new tight and free index sets
                    int numLowerTightIndices = 0;
                    int numUpperTightIndices = 0;
                    int numFreeIndices = 0;
                    for (int i = 0; i < dim; ++i)
                    {
                        switch (indexSetArray[i])
                        {
                            case MLCPIndexFlag.Free:
                            {
                                F[numFreeIndices++] = i;
                                break;
                            }
                            case MLCPIndexFlag.LowerTight:
                            {
                                L[numLowerTightIndices++] = i;
                                break;
                            }
                            case MLCPIndexFlag.UpperTight:
                            {
                                U[numUpperTightIndices++] = i;
                                break;
                            }
                        }
                    }

                    ////
                    // 2. Solve for free variables z_F if there are any (F not empty)

                    // start by creating lower and upper limit vectors for subsequent calculations:
                    if (numLowerTightIndices > 0)
                    {
                        // Note: l_L will be reused below in step 3 during slack calculation and will occupy the first portion of lu_Max
                        // @todo: could share vector space with z_F. Combined size of z_F, l_L and u_U is dim. Note that z_F is stored inside B matrix.
                        l_L = lu_Max.Subvector(0, numLowerTightIndices);
                        l.CreateSubvector(l_L, L, numLowerTightIndices);
                    }

                    if (numUpperTightIndices > 0)
                    {
                        // Note: u_U will be reused below in step 3 during slack calculation and will occupy the remaining portion of lu_Max
                        u_U = lu_Max.Subvector(numLowerTightIndices, numUpperTightIndices);
                        u.CreateSubvector(u_U, U, numUpperTightIndices);
                    }

                    if (numFreeIndices > 0)
                    {
                        // Build M_FF from free set
                        var M_FF = M_Max.Submatrix(0, 0, numFreeIndices, numFreeIndices);
                        if (numFreeIndices == dim)
                        {
                            // optimized path for M_FF == M
                            M.CopyTo(M_FF);
                        }
                        else
                        {
                            M.CreateSubmatrix(M_FF, F, numFreeIndices, F, numFreeIndices);
                        }

                        ////
                        // Solve the linear system M_FF * z_F = -b for z_F with -b = -M_FL * l_L - M_FU * u_U - q_F,
                        // under the assumption that the current lower tight, upper tight and free index sets (U, T and F) are accurate.

                        // Compute LU factorization of M_FF with M_FF = LU.
                        LU.Factor(M_FF, ref pivots, out var singularRow);
                        if (singularRow != -1)
                        {
                            UnityEngine.Debug.LogError(
                                "Principal submatrix M_FF is singular and could not be factorized. Aborting.");
                            return float.PositiveInfinity;
                        }

                        ////
                        // 2a. Form right hand side -b = -M_FL * l_L - M_FU * u_U - q_F = -(M_FL * l_L + M_FU * u_U + q_F)
                        // Note: quickly obtain a matrix with the right dimension for the rhs vector of the solve (data is shared through use of Submatrix function)
                        Matrix B = RHS_Max.Submatrix(0, 0, numFreeIndices);
                        q.CreateSubvector(B.Cols[0], F, numFreeIndices);
                        // B.Cols[0] corresponds to q_F at this point

                        ////
                        // 2b. Create M_FL and M_FU matrices and incorporate the contribution of the lower and upper tight variables to the right hand side
                        if (numLowerTightIndices > 0)
                        {
                            // add M_FL * l_L to b
                            Matrix M_FL = M_Max.Submatrix(0, numFreeIndices, numFreeIndices, numLowerTightIndices);
                            M.CreateSubmatrix(M_FL, F, numFreeIndices, L, numLowerTightIndices);

                            // add M_FL * l_L to b = q_F yielding b = M_FL * l_L + q_F
                            B.Cols[0].ScaleAndAddProduct(M_FL, Op.None, l_L, 1.0f, 1.0f);
                            M_FL.Dispose();
                        }

                        if (numUpperTightIndices > 0)
                        {
                            // add M_FU * l_U to b
                            Matrix M_FU = M_Max.Submatrix(0, numFreeIndices + numLowerTightIndices, numFreeIndices,
                                numUpperTightIndices);
                            M.CreateSubmatrix(M_FU, F, numFreeIndices, U, numUpperTightIndices);

                            // add M_FU * u_U to b = M_FL * l_L + q_F yielding b = M_FU * u_U + M_FL * l_L + q_F
                            B.Cols[0].ScaleAndAddProduct(M_FU, Op.None, u_U, 1.0f, 1.0f);
                            M_FU.Dispose();
                        }

                        // @todo: add a linear system solver to Numerics which uses LU factorization, for use here and in Solver.cs

                        ////
                        // 2.b With M_FF = LU, solve linear system LU * z_F = -b via forward and backward substitution as follows:
                        //      1. Define y := U * z_F
                        //      2. Solve L * y = -b for y
                        //      3. Then, solve U * z_F = y for z_F

                        // Note: we need to use the pivot array from the factorization above to permute the input rhs
                        // vector in order to solve the correct system.
                        var pivot = false;
                        for (int i = 0; i < numFreeIndices; ++i)
                        {
                            pivot |= pivots[i] != i;
                        }

                        if (pivot)
                        {
                            B.InterchangeRows(pivots, 0, numFreeIndices - 1);
                        }

                        // Solve L * y = -b for y
                        // Note: At this point we have B.Col[0] = b, so we need alpha set to -1 below to set the right hand side
                        // in the linear system solve correctly to -b.
                        var alpha = -1.0f;
                        B.SolveGeneralizedTriangular(Side.Left, TriangularType.Lower, Op.None, DiagonalType.Unit,
                            alpha, M_FF);
                        // Note: at this point, B contains y

                        // Solve U * z_F = y for z_F
                        B.SolveGeneralizedTriangular(Side.Left, TriangularType.Upper, Op.None, DiagonalType.Explicit,
                            1.0f, M_FF);
                        // quickly extract result from matrix B where B.Cols[0] = z_F (data is shared through use of Subvector function)
                        z_F = B.Cols[0].Subvector(0);
                        M_FF.Dispose();
                        B.Dispose();
                    }
                    // else: z_F has dimension 0

                    ////
                    // 3. Calculate slack variables, w_L and w_U, for lower and upper tight variables, respectively.

                    ////
                    // 3a. Calculate slack for lower tight variables, w_L:
                    //     Build M_LF, M_LL and M_LU and calculate slack vector for lower tight indices as
                    //          w_L = q_L + M_LF * z_F + M_LL * l_L + M_LU * u_U
                    if (numLowerTightIndices > 0)
                    {
                        w_L = w_Max.Subvector(0, numLowerTightIndices);
                        // Initialize slack vector w_L to w_L = q_L
                        q.CreateSubvector(w_L, L, numLowerTightIndices);

                        // w_L += M_LF * z_F
                        if (numFreeIndices > 0)
                        {
                            var M_LF = M_Max.Submatrix(numFreeIndices, 0, numLowerTightIndices, numFreeIndices);
                            M.CreateSubmatrix(M_LF, L, numLowerTightIndices, F, numFreeIndices);
                            w_L.ScaleAndAddProduct(M_LF, Op.None, z_F, 1.0f, 1.0f);
                            M_LF.Dispose();
                        }

                        // w_L += M_LL * l_L
                        var M_LL = M_Max.Submatrix(numFreeIndices, numFreeIndices, numLowerTightIndices,
                            numLowerTightIndices);
                        M.CreateSubmatrix(M_LL, L, numLowerTightIndices, L, numLowerTightIndices);
                        w_L.ScaleAndAddProduct(M_LL, Op.None, l_L, 1.0f, 1.0f);
                        M_LL.Dispose();

                        // w_L += M_LU * l_U
                        if (numUpperTightIndices > 0)
                        {
                            var M_LU = M_Max.Submatrix(numFreeIndices, numFreeIndices + numLowerTightIndices,
                                numLowerTightIndices, numUpperTightIndices);
                            M.CreateSubmatrix(M_LU, L, numLowerTightIndices, U, numUpperTightIndices);
                            w_L.ScaleAndAddProduct(M_LU, Op.None, u_U, 1.0f, 1.0f);
                            M_LU.Dispose();
                        }
                    }
                    // else: w_L has dimension 0

                    ////
                    // 3b. Calculate slack for upper tight variables, w_U:
                    //     Build M_UF, M_UL and M_UU and calculate slack vector for lower tight indices as
                    //          w_U = q_U + M_UF * z_F + M_UL * l_L + M_UU * u_U
                    if (numUpperTightIndices > 0)
                    {
                        w_U = w_Max.Subvector(numLowerTightIndices, numUpperTightIndices);
                        // Initialize slack vector w_U to w_U = q_U
                        q.CreateSubvector(in w_U, U, numUpperTightIndices);

                        // w_U += M_UF * z_F
                        if (numFreeIndices > 0)
                        {
                            var M_UF = M_Max.Submatrix(numFreeIndices + numLowerTightIndices, 0, numUpperTightIndices,
                                numFreeIndices);
                            M.CreateSubmatrix(in M_UF, U, numUpperTightIndices, F, numFreeIndices);
                            w_U.ScaleAndAddProduct(M_UF, Op.None, z_F, 1.0f, 1.0f);
                            M_UF.Dispose();
                        }

                        // w_U += M_UL * l_L
                        if (numLowerTightIndices > 0)
                        {
                            var M_UL = M_Max.Submatrix(numFreeIndices + numLowerTightIndices, numFreeIndices,
                                numUpperTightIndices, numLowerTightIndices);
                            M.CreateSubmatrix(in M_UL, U, numUpperTightIndices, L, numLowerTightIndices);
                            w_U.ScaleAndAddProduct(M_UL, Op.None, l_L, 1.0f, 1.0f);
                            M_UL.Dispose();
                        }

                        // w_U += M_UU * u_U
                        var M_UU = M_Max.Submatrix(numFreeIndices + numLowerTightIndices,
                            numFreeIndices + numLowerTightIndices, numUpperTightIndices, numUpperTightIndices);
                        M.CreateSubmatrix(in M_UU, U, numUpperTightIndices, U, numUpperTightIndices);
                        w_U.ScaleAndAddProduct(M_UU, Op.None, u_U, 1.0f, 1.0f);
                        M_UU.Dispose();
                    }
                    // else: w_U has dimension 0

                    ////
                    // 4. Check for infeasibilities and apply pivoting strategy:
                    // If # of infeasibilities has reduced below the past minimum # of infeasibilities thus far encountered, stay in block pivoting and
                    // update all index sets with the infeasibilities. Otherwise, continue block pivoting maximum p times while attempting
                    // to further reduce the # of infeasibilities beyond the minimum. If this does not succeed, switch to single pivoting, updating only the first
                    // encountered infeasibility, until the minimum has reduced. Once this is the case, switch back to block pivoting and reset the block
                    // pivoting counter.

                    // count infeasibilities
                    int freeToTight = 0;
                    int lowerTightToFree = 0;
                    int upperTightToFree = 0;

                    // sum of the componentwise lcp errors in the current solution
                    float lcpError = 0;

                    // @todo avoid accessing data and checking LCP condition twice (here for counting infeasibilities and again below)
                    // could write result as MLCPIndexFlag into a local array tmpIndexSetArray while counting the infeasibilities (same sorting as indexSetArray), and then when
                    // deciding whether we choose single or block pivoting below, copy either only the first infeasibility from tmpIndexSetArray into indexSetArray for
                    // next iteration, or memcopy the entire array over.
                    for (int i = 0; i < numFreeIndices; ++i)
                    {
                        var index = F[i];
                        var lower = l[index] - eps;
                        var upper = u[index] + eps;

                        var lowError = lower - z_F[i];
                        var upError = z_F[i] - upper;
                        var error = math.max(lowError, upError);
                        bool hasError = error > 0;
                        lcpError += math.select(0, error, hasError);

                        freeToTight += math.select(0, 1, hasError);
                    }

                    for (int i = 0; i < numLowerTightIndices; ++i)
                    {
                        // For lower tight slack we expect w_L = w_+ >= 0.
                        // Accordingly, this condition is violated if w_L < 0.

                        var error = -eps - w_L[i];
                        bool hasError = error > 0;
                        lcpError += math.select(0, error, hasError);

                        lowerTightToFree += math.select(0, 1, hasError);
                    }

                    for (int i = 0; i < numUpperTightIndices; ++i)
                    {
                        // For upper tight slack we expect -w_U = w_- >= 0.
                        // Accordingly, this condition is violated if -w_U < 0, i.e., w_U > 0.

                        var error = w_U[i] - eps;
                        bool hasError = error > 0;
                        lcpError += math.select(0, error, hasError);

                        upperTightToFree += math.select(0, 1, w_U[i] > eps);
                    }

                    // Update result vectors

                    // Approach:
                    // - resultant vector z: copy z_F into z for free variables and set the result for lower and upper tight variables to the limits
                    //   defined by l and u respectively.

                    for (int i = 0; i < numFreeIndices; ++i)
                    {
                        z[F[i]] = z_F[i];
                        // note: slack for free variables is zero, which is achieved by clearing w above
                    }

                    for (int i = 0; i < numLowerTightIndices; ++i)
                    {
                        z[L[i]] = l_L[i];
                    }

                    for (int i = 0; i < numUpperTightIndices; ++i)
                    {
                        z[U[i]] = u_U[i];
                    }

                    bool improvedSolution = false;

                    // check if number of infeasibilities has reduced
                    int numInf = freeToTight + lowerTightToFree + upperTightToFree;
                    if (numInf == 0 &&  // no more infeasibilities
                        couplingIter + 1 > minNumCouplingIterations) // sufficient number of coupling iterations
                    {
                        // No more infeasibilities. We found a solution.

                        // just update the slack variables. The resultant vector z was already updated above.

                        // Approach:

                        w.Clear();
                        for (int i = 0; i < numLowerTightIndices; ++i)
                        {
                            w[L[i]] = w_L[i];
                        }

                        for (int i = 0; i < numUpperTightIndices; ++i)
                        {
                            w[U[i]] = w_U[i];
                        }

                        minError = lcpError;

                        done = true;
                    }
                    else if (iter < maxNumIterations) // check if we reached the maximum iteration count
                    {
                        // Prepare next iteration:

                        // 1. update best result
                        if (lcpError < minError)
                        {
                            minError = lcpError;
                            z.CopyTo(z_best);

                            // also need to cache slack
                            w_best.Clear();
                            for (int i = 0; i < numLowerTightIndices; ++i)
                            {
                                w_best[L[i]] = w_L[i];
                            }

                            for (int i = 0; i < numUpperTightIndices; ++i)
                            {
                                w_best[U[i]] = w_U[i];
                            }

                            improvedSolution = true;
                        }

                        // 2. check if we need to change pivoting mode:

                        if (minNumInfeasibilities > numInf)
                        {
                            // if this is the case store the new min infeasibility count
                            minNumInfeasibilities = numInf;
                            // go into block mode (or ensure we stay in it)
                            blockMode = true;
                            // reset block iteration count
                            blockIter = 0;
                        }
                        else if (blockMode)
                        {
                            // increase block iter in this case, if we are in block mode
                            ++blockIter;
                        }

                        // check if we need to switch to single pivoting,
                        // which is the case when we have done "p" block pivoting iterations (see paper) in
                        // which the number of infeasibilities hasn't reduced.
                        if (blockMode && blockIter > maxNumBlockIterWithoutInfeasibilityReduction)
                        {
                            blockMode = false;
                        }

                        // 3. pivot:

                        // single pivot mode (pivot only the first infeasibility we find)
                        if (!blockMode)
                        {
                            // find first infeasibility among free or tight index sets and update index sets
                            // @todo here we should pick the one with the smallest index among all indices in F, L and U,
                            // but for now we just take the first one that we encounter
                            bool found = false;
                            if (freeToTight > 0)
                            {
                                for (int i = 0; i < numFreeIndices; ++i)
                                {
                                    var index = F[i];
                                    var lower = l[index];
                                    var upper = u[index];

                                    if (z_F[i] < lower - eps)
                                    {
                                        // remove from free set and put in lower tight set
                                        indexSetArray[F[i]] = MLCPIndexFlag.LowerTight;
                                        found = true;
                                        break;
                                    }
                                    else if (z_F[i] > upper + eps)
                                    {
                                        // remove from free set and put in upper tight set
                                        indexSetArray[F[i]] = MLCPIndexFlag.UpperTight;
                                        found = true;
                                        break;
                                    }
                                }
                            }

                            if (!found && lowerTightToFree > 0)
                            {
                                for (int i = 0; i < numLowerTightIndices; ++i)
                                {
                                    if (w_L[i] < -eps)
                                    {
                                        // remove from lower tight set and put into free set
                                        indexSetArray[L[i]] = MLCPIndexFlag.Free;
                                        found = true;
                                        break;
                                    }
                                }
                            }

                            if (!found && upperTightToFree > 0)
                            {
                                for (int i = 0; i < numUpperTightIndices; ++i)
                                {
                                    if (w_U[i] > eps)
                                    {
                                        // remove from upper tight set and put into free set
                                        indexSetArray[U[i]] = MLCPIndexFlag.Free;
                                        break;
                                    }
                                }
                            }
                        }
                        // block pivot mode (pivot all infeasibilities)
                        else
                        {
                            // find all infeasibilities and update index sets
                            if (freeToTight > 0)
                            {
                                for (int i = 0; i < numFreeIndices; ++i)
                                {
                                    var index = F[i];
                                    var lower = l[index];
                                    var upper = u[index];

                                    if (z_F[i] < lower - eps)
                                    {
                                        // remove from free set and put in lower tight set
                                        indexSetArray[F[i]] = MLCPIndexFlag.LowerTight;
                                    }
                                    else if (z_F[i] > upper + eps)
                                    {
                                        // remove from free set and put in upper tight set
                                        indexSetArray[F[i]] = MLCPIndexFlag.UpperTight;
                                    }
                                }
                            }

                            if (lowerTightToFree > 0)
                            {
                                for (int i = 0; i < numLowerTightIndices; ++i)
                                {
                                    if (w_L[i] < -eps)
                                    {
                                        // remove from lower tight set and put into free set
                                        indexSetArray[L[i]] = MLCPIndexFlag.Free;
                                    }
                                }
                            }

                            if (upperTightToFree > 0)
                            {
                                for (int i = 0; i < numUpperTightIndices; ++i)
                                {
                                    if (w_U[i] > eps)
                                    {
                                        // remove from upper tight set and put into free set
                                        indexSetArray[U[i]] = MLCPIndexFlag.Free;
                                    }
                                }
                            }
                        }

                        // update the coupled bounds and force the variables to be in the free set so that we can solve for
                        // these variables again within their updated bounds
                        if ((minError < eps || improvedSolution) && couplingIter < maxCouplingIterations && maxNumIterations - iter > minNumRefinementIterations)
                        {
                            ++couplingIter;

                            // update coupled lower and upper bounds symmetrically
                            for (int i = 0; i < couplingDataArray.Length; ++i)
                            {
                                var data = couplingDataArray[i];
                                var startIndex = data.variableStartIndex;
                                var bounds = 0f;

                                // calculate sum of z-component values
                                for (int j = 0; j < data.variableCount; ++j)
                                {
                                    int refIndex = startIndex + j;
                                    var z_val = z[refIndex];
                                    var l_val = l[refIndex];
                                    var u_val = u[refIndex];
                                    // cap z values to prevent us from bringing negative values from infeasibilities into the equation
                                    // todo: could also ignore the infeasibilities
                                    var z_capped = math.max(math.min(z_val, u_val), l_val);
                                    bounds += z_capped;
                                }

                                // calculate new bounds scaled sum of z-component values
                                // Note: in the contact friction case, using the sum of the z-component values and not the average is important since the
                                // sum of the z-values corresponds to the total normal force at the contact surface which we need to use to calculate the Coulomb
                                // friction force by multiplying it with the provided factor which in this case represents the friction coefficient.
                                bounds *= data.factor;

                                var newLower = -bounds;
                                var newUpper = bounds;

                                for (int j = 0; j < data.coupledVariableCount; ++j)
                                {
                                    var coupledVariableIndex = data.coupledVariableStartIndex + j;

                                    var oldLower = l[coupledVariableIndex];
                                    var oldUpper = u[coupledVariableIndex];

                                    if (math.abs(oldLower - newLower) > eps || math.abs(oldUpper - newUpper) > eps)
                                    {
                                        // current index flag
                                        var indexFlag = indexSetArray[coupledVariableIndex];

                                        if ((indexFlag == MLCPIndexFlag.LowerTight && newLower < oldLower) ||
                                            indexFlag == MLCPIndexFlag.UpperTight && newUpper > oldUpper)
                                        {
                                            indexSetArray[coupledVariableIndex] = MLCPIndexFlag.Free;
                                        }

                                        l[coupledVariableIndex] = newLower;
                                        u[coupledVariableIndex] = newUpper;

                                        // reset LCP error since we have a new LCP problem
                                        minError = float.PositiveInfinity;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            finally
            {
                RHS_Max.Dispose();
                M_Max.Dispose();
                lu_Max.Dispose();
                w_Max.Dispose();
                z_best.Dispose();
                w_best.Dispose();
            }

            if (!done && iter >= maxNumIterations)
            {
                // if we ran out of iterations but couldn't find a solution to the lcp, we return the best solution we found so far.
                z_best.CopyTo(z);
                w_best.CopyTo(w);

                UnityEngine.Debug.LogWarning("Maximum number of iterations exceeded. Returning best result.");
            }

            return minError;
        }
    }
}
