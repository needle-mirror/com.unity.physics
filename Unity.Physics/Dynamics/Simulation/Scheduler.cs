using System;
using System.Diagnostics;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine.Assertions;

namespace Unity.Physics
{
    // Builds phased pairs of interacting bodies, used to parallelize work items during the simulation step.
    public class Scheduler : IDisposable
    {
        private readonly BitLookupTable m_BitLookupTable;

        // A pair of interacting bodies (either potentially colliding, or constrained together using a Joint).
        // The indices are compressed into a single 64 bit value, for deterministic sorting, as follows:
        // [BodyAIndex|BodyBIndex|JointIndex]
        // We additionally choose indices so that BodyAIndex < BodyBIndex. This has subtle side-effects:
        // * If one body in the pair is static, it will be body B.
        // * Indices used for jointed pairs are not necessarily the same as selected in the joint
        // * For some body A, all it's static collisions will be contiguous.
        [DebuggerDisplay("{IsJoint ? \"Joint\" : \"Contact\"}, [{BodyAIndex}, {BodyBIndex}]")]
        public struct DispatchPair
        {
            private ulong m_Data;

            private const int k_InvalidBodyIndex = 0xffffff;
            private const int k_InvalidJointIndex = 0x7fff;
            private const ulong k_EnableJointCollisionBit = 0x8000;

            public bool IsValid => m_Data != 0xffffffffffffffff;
            public bool IsContact => JointIndex == k_InvalidJointIndex;
            public bool IsJoint => JointIndex != k_InvalidJointIndex;

            public static DispatchPair Invalid = new DispatchPair { m_Data = 0xffffffffffffffff };

            public int BodyAIndex
            {
                get => (int)(m_Data >> 40);
                set
                {
                    Assert.IsTrue(value < k_InvalidBodyIndex);
                    m_Data = (m_Data & 0x000000ffffffffff) | ((ulong)value << 40);
                }
            }

            public int BodyBIndex
            {
                get => (int)((m_Data >> 16) & k_InvalidBodyIndex);
                set
                {
                    Assert.IsTrue(value < k_InvalidBodyIndex);
                    m_Data = (m_Data & 0xffffff000000ffff) | ((ulong)value << 16);
                }
            }

            public int JointIndex
            {
                get => (int)(m_Data & k_InvalidJointIndex);
                set
                {
                    Assert.IsTrue(value < k_InvalidJointIndex);
                    m_Data = (m_Data & 0xffffffffffff0000) | (uint)(value);
                }
            }

            public bool JointAllowsCollision
            {
                get => (m_Data & k_EnableJointCollisionBit) != 0;
            }

            public static DispatchPair CreateContact(BodyIndexPair pair)
            {
                return Create(pair, k_InvalidJointIndex, 0);
            }

            public static DispatchPair CreateJoint(BodyIndexPair pair, int jointIndex, int allowCollision)
            {
                Assert.IsTrue(jointIndex < k_InvalidJointIndex);
                return Create(pair, jointIndex, allowCollision);
            }

            private static DispatchPair Create(BodyIndexPair pair, int jointIndex, int allowCollision)
            {
                Assert.IsTrue(pair.BodyAIndex < 0xffffff && pair.BodyBIndex < 0xffffff);
                int selectedA = math.min(pair.BodyAIndex, pair.BodyBIndex);
                int selectedB = math.max(pair.BodyAIndex, pair.BodyBIndex);
                return new DispatchPair
                {
                    m_Data = ((ulong)selectedA << 40) | ((ulong)selectedB << 16) | ((ulong)math.min(1, allowCollision) << 15) | (uint)jointIndex
                };
            }
        }

        // A phased set of dispatch pairs.
        // TODO: Better name for this?
        public struct SolverSchedulerInfo : IDisposable
        {
            // A structure which describes the number of items in a single phase
            internal struct SolvePhaseInfo
            {
                internal int DispatchPairCount; // The total number of pairs in this phase
                internal int BatchSize; // The number of items per thread work item; at most, the number of pairs
                internal int NumWorkItems; // The amount of "subtasks" of size BatchSize in this phase
                internal int FirstWorkItemIndex; // The sum of NumWorkItems in all previous phases. Used for output blockstream.
                internal int FirstDispatchPairIndex; // Index into the array of DispatchPairs for this phase.
            }

            internal NativeArray<SolvePhaseInfo> PhaseInfo;
            internal NativeArray<int> NumActivePhases;
            internal NativeArray<int> NumWorkItems;

            public int NumPhases => PhaseInfo.Length;
            internal static int CalculateNumWorkItems(NativeArray<SolvePhaseInfo> phaseInfos)
            {
                int numWorkItems = 0;
                for (int i = 0; i < phaseInfos.Length; i++)
                    numWorkItems += phaseInfos[i].NumWorkItems;
                return numWorkItems;
            }

            // For a given work item returns phase id.
            internal int FindPhaseId(int workItemIndex)
            {
                int phaseId = 0;
                for (int i = NumActivePhases[0] - 1; i >= 0; i--)
                {
                    if (workItemIndex >= PhaseInfo[i].FirstWorkItemIndex)
                    {
                        phaseId = i;
                        break;
                    }
                }

                return phaseId;
            }

            // For a given work item returns index into PhasedDispatchPairs and number of pairs to read.
            internal int GetWorkItemReadOffset(int workItemIndex, out int pairReadCount)
            {
                var phaseInfo = PhaseInfo[FindPhaseId(workItemIndex)];

                int numItemsToRead = phaseInfo.BatchSize;
                int readStartOffset = phaseInfo.FirstDispatchPairIndex + (workItemIndex - phaseInfo.FirstWorkItemIndex) * phaseInfo.BatchSize;

                int lastWorkItemIndex = phaseInfo.FirstWorkItemIndex + phaseInfo.NumWorkItems - 1;
                bool isLastWorkItemInPhase = workItemIndex == lastWorkItemIndex;
                if (isLastWorkItemInPhase)
                {
                    int numPairsBeforeLastWorkItem = (phaseInfo.NumWorkItems - 1) * phaseInfo.BatchSize;
                    numItemsToRead = phaseInfo.DispatchPairCount - numPairsBeforeLastWorkItem;
                    readStartOffset = phaseInfo.FirstDispatchPairIndex + numPairsBeforeLastWorkItem;
                }

                pairReadCount = numItemsToRead;

                return readStartOffset;
            }

            public SolverSchedulerInfo(int numPhases)
            {
                PhaseInfo = new NativeArray<SolvePhaseInfo>(numPhases, Allocator.TempJob);
                NumActivePhases = new NativeArray<int>(1, Allocator.TempJob);
                NumWorkItems = new NativeArray<int>(1, Allocator.TempJob);
            }

            public void Dispose()
            {
                if (PhaseInfo.IsCreated)
                {
                    PhaseInfo.Dispose();
                }

                if (NumActivePhases.IsCreated)
                {
                    NumActivePhases.Dispose();
                }
                
                if (NumWorkItems.IsCreated)
                {
                    NumWorkItems.Dispose();
                }
            }

            public JobHandle ScheduleDisposeJob(JobHandle inputDeps)
            {
                return new DisposeJob { PhaseInfo = PhaseInfo, NumActivePhases = NumActivePhases, NumWorkItems = NumWorkItems }.Schedule(inputDeps);
            }

            // A job to dispose the phase information
            [BurstCompile]
            private struct DisposeJob : IJob
            {
                [DeallocateOnJobCompletion]
                public NativeArray<SolvePhaseInfo> PhaseInfo;

                [DeallocateOnJobCompletion]
                public NativeArray<int> NumActivePhases;

                [DeallocateOnJobCompletion]
                public NativeArray<int> NumWorkItems;
                
                public void Execute() { }
            }
        }

        public Scheduler()
        {
            int numPhases = 17;
            m_BitLookupTable = new BitLookupTable(numPhases);
        }

        public void Dispose()
        {
            m_BitLookupTable.Dispose();
        }
      
        // Sort interacting pairs of bodies into phases for multi-threaded simulation
        public unsafe JobHandle ScheduleCreatePhasedDispatchPairsJob(
            ref PhysicsWorld world, ref BlockStream dynamicVsDynamicBroadphasePairsStream, ref BlockStream staticVsDynamicBroadphasePairStream,
            ref Simulation.Context context, JobHandle inputDeps)
        {            
            // First build a sorted array of dispatch pairs.
            var unsortedPairs = new NativeList<DispatchPair>(Allocator.TempJob);
            var sortedPairs = new NativeList<DispatchPair>(Allocator.TempJob);

            JobHandle sortHandle;
            {
                // Merge broadphase pairs and joint pairs into the unsorted array
                var dispatchHandle = new CreateDispatchPairsJob
                {
                    DynamicVsDynamicPairReader = dynamicVsDynamicBroadphasePairsStream,
                    StaticVsDynamicPairReader = staticVsDynamicBroadphasePairStream,
                    Joints = world.Joints,
                    DispatchPairs = unsortedPairs, 
                    DispatchPairsUninitialized = sortedPairs
                }.Schedule(inputDeps);

                // Dispose our broad phase pairs
                context.DisposeBroadphasePairs0 = dynamicVsDynamicBroadphasePairsStream.Dispose(dispatchHandle);
                context.DisposeBroadphasePairs1 = staticVsDynamicBroadphasePairStream.Dispose(dispatchHandle);

                // Sort into the target array
                sortHandle = ScheduleSortJob(world.NumBodies, unsortedPairs.AsDeferredJobArray(), sortedPairs.AsDeferredJobArray(), dispatchHandle);
            }

            // Create phases for multi-threading
            context.SolverSchedulerInfo = new SolverSchedulerInfo(m_BitLookupTable.NumPhases);
            context.PhasedDispatchPairs = unsortedPairs;
            var dispatchPairHandle = new CreateDispatchPairPhasesJob
            {
                DispatchPairs = sortedPairs.AsDeferredJobArray(),
                SolverSchedulerInfo = context.SolverSchedulerInfo,
                NumDynamicBodies = world.NumDynamicBodies,
                BitLookupTable = m_BitLookupTable.Table,
                NumPhases = m_BitLookupTable.NumPhases,
                PhasedDispatchPairs = unsortedPairs.AsDeferredJobArray()
            }.Schedule(sortHandle);

            // Dispose
            context.DisposePhasedDispatchPairs = NativeListUtilityTemp.DisposeHotFix(ref sortedPairs, dispatchPairHandle);

            return dispatchPairHandle;
        }

        #region Helpers

        // A lookup table used by CreateDispatchPairPhasesJob
        private struct BitLookupTable : IDisposable
        {
            public readonly int NumPhases;
            public readonly NativeArray<ushort> Table;

            public BitLookupTable(int numPhases)
            {
                NumPhases = numPhases;

                Table = new NativeArray<ushort>(UInt16.MaxValue + 1, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                const ushort end = UInt16.MaxValue;
                ushort numBits = (ushort)(numPhases - 1);
                for (ushort value = 0; value < end; value++)
                {
                    ushort valueCopy = value;
                    for (ushort i = 0; i < numBits; i++)
                    {
                        if ((valueCopy & 1) == 0)
                        {
                            Table[value] = i;

                            break;
                        }

                        valueCopy >>= 1;
                    }
                }
                Table[end] = numBits;
            }

            public void Dispose()
            {
                Table.Dispose();
            }
        }

        // Helper function to schedule jobs to sort an array of dispatch pairs.
        // The first single threaded job is a single pass Radix sort on bits 16th to 40th (bodyA index),
        // resulting in sub arrays with the same bodyA index.
        // The second parallel job dispatches default sorts on each sub array.
        private static unsafe JobHandle ScheduleSortJob(
            int numBodies,
            NativeArray<DispatchPair> unsortedPairsIn,
            NativeArray<DispatchPair> sortedPairsOut,
            JobHandle handle)
        {
            NativeArray<int> totalCountUpToDigit = new NativeArray<int>(numBodies + 1, Allocator.TempJob);

            // Calculate number digits needed to encode all body indices
            int numDigits = 0;
            int maxBodyIndex = numBodies - 1;
            {
                int val = maxBodyIndex;
                while (val > 0)
                {
                    val >>= 1;
                    numDigits++;
                }
            }

            // Perform single pass of single threaded radix sort.
            handle = new RadixSortPerBodyAJob
            {
                InputArray = unsortedPairsIn,
                OutputArray = sortedPairsOut,
                MaxDigits = numDigits,
                MaxIndex = maxBodyIndex,
                DigitCount = totalCountUpToDigit
            }.Schedule(handle);

            // Sort sub arrays with default sort.
            int numPerBatch = math.max(1, maxBodyIndex / 32);

            handle = new SortSubArraysJob
            {
                InOutArray = sortedPairsOut,
                NextElementIndex = totalCountUpToDigit
            }.Schedule(totalCountUpToDigit.Length, numPerBatch, handle);

            return handle;
        }

        #endregion

        #region Jobs

        // Combines body pairs and joint pairs into an array of dispatch pairs
        [BurstCompile]
        private struct CreateDispatchPairsJob : IJob
        {
            // Body pairs from broadphase overlap jobs
            public BlockStream.Reader DynamicVsDynamicPairReader;
            public BlockStream.Reader StaticVsDynamicPairReader;

            // Joints from dynamics world
            [ReadOnly]
            public NativeSlice<Joint> Joints;

            public NativeList<DispatchPair> DispatchPairs;
            public NativeList<DispatchPair> DispatchPairsUninitialized;

            public void Execute()
            {
                int numDispatchPairs = DynamicVsDynamicPairReader.ComputeItemCount() + StaticVsDynamicPairReader.ComputeItemCount() + Joints.Length;
                
                DispatchPairs.ResizeUninitialized(numDispatchPairs);
                DispatchPairsUninitialized.ResizeUninitialized(numDispatchPairs);

                var pairs = DispatchPairs.AsArray();
                
                int counter = 0;
                for (int i = 0; i < DynamicVsDynamicPairReader.ForEachCount; i++)
                {
                    DynamicVsDynamicPairReader.BeginForEachIndex(i);
                    int rangeItemCount = DynamicVsDynamicPairReader.RemainingItemCount;
                    for (int j = 0; j < rangeItemCount; j++)
                    {
                        var pair = DynamicVsDynamicPairReader.Read<BodyIndexPair>();
                        pairs[counter++] = DispatchPair.CreateContact(pair);
                    }
                    DynamicVsDynamicPairReader.EndForEachIndex();
                }

                for (int i = 0; i < StaticVsDynamicPairReader.ForEachCount; i++)
                {
                    StaticVsDynamicPairReader.BeginForEachIndex(i);
                    int rangeItemCount = StaticVsDynamicPairReader.RemainingItemCount;
                    for (int j = 0; j < rangeItemCount; j++)
                    {
                        var pair = StaticVsDynamicPairReader.Read<BodyIndexPair>();
                        pairs[counter++] = DispatchPair.CreateContact(pair);
                    }
                    StaticVsDynamicPairReader.EndForEachIndex();
                }

                for (int i = 0; i < Joints.Length; i++)
                {
                    pairs[counter++] = DispatchPair.CreateJoint(Joints[i].BodyPair, i, Joints[i].EnableCollision);
                }

                Assert.AreEqual(counter, pairs.Length);
            }
        }

        // Sorts an array of dispatch pairs by Body A index
        [BurstCompile]
        unsafe public struct RadixSortPerBodyAJob : IJob
        {
            [ReadOnly]
            public NativeArray<DispatchPair> InputArray;
            [NativeDisableParallelForRestriction]
            public NativeArray<DispatchPair> OutputArray;
            [NativeDisableParallelForRestriction]
            public NativeArray<int> DigitCount;

            public int MaxDigits;
            public int MaxIndex;
            
            //@TODO: Add ReinterpretCast to NativeArray

            public void Execute()
            {
                const int shift = 40;
                Assert.AreEqual(InputArray.Length, OutputArray.Length);
                RadixSortPerBodyA((ulong*)InputArray.GetUnsafeReadOnlyPtr(), (ulong*)OutputArray.GetUnsafePtr(), InputArray.Length, DigitCount, MaxDigits, MaxIndex, shift);
            }

            // Performs single pass of Radix sort on NativeArray<ulong> based on 16th to 40th bit. Those bits contain bodyA index in DispatchPair.
            public static void RadixSortPerBodyA(ulong* inputArray, ulong* outputArray, int length, NativeArray<int> digitCount, int maxDigits, int maxIndex, int shift)
            {
                ulong mask = ((ulong)(1 << maxDigits) - 1) << shift;

                // Count digits
                for (int i = 0; i < length; i++)
                {
                    ulong usIndex = inputArray[i] & mask;
                    int sIndex = (int)(usIndex >> shift);
                    digitCount[sIndex]++;
                }

                // Calculate start index for each digit
                int prev = digitCount[0];
                digitCount[0] = 0;
                for (int i = 1; i <= maxIndex; i++)
                {
                    int current = digitCount[i];
                    digitCount[i] = digitCount[i - 1] + prev;
                    prev = current;
                }

                // Copy elements into buckets based on bodyA index
                for (int i = 0; i < length; i++)
                {
                    ulong value = inputArray[i];
                    ulong usindex = value & mask;
                    int sindex = (int)(usindex >> shift);
                    int index = digitCount[sindex]++;
                    if (index == 1 && length == 1)
                    {
                        outputArray[0] = 0;
                    }
                    outputArray[index] = value;
                }
            }
        }

        // Sorts slices of an array in parallel
        [BurstCompile]
        public struct SortSubArraysJob : IJobParallelFor
        {
            [NativeDisableParallelForRestriction]
            public NativeArray<DispatchPair> InOutArray;

            // Typically lastDigitIndex is resulting RadixSortPerBodyAJob.digitCount. nextElementIndex[i] = index of first element with bodyA index == i + 1
            [NativeDisableParallelForRestriction]
            [DeallocateOnJobCompletion] public NativeArray<int> NextElementIndex;

            unsafe public void Execute(int workItemIndex)
            {
                int startIndex = 0;
                if (workItemIndex > 0)
                {
                    startIndex = NextElementIndex[workItemIndex - 1];
                }

                if (startIndex < InOutArray.Length)
                {
                    int length = NextElementIndex[workItemIndex] - startIndex;
                    DefaultSortOfSubArrays((ulong*)InOutArray.GetUnsafePtr(), startIndex, length);
                }
            }

            // Sorts sub array using default sort
            unsafe public static void DefaultSortOfSubArrays(ulong* inOutArray, int startIndex, int length)
            {
                // inOutArray[startIndex] to inOutArray[startIndex + length - 1] have the same bodyA index (16th to 40th big) so we can do a simple sorting.
                if (length > 2)
                {
                    NativeSortExtension.Sort(inOutArray + startIndex, length);
                }
                else if (length == 2)
                {
                    if (inOutArray[startIndex] > inOutArray[startIndex + 1])
                    {
                        ulong temp = inOutArray[startIndex + 1];
                        inOutArray[startIndex + 1] = inOutArray[startIndex];
                        inOutArray[startIndex] = temp;
                    }
                }
            }
        }

        // Creates phases based on sorted list of dispatch pairs
        [BurstCompile]
        private struct CreateDispatchPairPhasesJob : IJob
        {
            [ReadOnly]
            public NativeArray<DispatchPair> DispatchPairs;

            [ReadOnly]
            public NativeArray<ushort> BitLookupTable;

            public SolverSchedulerInfo SolverSchedulerInfo;

            public NativeArray<DispatchPair> PhasedDispatchPairs;

            public int NumDynamicBodies;
            public int NumPhases;

            const int k_MinBatchSize = 8;
            private int m_LastPhaseIndex;

            public void Execute()
            {
                // Call function here, so that one can find it in VTune
                CreateDispatchPairPhasesJobFunction();
            }

            public unsafe void CreateDispatchPairPhasesJobFunction()
            {
                var phaseIdPerPair = new NativeArray<byte>(DispatchPairs.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                var rigidBodyMask = new NativeArray<ushort>(NumDynamicBodies, Allocator.Temp);

                m_LastPhaseIndex = NumPhases - 1;
                int* numPairsPerPhase = stackalloc int[NumPhases];
                numPairsPerPhase[m_LastPhaseIndex] = 0;

                const byte invalidPhaseId = 0xff;

                // Guaranteed not to be a real pair, as bodyA==bodyB==0
                int lastPairA = 0;
                int lastPairB = 0;

                bool contactsPermitted = true; // Will avoid creating a contact (=non-joint) pair if this is false

                BatchInfo* batchInfos = stackalloc BatchInfo[m_LastPhaseIndex];
                for (int i = 0; i < m_LastPhaseIndex; i++)
                {
                    batchInfos[i].m_NumDynamicBodies = NumDynamicBodies;
                    batchInfos[i].m_NumElements = 0;
                    batchInfos[i].m_PhaseMask = (ushort)(1 << i);
                    batchInfos[i].m_NumBatchesProcessed = 0;
                }

                // Find phase for each pair
                for (int i = 0; i < DispatchPairs.Length; i++)
                {
                    int bodyAIndex = DispatchPairs[i].BodyAIndex;
                    int bodyBIndex = DispatchPairs[i].BodyBIndex;

                    bool indicesChanged = !(lastPairA == bodyAIndex && lastPairB == bodyBIndex);
                    bool isJoint = DispatchPairs[i].IsJoint;

                    if (indicesChanged || contactsPermitted || isJoint)
                    {
                        int phaseIndex = FindFreePhase(rigidBodyMask, bodyAIndex, bodyBIndex);
                        phaseIdPerPair[i] = (byte)phaseIndex;

                        if (phaseIndex != m_LastPhaseIndex)
                        {
                            batchInfos[phaseIndex].Add(rigidBodyMask, bodyAIndex, bodyBIndex);
                        }
                        else
                        {
                            numPairsPerPhase[m_LastPhaseIndex]++;
                        }

                        // If _any_ Joint between each Pair has Enable Collision set to true, then contacts will be permitted
                        bool thisPermitsContacts = isJoint && DispatchPairs[i].JointAllowsCollision;
                        contactsPermitted = (contactsPermitted && !indicesChanged) || thisPermitsContacts;

                        lastPairA = bodyAIndex;
                        lastPairB = bodyBIndex;
                    }
                    else
                    {
                        phaseIdPerPair[i] = invalidPhaseId;
                    }
                }

                for (int i = 0; i < m_LastPhaseIndex; i++)
                {
                    numPairsPerPhase[i] = batchInfos[i].m_NumBatchesProcessed * k_MinBatchSize + batchInfos[i].m_NumElements;
                }

                // Calculate phase start offset
                int* offsetInPhase = stackalloc int[NumPhases];
                offsetInPhase[0] = 0;
                for (int i = 1; i < NumPhases; i++)
                {
                    offsetInPhase[i] = offsetInPhase[i - 1] + numPairsPerPhase[i - 1];
                }

                // Populate PhasedDispatchPairsArray
                for (int i = 0; i < DispatchPairs.Length; i++)
                {
                    if (phaseIdPerPair[i] != invalidPhaseId)
                    {
                        int phaseForPair = phaseIdPerPair[i];
                        int indexInArray = offsetInPhase[phaseForPair]++;
                        PhasedDispatchPairs[indexInArray] = DispatchPairs[i];
                    }
                }

                // Populate SolvePhaseInfo for each solve phase
                int firstWorkItemIndex = 0;
                int numPairs = 0;
                int numActivePhases = 0;
                for (int i = 0; i < NumPhases; i++)
                {
                    SolverSchedulerInfo.SolvePhaseInfo info;
                    info.DispatchPairCount = numPairsPerPhase[i];

                    if (info.DispatchPairCount == 0)
                    {
                        break;
                    }

                    info.BatchSize = math.min(k_MinBatchSize, info.DispatchPairCount);
                    info.NumWorkItems = (info.DispatchPairCount + info.BatchSize - 1) / info.BatchSize;
                    info.FirstWorkItemIndex = firstWorkItemIndex;
                    info.FirstDispatchPairIndex = numPairs;

                    firstWorkItemIndex += info.NumWorkItems;
                    numPairs += info.DispatchPairCount;

                    SolverSchedulerInfo.PhaseInfo[i] = info;
                    numActivePhases++;
                }

                SolverSchedulerInfo.NumActivePhases[0] = numActivePhases;

                //<todo.eoin.usermod Can we get rid of this max()? Needed if the user wants to add contacts themselves.
                int numWorkItems = math.max(1, SolverSchedulerInfo.CalculateNumWorkItems(SolverSchedulerInfo.PhaseInfo));
                SolverSchedulerInfo.NumWorkItems[0] = numWorkItems;
            }

            private unsafe struct BatchInfo
            {
                internal void Add(NativeArray<ushort> rigidBodyMasks, int bodyAIndex, int bodyBIndex)
                {
                    int indexInBuffer = m_NumElements++ * 2;

                    fixed (int* bodyIndices = m_BodyIndices)
                    {
                        bodyIndices[indexInBuffer++] = bodyAIndex;
                        bodyIndices[indexInBuffer] = bodyBIndex;

                        if (m_NumElements == k_MinBatchSize)
                        {
                            // Flush
                            indexInBuffer = 0;
                            for (int i = 0; i < k_MinBatchSize; i++)
                            {
                                rigidBodyMasks[bodyIndices[indexInBuffer++]] |= m_PhaseMask;

                                int bIndex = bodyIndices[indexInBuffer++];
                                if (bIndex < m_NumDynamicBodies)
                                {
                                    rigidBodyMasks[bIndex] |= m_PhaseMask;
                                }
                            }

                            m_NumBatchesProcessed++;
                            m_NumElements = 0;
                        }
                    }
                }

                private fixed int m_BodyIndices[k_MinBatchSize * 2];

                internal int m_NumDynamicBodies;
                internal int m_NumBatchesProcessed;
                internal ushort m_PhaseMask;
                internal int m_NumElements;
            }

            private int FindFreePhase(NativeArray<ushort> rigidBodyMask, int bodyAIndex, int bodyBIndex)
            {
                int mask = rigidBodyMask[bodyAIndex];

                if (bodyBIndex < NumDynamicBodies)
                {
                    // Don't need this check for bodyA, as we can guarantee it is dynamic.
                    mask |= rigidBodyMask[bodyBIndex];
                }

                int phaseIndex = BitLookupTable[mask];

                Assert.IsTrue(phaseIndex >= 0 && phaseIndex <= m_LastPhaseIndex);
                return phaseIndex;
            }
        }

        #endregion
    }
}
