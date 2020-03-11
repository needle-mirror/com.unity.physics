using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using static Unity.Physics.DispatchPairSequencer;

namespace Unity.Physics.Tests.Dynamics.Schedule
{
    // Tests to validate scheduler implementation
    class SchedulerTests
    {
        // Regression test.
        // It is not very often in current tests that there are full batches of dispatch pairs in solve phases.
        // There was an issue where having 2 full batches (and some overflow pairs) in 2 consecutive phases lead
        // to a race condition in the solver as wrong pairs were being solved in parallel.
        // This test illustrates one example of such issue and verifies that it doesn't happen anymore.
        [Test]
        public void TestSchedulerWithFullBatches()
        {
            DispatchPairSequencer sequencer = new DispatchPairSequencer();

            int numDynamicBodies = 15;
            var solverSchedulerInfo = new SolverSchedulerInfo(sequencer.m_PhaseLookupTableDynamicDynamicPairs.NumPhases);
            var phasedDispatchPairs = new NativeArray<DispatchPair>(20, Allocator.Temp);
            var sortedDispatchPairs = new NativeArray<DispatchPair>(20, Allocator.Temp);
            {
                sortedDispatchPairs[0] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 0, BodyBIndex = 1 });
                sortedDispatchPairs[1] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 0, BodyBIndex = 2 });
                sortedDispatchPairs[2] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 0, BodyBIndex = 3 });
                sortedDispatchPairs[3] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 0, BodyBIndex = 4 });
                sortedDispatchPairs[4] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 0, BodyBIndex = 5 });
                sortedDispatchPairs[5] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 0, BodyBIndex = 6 });
                sortedDispatchPairs[6] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 0, BodyBIndex = 7 });
                sortedDispatchPairs[7] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 0, BodyBIndex = 8 });
                sortedDispatchPairs[8] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 0, BodyBIndex = 15 });
                sortedDispatchPairs[9] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 0, BodyBIndex = 16 });
                sortedDispatchPairs[10] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 1, BodyBIndex = 2 });
                sortedDispatchPairs[11] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 1, BodyBIndex = 3 });
                sortedDispatchPairs[12] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 1, BodyBIndex = 4 });
                sortedDispatchPairs[13] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 1, BodyBIndex = 5 });
                sortedDispatchPairs[14] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 1, BodyBIndex = 6 });
                sortedDispatchPairs[15] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 1, BodyBIndex = 7 });
                sortedDispatchPairs[16] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 1, BodyBIndex = 8 });
                sortedDispatchPairs[17] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 1, BodyBIndex = 9 });
                sortedDispatchPairs[18] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 1, BodyBIndex = 15 });
                sortedDispatchPairs[19] = DispatchPair.CreateCollisionPair(new BodyIndexPair { BodyAIndex = 1, BodyBIndex = 16 });
            }

            var createDispatchPairsJob = new CreateDispatchPairPhasesJob
            {
                DispatchPairs = sortedDispatchPairs,
                NumDynamicBodies = numDynamicBodies,
                NumPhases = sequencer.m_PhaseLookupTableDynamicDynamicPairs.NumPhases,
                PhasedDispatchPairs = phasedDispatchPairs,
                PhaseLookupTableDynamicDynamicPairs = sequencer.m_PhaseLookupTableDynamicDynamicPairs.Table,
                PhaseLookupTableDynamicStaticPairs = sequencer.m_PhaseLookupTableDynamicStaticPairs.Table,
                SolverSchedulerInfo = solverSchedulerInfo
            };
            createDispatchPairsJob.Execute();

            // Verification
            {
                int dispatchPairCount = 0;
                for (int i = 0; i < solverSchedulerInfo.NumActivePhases[0]; i++)
                {
                    SolverSchedulerInfo.SolvePhaseInfo info = solverSchedulerInfo.PhaseInfo[i];
                    dispatchPairCount += info.DispatchPairCount;

                    var firstWorkItemIndex = info.FirstWorkItemIndex;
                    {
                        for (int j = firstWorkItemIndex; j < firstWorkItemIndex + info.NumWorkItems - 1; j++)
                        {
                            for (int k = j + 1; k < j + 1 + info.NumWorkItems; k++)
                            {
                                int firstIndex = j * info.BatchSize;
                                int secondIndex = k * info.BatchSize;
                                for (int pairIndex1 = firstIndex; pairIndex1 < math.min(firstIndex + info.BatchSize, dispatchPairCount); pairIndex1++)
                                {
                                    int aIndex1 = phasedDispatchPairs[pairIndex1].BodyAIndex;
                                    int bIndex1 = phasedDispatchPairs[pairIndex1].BodyBIndex;
                                    for (int pairIndex2 = secondIndex; pairIndex2 < math.min(secondIndex + info.BatchSize, dispatchPairCount); pairIndex2++)
                                    {
                                        int aIndex2 = phasedDispatchPairs[pairIndex2].BodyAIndex;
                                        int bIndex2 = phasedDispatchPairs[pairIndex2].BodyBIndex;

                                        // Verify that different batches can't contain same dynamic bodies
                                        Assert.AreNotEqual(aIndex1, aIndex2);
                                        if (bIndex1 < numDynamicBodies || bIndex2 < numDynamicBodies)
                                        {
                                            Assert.AreNotEqual(bIndex1, bIndex2);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            solverSchedulerInfo.Dispose();
            sequencer.Dispose();
        }
    }
}
