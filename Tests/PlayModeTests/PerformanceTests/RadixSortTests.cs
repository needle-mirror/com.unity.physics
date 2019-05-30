using NUnit.Framework;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.PerformanceTesting;
using Assert = UnityEngine.Assertions.Assert;
using Random = UnityEngine.Random;

namespace Unity.Physics.Tests.PerformanceTests
{
    public class RadixSortTests
    {
        //@TODO: Make part of NativeArray API
        unsafe static NativeArray<U> ReinterpretCast<T, U>(NativeArray<T> array)
            where T : struct
            where U : struct
        {
            Assert.AreEqual(UnsafeUtility.SizeOf<T>(), UnsafeUtility.SizeOf<U>());
            
            var castedArray = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<U>((byte*)array.GetUnsafePtr(), array.Length, Allocator.Invalid);
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref castedArray, NativeArrayUnsafeUtility.GetAtomicSafetyHandle(array));
#endif
            return castedArray;
        }

        
        public static void InitPairs(int minIndex, int maxIndex, int count, NativeArray<ulong> pairs)
        {
            Random.InitState(1234);

            for (var i = 0; i < pairs.Length; ++i)
            {
                ulong indexA = (ulong)Random.Range(minIndex, maxIndex);
                ulong indexB = (ulong)Random.Range(minIndex, maxIndex);

                if (indexB == indexA)
                {
                    if (indexB < (ulong)maxIndex)
                    {
                        indexB++;
                    }
                }

                pairs[i] = indexB << 40 | indexA << 16;
            }
        }



#if UNITY_2019_2_OR_NEWER
        [Test, Performance]
#else
        [PerformanceTest]
#endif
        [TestCase(1, TestName = "PerfRadixPassOnBodyA 1")]
        [TestCase(10, TestName = "PerfRadixPassOnBodyA 10")]
        [TestCase(100, TestName = "PerfRadixPassOnBodyA 100")]
        [TestCase(1000, TestName = "PerfRadixPassOnBodyA 1000")]
        [TestCase(10000, TestName = "PerfRadixPassOnBodyA 10 000")]
        [TestCase(100000, TestName = "PerfRadixPassOnBodyA 100 000")]
        public void PerfRadixPassOnBodyA(int count)
        {
            int maxBodyIndex = (int)math.pow(count, 0.7f);
            int numDigits = 0;
            int val = maxBodyIndex;
            while (val > 0)
            {
                val >>= 1;
                numDigits++;
            }

            var pairs = new NativeArray<ulong>(count, Allocator.TempJob);
            var sortedPairs = new NativeArray<ulong>(count, Allocator.TempJob);
            var tempCount = new NativeArray<int>(maxBodyIndex + 1, Allocator.TempJob);

            InitPairs(1, maxBodyIndex, count, pairs);

            var job = new Scheduler.RadixSortPerBodyAJob
            {
                InputArray = ReinterpretCast<ulong, Scheduler.DispatchPair>(pairs),
                OutputArray = ReinterpretCast<ulong, Scheduler.DispatchPair>(sortedPairs),
                DigitCount = tempCount,
                MaxDigits = numDigits,
                MaxIndex = maxBodyIndex
            };

            Measure.Method(() =>
            {
                job.Run();
            })
            .Definition(sampleUnit: SampleUnit.Microsecond)
            .MeasurementCount(1)
            .Run();

            for (int i = 0; i < count - 1; i++)
            {
                Assert.IsTrue((sortedPairs[i] & 0xffffff0000000000) <= (sortedPairs[i + 1] & 0xffffff0000000000),
                    $"Not sorted for index {i}, sortedPairs[i]= {sortedPairs[i]}, sortedPairs[i+1]= {sortedPairs[i + 1]}");
            }

            // Dispose all allocated data.
            pairs.Dispose();
            sortedPairs.Dispose();
            tempCount.Dispose();
        }

#if UNITY_2019_2_OR_NEWER
        [Test, Performance]
#else
        [PerformanceTest]
#endif
        [TestCase(1, TestName = "PerfDefaultSortOnSubarrays 1")]
        [TestCase(10, TestName = "PerfDefaultSortOnSubarrays 10")]
        [TestCase(100, TestName = "PerfDefaultSortOnSubarrays 100")]
        [TestCase(1000, TestName = "PerfDefaultSortOnSubarrays 1000")]
        [TestCase(10000, TestName = "PerfDefaultSortOnSubarrays 10 000")]
        [TestCase(100000, TestName = "PerfDefaultSortOnSubarrays 100 000")]
        unsafe public void PerfDefaultSortOnSubarrays(int count)
        {
            int maxBodyIndex = (int)math.pow(count, 0.7f);
            int numDigits = 0;
            int val = maxBodyIndex;
            while (val > 0)
            {
                val >>= 1;
                numDigits++;
            }

            var pairs = new NativeArray<ulong>(count, Allocator.TempJob);
            var sortedPairs = new NativeArray<ulong>(count, Allocator.TempJob);

            InitPairs(1, maxBodyIndex, count, pairs);

            // Do a single pass of radix sort on bodyA only.
            var tempCount = new NativeArray<int>(maxBodyIndex + 1, Allocator.TempJob);
            Scheduler.RadixSortPerBodyAJob.RadixSortPerBodyA((ulong*)pairs.GetUnsafePtr(), (ulong*)sortedPairs.GetUnsafePtr(), pairs.Length, tempCount, numDigits, maxBodyIndex, 16);

            var job = new Scheduler.SortSubArraysJob
            {
                InOutArray = ReinterpretCast<ulong, Scheduler.DispatchPair>(sortedPairs),
                NextElementIndex = tempCount
            };

            Measure.Method(() =>
            {
                job.Run(tempCount.Length);
            })
            .Definition(sampleUnit: SampleUnit.Microsecond)
            .MeasurementCount(1)
            .Run();

            for (int i = 0; i < count - 1; i++)
            {
                Assert.IsTrue((sortedPairs[i] & 0x00000000ffffffff) < (sortedPairs[i + 1] & 0x00000000ffffffff) ||
                    (sortedPairs[i] <= sortedPairs[i + 1]),
                    $"Not sorted for index {i}, sortedPairs[i] = {sortedPairs[i]}, sortedPairs[i+1] = {sortedPairs[i + 1]}");
            }

            // Dispose all allocated data.
            pairs.Dispose();
            sortedPairs.Dispose();
        }
    }
}
