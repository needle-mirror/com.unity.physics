using System;
using NUnit.Framework;
using Unity.Collections;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using UnityEngine;

namespace Unity.Physics.Tests.Base.Containers
{
    public class BlockStreamTests
    {
        struct WriteInts : IJobParallelFor
        {
            public BlockStream.Writer Writer;

            public void Execute(int index)
            {
                Writer.BeginForEachIndex(index);
                for (int i = 0; i != index; i++)
                    Writer.Write(i);
                Writer.EndForEachIndex();
            }
        }

        struct ReadInts : IJobParallelFor
        {
            public BlockStream.Reader Reader;

            public void Execute(int index)
            {
                int count = Reader.BeginForEachIndex(index);
                Assert.AreEqual(count, index);

                for (int i = 0; i != index; i++)
                {
                    Assert.AreEqual(index - i, Reader.RemainingItemCount);
                    var peekedValue = Reader.Peek<int>();
                    var value = Reader.Read<int>();
                    Assert.AreEqual(i, value);
                    Assert.AreEqual(i, peekedValue);
                }
                Reader.EndForEachIndex();
            }
        }

        [Test]
        public void PopulateInts([Values(1, 100, 200)] int count, [Values(1, 3, 10)] int batchSize)
        {
            var stream = new BlockStream(count, 0x9b98651c);
            var fillInts = new WriteInts { Writer = stream };
            var jobHandle = fillInts.Schedule(count, batchSize);

            var compareInts = new ReadInts { Reader = stream };
            var res0 = compareInts.Schedule(count, batchSize, jobHandle);
            var res1 = compareInts.Schedule(count, batchSize, jobHandle);

            res0.Complete();
            res1.Complete();

            stream.Dispose();
        }
        
        [Test]
        public void CreateAndDestroy([Values(1, 100, 200)] int count)
        {
            var stream = new BlockStream(count, 0x9b98651b);

            Assert.IsTrue(stream.IsCreated);
            Assert.IsTrue(stream.ForEachCount == count);
            Assert.IsTrue(stream.ComputeItemCount() == 0);

            stream.Dispose();
            Assert.IsFalse(stream.IsCreated);
        }

        [Test]
        public void ItemCount([Values(1, 100, 200)] int count, [Values(1, 3, 10)] int batchSize)
        {
            var stream = new BlockStream(count, 0xd3e8afdd);
            var fillInts = new WriteInts { Writer = stream };
            fillInts.Schedule(count, batchSize).Complete();

            Assert.AreEqual(count * (count - 1) / 2, stream.ComputeItemCount());

            stream.Dispose();
        }

        [Test]
        public void ToArray([Values(1, 100, 200)] int count, [Values(1, 3, 10)] int batchSize)
        {
            var stream = new BlockStream(count, 0x11843789);
            var fillInts = new WriteInts { Writer = stream };
            fillInts.Schedule(count, batchSize).Complete();

            var array = stream.ToNativeArray<int>();
            int itemIndex = 0;

            for (int i = 0; i != count; ++i)
            {
                for (int j = 0; j < i; ++j)
                {
                    Assert.AreEqual(j, array[itemIndex]);
                    itemIndex++;
                }
            }

            array.Dispose();

            stream.Dispose();
        }
        
        [Test]
        public void DisposeAfterSchedule()
        {
            var stream = new BlockStream(100, 0xd3e8afdd);
            var fillInts = new WriteInts { Writer = stream };
            var writerJob = fillInts.Schedule(100, 16);

            var disposeJob = stream.Dispose(writerJob);

            Assert.IsFalse(stream.IsCreated);

            disposeJob.Complete();
        }

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        [Test]
        public void ParallelWriteThrows()
        {
            var stream = new BlockStream(100, 0xd3e8afdd);
            var fillInts = new WriteInts { Writer = stream };

            var writerJob = fillInts.Schedule(100, 16);
            Assert.Throws<InvalidOperationException>(() => fillInts.Schedule(100, 16) );

            writerJob.Complete();
            stream.Dispose();
        }
        
        [Test]
        public void ScheduleCreateThrows()
        {
            var list = new NativeList<int>(Allocator.Persistent);
            list.Add(2);
            
            BlockStream stream;
            var jobHandle = BlockStream.ScheduleConstruct(out stream, list, 0xd3e8afdd, default(JobHandle));

            Assert.Throws<InvalidOperationException>(() => Debug.Log(stream.ForEachCount));

            jobHandle.Complete();
            
            Assert.AreEqual(1, stream.ForEachCount);
            
            stream.Dispose();
            list.Dispose();
        }
        
        [Test]
        public void OutOfBoundsWrite()
        {
            var stream = new BlockStream(1, 0x9b98651c);
            BlockStream.Writer writer = stream;
            Assert.Throws<ArgumentException>(() => writer.BeginForEachIndex(-1));
            Assert.Throws<ArgumentException>(() => writer.BeginForEachIndex(2));

            stream.Dispose();
        }

        [Test]
        public void EndForEachIndexWithoutBegin()
        {
            var stream = new BlockStream(1, 0x9b98651c);
            BlockStream.Writer writer = stream;
            Assert.Throws<ArgumentException>(() => writer.EndForEachIndex());

            stream.Dispose();
        }

        [Test]
        public void WriteWithoutBegin()
        {
            var stream = new BlockStream(1, 0x9b98651c);
            BlockStream.Writer writer = stream;
            Assert.Throws<ArgumentException>(() => writer.Write(5));

            stream.Dispose();
        }

        
        static void CreateBlockStream1And2Int(out BlockStream stream)
        {
            stream = new BlockStream(2, 0x9b98651c);

            BlockStream.Writer writer = stream;
            writer.BeginForEachIndex(0);
            writer.Write(0);
            writer.EndForEachIndex();

            writer.BeginForEachIndex(1);
            writer.Write(1);
            writer.Write(2);
            writer.EndForEachIndex();
        }
        
        [Test]
        public void IncompleteRead()
        {
            BlockStream stream;
            CreateBlockStream1And2Int(out stream);

            BlockStream.Reader reader = stream;
            
            reader.BeginForEachIndex(0);
            reader.Read<byte>();
            Assert.Throws<ArgumentException>(() => reader.EndForEachIndex() );
            
            reader.BeginForEachIndex(1);
            
            stream.Dispose();
        }
        
        [Test]
        public void ReadWithoutBegin()
        {
            BlockStream stream;
            CreateBlockStream1And2Int(out stream);

            BlockStream.Reader reader = stream;
            Assert.Throws<ArgumentException>(() => reader.Read<int>() );
            
            stream.Dispose();
        }
        
        [Test]
        public void TooManyReads()
        {
            BlockStream stream;
            CreateBlockStream1And2Int(out stream);

            BlockStream.Reader reader = stream;
            
            reader.BeginForEachIndex(0);
            reader.Read<byte>();
            Assert.Throws<ArgumentException>(() => reader.Read<byte>());
            
            stream.Dispose();
        }
        
        [Test]
        public void OutOfBoundsRead()
        {
            BlockStream stream;
            CreateBlockStream1And2Int(out stream);

            BlockStream.Reader reader = stream;
            
            reader.BeginForEachIndex(0);
            Assert.Throws<ArgumentException>(() => reader.Read<long>());
            
            stream.Dispose();
        }
#endif
        

// Type checks would be nice to have as a toggle'able feature but currently don't exist...
#if false

        [Test]
        public void IncorrectTypedReads()
        {
            var stream = new BlockStream(1);
            BlockStream.Writer writer = stream;
            writer.BeginForEachIndex(0);
            writer.Write<int>(5);
            writer.EndForEachIndex();

            BlockStream.Reader reader = stream;

            reader.BeginForEachIndex(0);
            Assert.Throws<UnityEngine.Assertions.AssertionException>(() => reader.Read<float>());

            stream.Dispose();
        }
#endif

    }
}

