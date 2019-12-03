using System;
using NUnit.Framework;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace Unity.Physics.Tests.Base.Containers
{
    class EventStreamTests
    {
        private unsafe void WriteEvent(LowLevel.CollisionEvent collisionEvent, ref NativeStream.Writer collisionEventWriter)
        {
            int numContactPoints = collisionEvent.NumNarrowPhaseContactPoints;
            int size = UnsafeUtility.SizeOf<LowLevel.CollisionEvent>() + numContactPoints * UnsafeUtility.SizeOf<ContactPoint>();

            collisionEventWriter.Write(size);
            byte* eventPtr = collisionEventWriter.Allocate(size);
            ref LowLevel.CollisionEvent eventRef = ref UnsafeUtilityEx.AsRef<LowLevel.CollisionEvent>(eventPtr);
            eventRef = collisionEvent;
            for (int i = 0; i < numContactPoints; i++)
            {
                eventRef.AccessContactPoint(i) = new ContactPoint();
            }
        }

        [Test]
        public void ReadCollisionEvents()
        {
            // Allocate a native stream for up to 10 parallel writes
            var collisionEventStream = new NativeStream(10, Allocator.TempJob);

            // Do a couple of writes to different forEach indices
            int writeCount = 0;
            unsafe
            {
                NativeStream.Writer collisionEventWriter = collisionEventStream.AsWriter();

                var collisionEvent = new LowLevel.CollisionEvent();

                collisionEventWriter.BeginForEachIndex(1);
                {
                    collisionEvent.NumNarrowPhaseContactPoints = 1;
                    WriteEvent(collisionEvent, ref collisionEventWriter);
                    writeCount++;

                    collisionEvent.NumNarrowPhaseContactPoints = 4;
                    WriteEvent(collisionEvent, ref collisionEventWriter);
                    writeCount++;
                }
                collisionEventWriter.EndForEachIndex();

                collisionEventWriter.BeginForEachIndex(3);
                {
                    collisionEvent.NumNarrowPhaseContactPoints = 3;
                    WriteEvent(collisionEvent, ref collisionEventWriter);
                    writeCount++;
                }
                collisionEventWriter.EndForEachIndex();

                collisionEventWriter.BeginForEachIndex(5);
                {
                    collisionEvent.NumNarrowPhaseContactPoints = 4;
                    WriteEvent(collisionEvent, ref collisionEventWriter);
                    writeCount++;

                    collisionEvent.NumNarrowPhaseContactPoints = 2;
                    WriteEvent(collisionEvent, ref collisionEventWriter);
                    writeCount++;
                }
                collisionEventWriter.EndForEachIndex();

                collisionEventWriter.BeginForEachIndex(7);
                {
                    collisionEvent.NumNarrowPhaseContactPoints = 1;
                    WriteEvent(collisionEvent, ref collisionEventWriter);
                    writeCount++;
                }
                collisionEventWriter.EndForEachIndex();

                collisionEventWriter.BeginForEachIndex(9);
                {
                    collisionEvent.NumNarrowPhaseContactPoints = 4;
                    WriteEvent(collisionEvent, ref collisionEventWriter);
                    writeCount++;

                    collisionEvent.NumNarrowPhaseContactPoints = 1;
                    WriteEvent(collisionEvent, ref collisionEventWriter);
                    writeCount++;
                }
                collisionEventWriter.EndForEachIndex();
            }

            // Iterate over written events and make sure they are all read
            LowLevel.CollisionEvents collisionEvents = new LowLevel.CollisionEvents(collisionEventStream);
            int readCount = 0;
            foreach(var collisionEvent in collisionEvents)
            {
                readCount++;
            }

            Assert.IsTrue(readCount == writeCount);

            // Cleanup
            var disposeJob = collisionEventStream.Dispose(default);
            disposeJob.Complete();
        }

        [Test]
        public void ReadTriggerEvents()
        {
            // Allocate a native stream for up to 10 parallel writes
            var triggerEventStream = new NativeStream(10, Allocator.TempJob);

            // Do a couple of writes to different forEach indices
            int writeCount = 0;
            {
                NativeStream.Writer triggerEventWriter = triggerEventStream.AsWriter();

                triggerEventWriter.BeginForEachIndex(1);
                triggerEventWriter.Write(new TriggerEvent());
                writeCount++;
                triggerEventWriter.EndForEachIndex();

                triggerEventWriter.BeginForEachIndex(3);
                triggerEventWriter.Write(new TriggerEvent());
                writeCount++;
                triggerEventWriter.Write(new TriggerEvent());
                writeCount++;
                triggerEventWriter.EndForEachIndex();

                triggerEventWriter.BeginForEachIndex(5);
                triggerEventWriter.Write(new TriggerEvent());
                writeCount++;
                triggerEventWriter.Write(new TriggerEvent());
                writeCount++;
                triggerEventWriter.EndForEachIndex();

                triggerEventWriter.BeginForEachIndex(7);
                triggerEventWriter.Write(new TriggerEvent());
                writeCount++;
                triggerEventWriter.Write(new TriggerEvent());
                writeCount++;
                triggerEventWriter.EndForEachIndex();

                triggerEventWriter.BeginForEachIndex(9);
                triggerEventWriter.Write(new TriggerEvent());
                writeCount++;
                triggerEventWriter.EndForEachIndex();
            }

            // Iterate over written events and make sure they are all read
            LowLevel.TriggerEvents triggerEvents = new LowLevel.TriggerEvents(triggerEventStream);
            int readCount = 0;
            foreach (var triggerEvent in triggerEvents)
            {
                readCount++;
            }

            Assert.IsTrue(readCount == writeCount);

            // Cleanup
            var disposeJob = triggerEventStream.Dispose(default);
            disposeJob.Complete();
        }
    }
}

