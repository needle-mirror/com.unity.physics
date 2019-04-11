using NUnit.Framework;
using Unity.Collections;

namespace Unity.Physics.Tests.Base.Containers
{
    public class EventStreamTests
    {
        [Test]
        public void ReadCollisionEvents()
        {
            // Allocate a block stream for up to 10 parallel writes
            BlockStream collisionEventStream = new BlockStream(10, 0xabbaabba);

            // Do a couple of writes to different forEach indices
            int writeCount = 0;
            {
                BlockStream.Writer collisionEventWriter = collisionEventStream;

                collisionEventWriter.BeginForEachIndex(1);
                collisionEventWriter.Write(new CollisionEvent());
                writeCount++;
                collisionEventWriter.Write(new CollisionEvent());
                writeCount++;
                collisionEventWriter.EndForEachIndex();

                collisionEventWriter.BeginForEachIndex(3);
                collisionEventWriter.Write(new CollisionEvent());
                writeCount++;
                collisionEventWriter.EndForEachIndex();

                collisionEventWriter.BeginForEachIndex(5);
                collisionEventWriter.Write(new CollisionEvent());
                writeCount++;
                collisionEventWriter.Write(new CollisionEvent());
                writeCount++;
                collisionEventWriter.EndForEachIndex();

                collisionEventWriter.BeginForEachIndex(7);
                collisionEventWriter.Write(new CollisionEvent());
                writeCount++;
                collisionEventWriter.EndForEachIndex();

                collisionEventWriter.BeginForEachIndex(9);
                collisionEventWriter.Write(new CollisionEvent());
                writeCount++;
                collisionEventWriter.Write(new CollisionEvent());
                writeCount++;
                collisionEventWriter.EndForEachIndex();
            }

            // Iterate over written events and make sure they are all read
            CollisionEvents collisionEvents = new CollisionEvents(collisionEventStream);
            int readCount = 0;
            foreach(var collisionEvent in collisionEvents)
            {
                readCount++;
            }

            Assert.IsTrue(readCount == writeCount);

            // Cleanup
            var disposeJob = collisionEventStream.ScheduleDispose(default);
            disposeJob.Complete();
        }

        [Test]
        public void ReadTriggerEvents()
        {
            // Allocate a block stream for up to 10 parallel writes
            BlockStream triggerEventStream = new BlockStream(10, 0xbccbbccb);

            // Do a couple of writes to different forEach indices
            int writeCount = 0;
            {
                BlockStream.Writer triggerEventWriter = triggerEventStream;

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
            TriggerEvents triggerEvents = new TriggerEvents(triggerEventStream);
            int readCount = 0;
            foreach (var triggerEvent in triggerEvents)
            {
                readCount++;
            }

            Assert.IsTrue(readCount == writeCount);

            // Cleanup
            var disposeJob = triggerEventStream.ScheduleDispose(default);
            disposeJob.Complete();
        }
    }
}

