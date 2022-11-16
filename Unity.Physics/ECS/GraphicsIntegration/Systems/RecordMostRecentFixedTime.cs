using Unity.Entities;
using Unity.Physics.Systems;
using Unity.Collections;

namespace Unity.Physics.GraphicsIntegration
{
    //Declaring big capacity since buffers with RigidBodySmoothingWorldIndex and MostRecentFixedTime will be stored together in a singleton Entity
    //and that Entity will get a whole Chunk allocated anyway. This capacity is just a limit for keeping the buffer inside the Chunk, reducing it does not affect
    //memory consumption

    /// <summary>
    /// Singleton dynamic buffer that record the last <c>Time.ElapsedTime</c> and <c>Time.DeltaTime</c>
    /// of the most recent tick for each stepped Physics World. The <seealso cref="PhysicsWorldIndex"/>
    /// value is used as index to store and retrieve the timing data. Because of that, the dynamic
    /// buffer size is always equals to the largest PhysicsWorldIndex value set by the application.
    /// </summary>
    [InternalBufferCapacity(256)]
    public struct MostRecentFixedTime : IBufferElementData
    {
        /// <summary>   The delta time. </summary>
        public double DeltaTime;
        /// <summary>   The elapsed time. </summary>
        public double ElapsedTime;
    }

    /// <summary>
    /// A system to keep track of the time values in the most recent tick of the <c>
    /// PhysicsSystemGroup</c>.
    /// </summary>
    [UpdateInGroup(typeof(PhysicsSystemGroup))]
    [UpdateAfter(typeof(PhysicsInitializeGroup)), UpdateBefore(typeof(ExportPhysicsWorld))]
    public partial class RecordMostRecentFixedTime : SystemBase
    {
        private SmoothRigidBodiesGraphicalMotion m_smoothGraphicalMotionSystem;
        private NativeHashSet<PhysicsWorldIndex> m_initializedWorlds;

        protected override void OnCreate()
        {
            base.OnCreate();
            m_smoothGraphicalMotionSystem = World.GetExistingSystemManaged<SmoothRigidBodiesGraphicalMotion>();
            RequireForUpdate<MostRecentFixedTime>();
            m_initializedWorlds = new NativeHashSet<PhysicsWorldIndex>(8, Allocator.Persistent);
        }

        protected override void OnDestroy()
        {
            m_initializedWorlds.Dispose();
            base.OnDestroy();
        }

        protected override void OnUpdate()
        {
            var worldIndex = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorldIndex;
            if (!m_initializedWorlds.Contains(worldIndex))
            {
                //Let the graphics system smooth the rigid body motion for the physics world using recent smooth time.
                m_smoothGraphicalMotionSystem.RegisterPhysicsWorldForSmoothRigidBodyMotion(worldIndex);
                m_initializedWorlds.Add(worldIndex);
            }
            var mostRecentTimeBuffer = SystemAPI.GetBuffer<MostRecentFixedTime>(SystemAPI.GetSingletonEntity<MostRecentFixedTime>());
            mostRecentTimeBuffer[(int)worldIndex.Value] = new MostRecentFixedTime
            {
                ElapsedTime = SystemAPI.Time.ElapsedTime,
                DeltaTime = SystemAPI.Time.DeltaTime
            };
        }
    }
}
