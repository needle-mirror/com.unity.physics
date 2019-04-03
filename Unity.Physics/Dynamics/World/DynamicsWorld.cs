using System;
using Unity.Collections;

namespace Unity.Physics
{
    // A collection of motion information used during physics simulation.
    public struct DynamicsWorld : IDisposable, ICloneable
    {
        private NativeArray<MotionData> m_MotionDatas;
        private NativeArray<MotionVelocity> m_MotionVelocities;
        private int m_NumMotions; // number of motionDatas and motionVelocities currently in use

        private NativeArray<Joint> m_Joints;
        private int m_NumJoints; // number of joints currently in use

        public NativeSlice<MotionData> MotionDatas => new NativeSlice<MotionData>(m_MotionDatas, 0, m_NumMotions);
        public NativeSlice<MotionVelocity> MotionVelocities => new NativeSlice<MotionVelocity>(m_MotionVelocities, 0, m_NumMotions);
        public NativeSlice<Joint> Joints => new NativeSlice<Joint>(m_Joints, 0, m_NumJoints);

        public int NumMotions
        {
            get => m_NumMotions;
            set
            {
                m_NumMotions = value;
                if (m_MotionDatas.Length < m_NumMotions)
                {
                    m_MotionDatas.Dispose();
                    m_MotionDatas = new NativeArray<MotionData>(m_NumMotions, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                }
                if (m_MotionVelocities.Length < m_NumMotions)
                {
                    m_MotionVelocities.Dispose();
                    m_MotionVelocities = new NativeArray<MotionVelocity>(m_NumMotions, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                }
            }
        }

        public int NumJoints
        {
            get => m_NumJoints;
            set
            {
                m_NumJoints = value;
                if (m_Joints.Length < m_NumJoints)
                {
                    m_Joints.Dispose();
                    m_Joints = new NativeArray<Joint>(m_NumJoints, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                }
            }
        }


        // Construct a dynamics world with the given number of uninitialized motions
        public DynamicsWorld(int numMotions, int numJoints)
        {
            m_MotionDatas = new NativeArray<MotionData>(numMotions, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            m_MotionVelocities = new NativeArray<MotionVelocity>(numMotions, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            m_NumMotions = numMotions;

            m_Joints = new NativeArray<Joint>(numMotions, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            m_NumJoints = numJoints;
        }

        // Free internal memory
        public void Dispose()
        {
            m_MotionDatas.Dispose();
            m_MotionVelocities.Dispose();
            m_Joints.Dispose();
        }

        // Clone the world
        public object Clone()
        {
            DynamicsWorld clone = new DynamicsWorld
            {
                m_MotionDatas = new NativeArray<MotionData>(m_MotionDatas.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory),
                m_MotionVelocities = new NativeArray<MotionVelocity>(m_MotionVelocities.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory),
                m_NumMotions = m_NumMotions,
                m_Joints = new NativeArray<Joint>(m_Joints.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory),
                m_NumJoints = m_NumJoints
            };
            clone.m_MotionDatas.CopyFrom(m_MotionDatas);
            clone.m_MotionVelocities.CopyFrom(m_MotionVelocities);
            clone.m_Joints.CopyFrom(m_Joints);
            return clone;
        }
    }
}
