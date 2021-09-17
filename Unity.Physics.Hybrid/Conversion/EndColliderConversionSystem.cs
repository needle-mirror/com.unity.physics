using System.Collections.Generic;
using Unity.Entities;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    [UpdateAfter(typeof(BeginColliderConversionSystem))]
    [AlwaysUpdateSystem]
    public sealed class EndColliderConversionSystem : GameObjectConversionSystem
    {
        BeginColliderConversionSystem m_BeginColliderConversionSystem;

        // TODO: move this to a ColliderConversionSystemGroup when groups are supported for GameObjectConversionSystems
        List<Component> m_ConvertedAuthoringComponents = new List<Component>(128);

        internal int PushAuthoringComponent(Component c)
        {
            m_ConvertedAuthoringComponents.Add(c);
            return m_ConvertedAuthoringComponents.Count - 1;
        }

        internal Component GetConvertedAuthoringComponent(int i) => m_ConvertedAuthoringComponents[i];

        protected override void OnCreate()
        {
            base.OnCreate();

            m_BeginColliderConversionSystem = World.GetOrCreateSystem<BeginColliderConversionSystem>();
        }

        protected override void OnUpdate()
        {
            m_BeginColliderConversionSystem.BlobComputationContext.Dispose();
            m_ConvertedAuthoringComponents.Clear();
        }
    }
}
