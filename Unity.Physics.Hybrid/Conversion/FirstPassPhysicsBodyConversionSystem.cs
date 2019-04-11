using UnityEngine;

namespace Unity.Physics.Authoring
{
    public class FirstPassPhysicsBodyConversionSystem : GameObjectConversionSystem
    {
        protected override void OnUpdate()
        {
            Entities.ForEach(
                (PhysicsBody body) =>
                {
                    if (!body.enabled) return;
                    var entity = GetPrimaryEntity(body.gameObject);
                    DstEntityManager.AddOrSetComponent(entity, new PhysicsCollider());
                }
            );
        }
    }
}
