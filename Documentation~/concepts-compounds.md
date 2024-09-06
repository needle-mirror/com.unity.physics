# Compound Colliders

Compound collider shapes allow you to place multiple collider shapes on one physics body. You can arrange the shapes into a hierarchy, and adjust the transform of each shape.

A single GameObject with a built-in `Rigidbody` or a custom `Physics Body` authoring component can contain multiple child GameObjects that have built-in `Collider` or custom `Physics Shape` authoring components. Unity Physics merges these into one compound `PhysicsCollider` for the physics body, containing a `CompoundCollider` blob asset whose children have the collider types you specified in the Editor.

This allows you to make complex rigid bodies made of multiple simpler representations, so that the collision detection remains fast yet detailed, and you don't have to settle for a coarse convex hull around the whole object.

In the following example, **Body** contains a single `Rigidbody` authoring component that represents a humanoid torso. The child GameObjects contain one `Collider` authoring component each; four `Box` colliders to represent limbs and a `Sphere` collider to represent a head. All child colliders will be regrouped in a `CompoundCollider` blob asset assigned to the `PhysicsCollider` component in the rigid body entity that is baked from the **Body** GameObject.

![compound_objs](images/compunds-H.png)
<br/>*Compound shape hierarchy*

>[!NOTE]
> For this example, you must put all GameObjects (**Body** and its children) in a [sub scene](https://docs.unity3d.com/Packages/com.unity.entities@latest/index.html?subfolder=/manual/conversion-subscenes.html) so that Unity can perform [baking](https://docs.unity3d.com/Packages/com.unity.entities@latest/index.html?subfolder=/manual/baking.html) to convert them into entities and components.

![compound_sim](images/compounds.gif)
<br/>*The Compound shape in Play mode*
