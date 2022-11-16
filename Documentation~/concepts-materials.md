# Physics Materials

## Using Unity Physics Authoring components

Materials are a set of properties that affect how a Physics Body reacts when it encounters another Physics Body. The material properties in Unity Physics include **Friction**, **Restitution** (bounciness), and what the object collides with. Material properties can be saved to a **Physics Material Template** asset, or set directly in the **Physics Shape** for an object.

Try the following settings with the example scene described in [Simulation setup demonstration](concepts-simulation-set-up.md).
1. Select the Sphere. In the **Physics Shape** component's **Material** section, set the **Restitution** to **1**. When you enter play mode, the sphere should now bounce back from collision to nearly where it began.
2. In the **Physics Body** component, change the **Linear Damping** to 0. The sphere should now get even closer to its starting point.
3. Rotate the floor so that it is tilted, to allow the Sphere to roll, then select the Sphere and go to **Physics Shape** > **Material**. Adjust the **Friction** setting and experiment with the change in movement. The higher the **Friction** value is, the more the sphere catches and rolls on the surface, rather than just sliding.
4. Select the Sphere and go to **Physics Shape** > **Material**. Experiment the with the **Restitution** value. The higher the **Restitution**, the more the sphere bounces on contact.

The following table describes the fields in the **PhysicsMaterialTemplate** asset:

| Field              | Description                                                                                                                                                                                                                                                                                           |
|--------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Collision Response | Collide: results in collision without raising any event. <br/>Collide Raise Collision Events: results in collision and raises events when contact is made. <br/>Raise Trigger Events: results in no collision but triggers events. <br/>None: results in no collision with no events being triggered. |
| Friction           | Range bar modifies the friction value when physics bodies slide down onto a tilded surface.                                                                                                                                                                                                           |
| Restitution        | Range bar modifies the restitution also known as bounciness of physics bodies when they collide each other.                                                                                                                                                                                           |
| Collision Filter   | Needed for triggering action either in Collide, Collide Raise Collision Events or Raise Trigger Events.                                                                                                                                                                                               |
| Custom Tags        | Select or add new Custom Physics Material Names.                                                                                                                                                                                                                                                      |

![concept-materials](images/material-template.png)<br/>_Example of the **PhysicsMaterialTemplate** asset._
