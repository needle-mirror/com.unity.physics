# Unity Physics 

## Design philosophy

We want to provide you with a complete deterministic rigid body dynamics and spatial query system written entirely in high performance C# using DOTS best practices. The design of Unity Physics follows from the overall DOTS philosophy of minimal dependencies and complete control. Many experiences today do not need a full monolithic physic package like PhysX or Havok. Instead you often want simpler subset of features that you can freely control and customize to achieve your vision. Unity Physics is designed to balance this control without sacrificing on performance in areas we feel it is critical. In order to achieve this goal we've made a number of design decisions described here

### Stateless

Modern physics engines maintain large amounts of cached state in order to achieve high performance and simulation robustness. This comes at the cost of added complexity in the simulation pipeline which can be a barrier to modifying code. It also complicates use cases like networking where you may want to roll back and forward physics state. Unity Physics forgoes this caching in favor of simplicity and control.

### Modular

Core algorithms are deliberately decoupled from jobs and ECS to encourage their reuse and to free you from the underlying framework and stepping strategy. As an example of this see the immediate mode sample which steps a physical simulation multiple times in a single frame, independent of the job system or ECS, to show future predictions.

### Highly performant

For any physics features that do not cache state, e.g. raw collision queries, we expect Unity Physics performance to be on par with, or outperform, similar functionality from commercially available physics engines.

### Interoperable 

We keep data compatibility between Unity Physics and the Havok Physics integration into unity (HPI). This give an easy upgrade path for those cases where you do want the full performance and robustness of a commercial solution. We split a single simulation step into discrete parts allowing you to write important user code like contact modification or trigger callbacks once and reuse this with either engine. In the future we intend to allow both Unity Physics and HPI to run side by side. 

## Code base structure

|Collider|Description|
|---|---|
| Base | Containers and Math used throughout Unity.Physics |
| Collision | Contains all code for collision detection and spatial queries |
| Dynamics | Contains all code for integration, constraint solving and world stepping |
| Extensions | Optional components for characters, vehicles, debugging helpers etc.  |
| ECS | Contain the components and systems for exposing Unity Physics to ECS |

[Back to Index](index.md)
