Design philosophy
=========

Unity Physics is a complete deterministic rigid body dynamics and spatial query system written entirely in high performance C# using DOTS best practices. The design of Unity Physics follows from the overall DOTS philosophy of minimal dependencies and complete control.

Many experiences today do not necessarily need a full monolithic physics package like PhysX or Havok. Instead you often want a simpler subset of features that can be freely controlled and customized to achieve your desired vision. Unity Physics is designed to balance this control without sacrificing on performance in areas where it appears to be critical. Here are described the design decisions that have been taken in order to achieve this goal.

## Stateless

Modern physics engines maintain large amounts of cached state in order to achieve high performance and simulation robustness. This comes at the cost of added complexity in the simulation pipeline which can be a barrier to modifying code. It also complicates use cases like networking where you may want to roll back and forward physics state. Unity Physics forgoes this caching in favor of simplicity and control.

## Modular

In Unity Physics, core algorithms are deliberately decoupled from jobs and ECS. This is done to encourage their reuse as well as to free you from the underlying framework and stepping strategy.

As an example of this, see the *Immediate Mode* sample. It steps a physical simulation multiple times in a single frame, independent of the job system or ECS, to show future predictions.

## Highly performant

For any physics features that do not cache state, e.g. raw collision queries, we expect the performance of Unity Physics to be on par with, or outperform, similar functionality from commercially available physics engines.

## Interoperable

Unity Physics keeps data compatibility with the Havok Physics Integration (HPI) into Unity. This provides an easy upgrade path for those cases where you do want the full performance and robustness of a commercial solution. Single simulation steps have been split into discrete parts allowing you to write important user code like contact modification or trigger callbacks once and reuse that code with either engine.

# Code base structure

|Collider|Description|
|---|---|
| Base        | Containers and Math used throughout Unity.Physics.                        |
| Collision   | Contains all code for collision detection and spatial queries.            |
| Dynamics    | Contains all code for integration, constraint solving and world stepping. |
| Extensions  | Optional components for characters, vehicles, debugging helpers etc.      |
| ECS         | Contains the components and systems for exposing Unity Physics to ECS.    |
