# Design philosophy

Unity Physics is a complete deterministic rigid body dynamics and spatial query system, written entirely in high performance C# using ECS best practices. The design of Unity Physics follows from the overall DOTS philosophy of minimal dependencies and complete control.

Unity Physics is designed to support users who do not necessarily need a full-featured physics package like PhysX or Havok, but instead want a simpler subset of features that they can freely control and customize. The goal of Unity Physics is to provide this in a stable way, without sacrificing on performance in areas where it is critical. This page describes the design decisions that have been taken in order to achieve this goal.

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

| Collider   | Description                                                                                        |
|------------|----------------------------------------------------------------------------------------------------|
| Base       | Containers and Math used throughout Unity.Physics.                                                 |
| Collision  | Contains all code for collision detection and spatial queries.                                     |
| DFG        | Contains all code for DataFlowGraph that performs Collider and Ray Cast query on a CollisionWorld. |
| Dynamics   | Contains all code for integration, constraint solving and world stepping.                          |
| ECS        | Contains the components and systems for exposing Unity Physics to ECS.                             |
| Extensions | Optional components for characters, vehicles, debugging helpers etc.                               |
