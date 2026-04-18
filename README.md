# Pathfinding

Pathfinding - using Unity Navmesh, ECS and Burst.

This fork represents self-sufficient
module that can be used with Unity Entities and NavMesh.

### Authoring pathfinder

In order to add all necessary components to authoring object
simply add `PathfinderAuthoring` component.

Specify agentID and minimal query distance and it's ready to be baked.

### Finding path

To find a path you need to get the disabled Pathfinder component 
of the relevant entity, update it's .to, .from, and clear its .pathstatus (set to 0).

Enable the pathfinder component, and wait.

After pathfinding is done the `PathBuffer` dynamic buffer will be filled with
direct path position. 

`Pathfinder` component will also get it's `pathStatus` field updated, and be disabled. Which is useful for job filtering via [WithNone(typeof(Pathfinder))]