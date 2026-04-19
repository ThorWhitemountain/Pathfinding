using Unity.Entities;
using Unity.Mathematics;
using UnityEngine.Experimental.AI;

namespace Pathfinding.Components
{
    public struct Pathfinder : IComponentData, IEnableableComponent
    {
        public int queryIndex; // -1 if no query assigned
        public bool useFunnel;
        public float requiredMinDistanceSq;
        public int agentId;

        public PathQueryStatus pathStatus;

        public float3 from;
        public float3 to;

        // Internal state for the NavMeshQuery to track progress
        public NavMeshLocation fromLocation;
        public NavMeshLocation toLocation;
    }
}