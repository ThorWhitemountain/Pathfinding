using Unity.Entities;
using Unity.Mathematics;
using UnityEngine.Experimental.AI;

namespace Pathfinding.Components
{
    public struct Pathfinder : IComponentData, IEnableableComponent
    {
        public bool useFunnel;
        public float requiredMinDistanceSq;
        public int agentId;

        public PathQueryStatus pathStatus;

        // Internal state for the NavMeshQuery to track progress
        public NavMeshLocation fromLocation;
        public NavMeshLocation toLocation;

        public float3 from;
        public float3 to;
    }
}