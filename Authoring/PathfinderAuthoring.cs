using Pathfinding.Components;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.AI;

namespace Pathfinding.Authoring
{
    public class PathfinderAuthoring : MonoBehaviour
    {
        [Tooltip("Minimal distance for which paths are queried")]
        public float minDistance;

        [Tooltip("Agent index which matches agent in navigation settings")]
        public int agentId;

        [Tooltip("If it should use the funnelling algorithm for a better path")]
        public bool useFunnel;

        public class PathfinderAuthoringBaker : Baker<PathfinderAuthoring>
        {
            public override void Bake(PathfinderAuthoring authoring)
            {
                Entity entity = GetEntity(TransformUsageFlags.Dynamic);
                AddComponent(entity, new Pathfinder
                {
                    requiredMinDistanceSq = math.pow(authoring.minDistance, 2),
                    agentId = NavMesh.GetSettingsByIndex(authoring.agentId).agentTypeID,
                    useFunnel = authoring.useFunnel,
                    pathStatus = 0
                });
                SetComponentEnabled<Pathfinder>(entity, false);

                AddBuffer<PathBuffer>(entity);
            }
        }
    }
}