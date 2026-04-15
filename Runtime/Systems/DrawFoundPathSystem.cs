using Pathfinding.Components;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.Authoring;

namespace Pathfinding.Systems
{
    [UpdateInGroup(typeof(PresentationSystemGroup))]
    public partial struct DrawFoundPathSystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<PhysicsDebugDisplayData>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            SystemAPI.GetSingleton<PhysicsDebugDisplayData>();

            foreach (DynamicBuffer<PathBuffer> path in SystemAPI.Query<DynamicBuffer<PathBuffer>>())
            {
                if (path.IsEmpty)
                {
                    continue;
                }

                NativeArray<float3> pathArray = path.AsNativeArray().Reinterpret<float3>();
                for (int i = 0; i < (pathArray.Length - 1); i++)
                {
                    float3 pos = pathArray[i];
                    float3 nextPos = pathArray[i + 1];
                    PhysicsDebugDisplaySystem.Line(pos, nextPos, Unity.DebugDisplay.ColorIndex.Green);
                }
            }
        }
    }
}