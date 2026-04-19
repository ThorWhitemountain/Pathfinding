using Pathfinding.Components;
using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Experimental.AI;

namespace Pathfinding.Systems
{
    [UpdateInGroup(typeof(SimulationSystemGroup))]
    public partial struct PathfinderSystem : ISystem
    {
        private NavMeshWorld _navMeshWorld;

        private NativeArray<NavMeshQuery> _navMeshQueries;

        // used to connect entities to NavmeshQueries
        private NativeQueue<int> _availableIndices;
        private const int MaxConcurrentPaths = 16;

        private EntityQuery pathfindingQ;

        public void OnCreate(ref SystemState state)
        {
            pathfindingQ = new EntityQueryBuilder(state.WorldUpdateAllocator)
                .WithAllRW<Pathfinder, PathBuffer>()
                .Build(state.EntityManager);

            state.RequireForUpdate<BeginInitializationEntityCommandBufferSystem.Singleton>();
            _navMeshWorld = NavMeshWorld.GetDefaultWorld();
            _navMeshQueries = new NativeArray<NavMeshQuery>(MaxConcurrentPaths, Allocator.Persistent);
            _availableIndices = new NativeQueue<int>(Allocator.Persistent);

            for (int i = 0; i < MaxConcurrentPaths; i++)
            {
                _navMeshQueries[i] = new NavMeshQuery(_navMeshWorld, Allocator.Persistent, 4096);
                _availableIndices.Enqueue(i);
            }
        }

        public void OnDestroy(ref SystemState state)
        {
            if (!_navMeshQueries.IsCreated)
            {
                return;
            }

            for (int i = 0; i < _navMeshQueries.Length; i++)
            {
                _navMeshQueries[i].Dispose();
            }

            _navMeshQueries.Dispose();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            EntityCommandBuffer.ParallelWriter ecb = SystemAPI.GetSingleton<BeginInitializationEntityCommandBufferSystem.Singleton>()
                .CreateCommandBuffer(state.WorldUnmanaged).AsParallelWriter();

            InitJob initJob = new()
            {
                queries = _navMeshQueries,
                availableIndices = _availableIndices
            };
            state.Dependency = initJob.Schedule(pathfindingQ, state.Dependency);

            PathProcessorJob processorJob = new()
            {
                queries = _navMeshQueries,
                availableIndices = _availableIndices.AsParallelWriter(),
                maxIterations = 100,
                ecb = ecb
            };
            state.Dependency = processorJob.ScheduleParallel(pathfindingQ, state.Dependency);
        }

        [BurstCompile]
        public partial struct InitJob : IJobEntity
        {
            [NativeDisableContainerSafetyRestriction]
            public NativeArray<NavMeshQuery> queries;

            public NativeQueue<int> availableIndices;

            private void Execute(Entity entity, [EntityIndexInQuery] int queryIndex, ref Pathfinder pathfinder)
            {
                if (pathfinder.queryIndex != -1)
                {
                    // Debug.Log("Already have a query!");
                    return;
                }

                ref float3 sourcePos = ref pathfinder.from;
                ref float3 targetPos = ref pathfinder.to;
                if (math.all(sourcePos == targetPos))
                {
                    return;
                }

                float length = math.distancesq(sourcePos, targetPos);
                if (length <= pathfinder.requiredMinDistanceSq)
                {
                    return;
                }

                // Already in progress!
                if (pathfinder.pathStatus != 0)
                {
                    // Debug.Log("Uh oh, would've gotten a new query even though im already in progress!");
                    return;
                }

                if (math.distancesq(pathfinder.from, pathfinder.to) < pathfinder.requiredMinDistanceSq)
                {
                    return;
                }

                if (!availableIndices.TryDequeue(out int availableIndex))
                {
                    // Debug.Log("No queries available!");
                    return;
                }

                NavMeshQuery query = queries[availableIndex];
                pathfinder.fromLocation = query.MapLocation(pathfinder.from, new float3(10), pathfinder.agentId);
                pathfinder.toLocation = query.MapLocation(pathfinder.to, new float3(10), pathfinder.agentId);

                if (query.IsValid(pathfinder.fromLocation) && query.IsValid(pathfinder.toLocation))
                {
                    // Debug.Log($"Using query at index: {availableIndex} -- Entity: {entity.ToFixedString()}");
                    pathfinder.queryIndex = queryIndex;
                    pathfinder.pathStatus = query.BeginFindPath(pathfinder.fromLocation, pathfinder.toLocation);
                }
                else
                {
                    // Debug.Log("Invalid query! returning!");

                    availableIndices.Enqueue(availableIndex);
                    pathfinder.queryIndex = -1;
                }
            }
        }

        [BurstCompile]
        public partial struct PathProcessorJob : IJobEntity
        {
            [NativeDisableContainerSafetyRestriction]
            public NativeArray<NavMeshQuery> queries;

            public NativeQueue<int>.ParallelWriter availableIndices;

            public int maxIterations;

            public EntityCommandBuffer.ParallelWriter ecb;

            private void Execute(Entity entity, ref Pathfinder pathfinder, DynamicBuffer<PathBuffer> pathBuffer)
            {
                // New Request Logic
                if (pathfinder.pathStatus == 0)
                {
                    if (pathfinder.queryIndex == -1)
                    {
                        // Debug.Log("shouldn't be pathfinding! (waiting for init Job)");
                        return;
                    }

                    // Debug.Log($"Returning index: {pathfinder.queryIndex} -- Entity: {entity.ToFixedString()}");

                    //"Give back" the navmeshQuery, and mark ourselves as invalid
                    availableIndices.Enqueue(pathfinder.queryIndex);
                    pathfinder.queryIndex = -1;
                    return;
                }

                if (pathfinder.queryIndex == -1)
                {
                    // Debug.Log("Not initialized yet!");
                    return;
                }

                // Debug.Log($"Getting query at index: {pathfinder.queryIndex}");
                NavMeshQuery query = queries[pathfinder.queryIndex];

                // Iterative Update
                if (pathfinder.pathStatus == PathQueryStatus.InProgress)
                {
                    pathfinder.pathStatus = query.UpdateFindPath(maxIterations, out int _);
                }

                // 3. Success & Funnel (Straight Path)
                if (pathfinder.pathStatus == PathQueryStatus.Success)
                {
                    pathfinder.pathStatus = query.EndFindPath(out int pathLength);

                    NativeArray<PolygonId> polygons = new(pathLength, Allocator.Temp);
                    query.GetPathResult(polygons);

                    PathQueryStatus status = PathQueryStatus.Success;
                    if (pathfinder.useFunnel)
                    {
                        //todo no +1
                        const int PathMaxSize = 512;
                        NativeArray<NavMeshLocation> straightPath = new(pathLength + 1, Allocator.Temp);
                        NativeArray<StraightPathFlags> straightPathFlags = new(PathMaxSize, Allocator.Temp);
                        NativeArray<float> vertexSide = new(PathMaxSize, Allocator.Temp);

                        int cornerCount = 0;

                        //todo profilermarker

                        status = FindStraightPath(
                            ref query,
                            pathfinder.fromLocation.position,
                            pathfinder.toLocation.position,
                            pathLength,
                            polygons,
                            ref straightPath,
                            ref straightPathFlags,
                            ref vertexSide,
                            ref cornerCount
                        );

                        pathBuffer.Clear();
                        if (status == PathQueryStatus.Failure)
                        {
                            // Debug.Log("Failure to find path!");
                            return;
                        }

                        for (int i = 0; i < straightPath.Length; i++)
                        {
                            bool3 equal = straightPath[i].position == new Vector3(0, 0, 0);
                            if (math.all(equal))
                            {
                                // Debug.Log("Corrupt Path part!");
                                continue;
                            }

                            pathBuffer.Add(new PathBuffer { position = straightPath[i].position });
                        }
                    }
                    else
                    {
                        pathBuffer.Add(new PathBuffer { position = pathfinder.fromLocation.position });

                        for (int i = 0; i < (pathLength - 1); i++)
                        {
                            // Get the shared edge (portal) between current polygon and next
                            query.GetPortalPoints(polygons[i], polygons[i + 1], out Vector3 left, out Vector3 right);

                            // Funnel Logic:
                            // This is where you would traditionally narrow your 'left' and 'right' 
                            // bounds. For a simple approximation, you can use the portal center:
                            float3 portalCenter = (left + right) * 0.5f;
                            bool3 equal = portalCenter == float3.zero;
                            if (math.all(equal))
                            {
                                // Debug.Log("Corrupt Path part!");
                                continue;
                            }

                            pathBuffer.Add(new PathBuffer { position = portalCenter });
                        }

                        pathBuffer.Add(new PathBuffer { position = pathfinder.toLocation.position });
                    }

                    // Reset status to allow the next logic / movement system to take over or to allow a re-path if the target moves again.
                    // pathfinder.pathStatus = 0; //todo ??
                    pathfinder.pathStatus = status; //todo ??

                    //"Give back" the navmeshQuery, and mark ourselves as invalid
                    // Debug.Log($"Returning index: {pathfinder.queryIndex} -- Entity: {entity.ToFixedString()}");
                    availableIndices.Enqueue(pathfinder.queryIndex);
                    pathfinder.queryIndex = -1;

                    ecb.SetComponentEnabled<Pathfinder>(JobsUtility.ThreadIndex, entity, false);
                }
            }

            private const int PathMaxSize = 512;

            private static PathQueryStatus FindStraightPath(
                ref NavMeshQuery query,
                float3 startPos,
                float3 endPos,
                int pathSize,
                NativeArray<PolygonId> polygons,
                ref NativeArray<NavMeshLocation> straightPath,
                ref NativeArray<StraightPathFlags> straightPathFlags,
                ref NativeArray<float> vertexSide,
                ref int straightPathCount)
            {
                if (!query.IsValid(polygons[0]))
                {
                    //Debug.Log("Query Failure 1");
                    straightPath[0] = new NavMeshLocation(); // empty terminator
                    return PathQueryStatus.Failure; // | PathQueryStatus.InvalidParam;
                }

                straightPath[0] = query.CreateLocation(startPos, polygons[0]);

                straightPathFlags[0] = StraightPathFlags.Start;

                int apexIndex = 0;
                int n = 1;
                if (pathSize > 1)
                {
                    Matrix4x4 startPolyWorldToLocal = query.PolygonWorldToLocalMatrix(polygons[0]);

                    float3 apex = startPolyWorldToLocal.MultiplyPoint(startPos);
                    // Vector3.zero accesses a static readonly which does not work in burst yet
                    float3 left = new(0, 0, 0);
                    float3 right = new(0, 0, 0);
                    int leftIndex = -1;
                    int rightIndex = -1;

                    for (int i = 1; i <= pathSize; ++i)
                    {
                        Matrix4x4 polyWorldToLocal = query.PolygonWorldToLocalMatrix(polygons[apexIndex]);

                        float3 vl, vr;
                        if (i == pathSize)
                        {
                            vl = vr = polyWorldToLocal.MultiplyPoint(endPos);
                        }
                        else
                        {
                            bool success = query.GetPortalPoints(polygons[i - 1], polygons[i], out Vector3 vecvl,
                                out Vector3 vecvr);
                            if (!success)
                            {
                                //Debug.Log("Query Failure 2");
                                return PathQueryStatus.Failure; // | PathQueryStatus.InvalidParam;
                            }

                            vl = polyWorldToLocal.MultiplyPoint(vecvl);
                            vr = polyWorldToLocal.MultiplyPoint(vecvr);
                        }

                        vl -= apex;
                        vr -= apex;

                        // Ensure left/right ordering
                        if (Perp2D(vl, vr) < 0)
                        {
                            Swap(ref vl, ref vr);
                        }

                        // Terminate funnel by turning
                        if (Perp2D(left, vr) < 0)
                        {
                            Matrix4x4 polyLocalToWorld = query.PolygonLocalToWorldMatrix(polygons[apexIndex]);
                            Vector3 termPos = polyLocalToWorld.MultiplyPoint(apex + left);

                            n = RetracePortals(ref query, apexIndex, leftIndex, n, termPos, polygons, ref straightPath,
                                ref straightPathFlags);
                            if (vertexSide.Length > 0)
                            {
                                vertexSide[n - 1] = -1;
                            }

                            //Debug.Log("LEFT");

                            if (n == PathMaxSize)
                            {
                                straightPathCount = n;
                                return PathQueryStatus.Success; // | PathQueryStatus.BufferTooSmall;
                            }

                            apex = polyWorldToLocal.MultiplyPoint(termPos);
                            left = float3.zero;
                            right = float3.zero;
                            i = apexIndex = leftIndex;
                            continue;
                        }

                        if (Perp2D(right, vl) > 0)
                        {
                            Matrix4x4 polyLocalToWorld = query.PolygonLocalToWorldMatrix(polygons[apexIndex]);
                            Vector3 termPos = polyLocalToWorld.MultiplyPoint(apex + right);

                            n = RetracePortals(ref query, apexIndex, rightIndex, n, termPos, polygons, ref straightPath,
                                ref straightPathFlags);
                            if (vertexSide.Length > 0)
                            {
                                vertexSide[n - 1] = 1;
                            }

                            //Debug.Log("RIGHT");

                            if (n == PathMaxSize)
                            {
                                straightPathCount = n;
                                return PathQueryStatus.Success; // | PathQueryStatus.BufferTooSmall;
                            }

                            apex = polyWorldToLocal.MultiplyPoint(termPos);
                            left = float3.zero;
                            right = float3.zero;
                            i = apexIndex = rightIndex;
                            continue;
                        }

                        // Narrow funnel
                        if (Perp2D(left, vl) >= 0)
                        {
                            left = vl;
                            leftIndex = i;
                        }

                        if (Perp2D(right, vr) <= 0)
                        {
                            right = vr;
                            rightIndex = i;
                        }
                    }
                }

                // Remove the next to last if duplicate point
                // - e.g., start and end positions are the same (in which case we have got a single point)
                if (n > 0 && math.all((float3)straightPath[n - 1].position == endPos))
                {
                    n--;
                }

                n = RetracePortals(ref query, apexIndex, pathSize - 1, n, endPos, polygons, ref straightPath,
                    ref straightPathFlags);
                if (vertexSide.Length > 0)
                {
                    vertexSide[n - 1] = 0;
                }

                if (n == PathMaxSize)
                {
                    straightPathCount = n;
                    return PathQueryStatus.Success; // | PathQueryStatus.BufferTooSmall;
                }

                // Fix flag for final path point
                straightPathFlags[n - 1] = StraightPathFlags.End;

                straightPathCount = n;
                return PathQueryStatus.Success;
            }

            private static int RetracePortals(
                ref NavMeshQuery query,
                int startIndex,
                int endIndex,
                int n,
                float3 termPos,
                NativeArray<PolygonId> path,
                ref NativeArray<NavMeshLocation> straightPath,
                ref NativeArray<StraightPathFlags> straightPathFlags)
            {
                for (int k = startIndex; k < (endIndex - 1); ++k)
                {
                    NavMeshPolyTypes type1 = query.GetPolygonType(path[k]);
                    NavMeshPolyTypes type2 = query.GetPolygonType(path[k + 1]);

                    if (type1 != type2)
                    {
                        query.GetPortalPoints(path[k], path[k + 1], out Vector3 vecl, out Vector3 vecr);

                        SegmentSegmentCpa(out float3 cpa1, out float3 _, vecl, vecr, straightPath[n - 1].position, termPos);
                        straightPath[n] = query.CreateLocation(cpa1, path[k + 1]);

                        straightPathFlags[n] = type2 == NavMeshPolyTypes.OffMeshConnection
                            ? StraightPathFlags.OffMeshConnection
                            : 0;
                        if (++n == PathMaxSize)
                        {
                            return PathMaxSize;
                        }
                    }
                }

                straightPath[n] = query.CreateLocation(termPos, path[endIndex]);
                straightPathFlags[n] = query.GetPolygonType(path[endIndex]) == NavMeshPolyTypes.OffMeshConnection
                    ? StraightPathFlags.OffMeshConnection
                    : 0;
                return ++n;
            }

            private static float Perp2D(float3 u, float3 v)
            {
                return (u.z * v.x) - (u.x * v.z);
            }

            private static void Swap(ref float3 a, ref float3 b)
            {
                (a, b) = (b, a);
            }

            private static void SegmentSegmentCpa(out float3 c0, out float3 c1, float3 p0, float3 p1, float3 q0, float3 q1)
            {
                float3 u = p1 - p0;
                float3 v = q1 - q0;
                float3 w0 = p0 - q0;

                float a = math.dot(u, u);
                float b = math.dot(u, v);
                float c = math.dot(v, v);
                float d = math.dot(u, w0);
                float e = math.dot(v, w0);

                float den = (a * c) - (b * b);
                float sc, tc;

                if (den == 0)
                {
                    sc = 0;
                    tc = d / b;

                    // todo: handle b = 0 (=> a and/or c is 0)
                }
                else
                {
                    sc = ((b * e) - (c * d)) / ((a * c) - (b * b));
                    tc = ((a * e) - (b * d)) / ((a * c) - (b * b));
                }

                c0 = math.lerp(p0, p1, sc);
                c1 = math.lerp(q0, q1, tc);
            }
        }

        [Flags]
        private enum StraightPathFlags
        {
            Start = 0x01, // The vertex is the start position.
            End = 0x02, // The vertex is the end position.
            OffMeshConnection = 0x04 // The vertex is start of an off-mesh link.
        }
    }
}