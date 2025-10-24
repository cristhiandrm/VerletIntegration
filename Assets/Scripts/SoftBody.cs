using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using System;
using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class SoftBody : MonoBehaviour
{
    [Header("Simulación")]
    public float3 gravity = new float3(0f, -9.81f, 0f);

    [Tooltip("Iteraciones del Solver. Bajo = Gelatinoso, Alto = Rígido.")]
    [Range(1, 30)]
    public int stiffness = 10;

    [Range(0.8f, 1.0f)]
    public float damping = 0.99f;

    [Header("Generación Cubo")]
    public float cubeSize = 2f;

    [Header("Interacción")]
    [Range(0f, 1f)]
    public float floorFriction = 0.95f;

    private List<Point> points = new List<Point>();
    private List<Stick> sticks = new List<Stick>();

    private NativeArray<float3> positions;
    private NativeArray<float3> oldPositions;
    private NativeArray<StickData> stickData; 
    private NativeArray<int> trianglesNative; 
    private NativeArray<float3> verticesNative; 

    // --- Mesh---
    private Mesh mesh;
    private Vector3[] verticesManaged; 
    private int[] trianglesManaged;

    // --- Interacción ---
    private int currentlyDraggedPointIndex = -1;
    private float3 mouseTargetPosition;
    private MeshCollider meshCollider;
    private struct StickData
    {
        public int indexA;
        public int indexB;
        public float length;
    }

    void Start()
    {
        mesh = new Mesh();
        GetComponent<MeshFilter>().mesh = mesh;
        meshCollider = GetComponent<MeshCollider>();
        if (meshCollider == null)
        {
            meshCollider = gameObject.AddComponent<MeshCollider>();
        }

        CreateCube(Vector3.zero, cubeSize);
        InitializeMesh();

        positions = new NativeArray<float3>(points.Count, Allocator.Persistent);
        oldPositions = new NativeArray<float3>(points.Count, Allocator.Persistent);
        for (int i = 0; i < points.Count; i++)
        {
            positions[i] = points[i].position;
            oldPositions[i] = points[i].oldPosition;
        }

        stickData = new NativeArray<StickData>(sticks.Count, Allocator.Persistent);
        for (int i = 0; i < sticks.Count; i++)
        {
            stickData[i] = new StickData
            {
                indexA = points.IndexOf(sticks[i].pointA),
                indexB = points.IndexOf(sticks[i].pointB),
                length = sticks[i].length
            };
        }
    }


    void OnDestroy()
    {
        if (positions.IsCreated) positions.Dispose();
        if (oldPositions.IsCreated) oldPositions.Dispose();
        if (stickData.IsCreated) stickData.Dispose();
        if (verticesNative.IsCreated) verticesNative.Dispose();
        if (trianglesNative.IsCreated) trianglesNative.Dispose();
    }

    void Update()
    {
        if (Mouse.current == null) return;
        if (Mouse.current.leftButton.wasPressedThisFrame)
        {
            Ray ray = Camera.main.ScreenPointToRay(Mouse.current.position.ReadValue());
            if (Physics.Raycast(ray, out RaycastHit hit) && hit.collider == meshCollider)
            {
                currentlyDraggedPointIndex = FindClosestPointIndex(hit.point);
            }
        }
        if (Mouse.current.leftButton.wasReleasedThisFrame)
        {
            currentlyDraggedPointIndex = -1;
        }
    }

    void FixedUpdate()
    {
        if (currentlyDraggedPointIndex != -1)
        {
            if (Mouse.current == null) return;
            Plane plane = new Plane(Camera.main.transform.forward, positions[currentlyDraggedPointIndex]);
            Ray ray = Camera.main.ScreenPointToRay(Mouse.current.position.ReadValue());
            if (plane.Raycast(ray, out float distance))
            {
                mouseTargetPosition = ray.GetPoint(distance);
            }
        }

        SimulateWithJobs();

        UpdateMeshVisuals();
    }

    private void SimulateWithJobs()
    {
        float dt = Time.fixedDeltaTime;


        var verletJob = new VerletJob
        {
            positions = this.positions,
            oldPositions = this.oldPositions,
            gravity = this.gravity,
            damping = this.damping,
            dtSqr = dt * dt,
            draggedIndex = this.currentlyDraggedPointIndex, 
            mouseTargetPos = this.mouseTargetPosition     
        };

        JobHandle verletHandle = verletJob.Schedule(points.Count, 64); // 64 = batch size

        JobHandle solverHandle = verletHandle;
        for (int i = 0; i < stiffness; i++)
        {
            var stickJob = new StickSolverJob
            {
                positions = this.positions,
                stickData = this.stickData,
                stiffnessFactor = 1.0f / stiffness
            };
            JobHandle stickHandle = stickJob.Schedule(sticks.Count, 64, solverHandle);

            var collisionJob = new CollisionJob
            {
                positions = this.positions,
                oldPositions = this.oldPositions,
                floorFriction = this.floorFriction,
                floorY = -3f
            };
            JobHandle collisionHandle = collisionJob.Schedule(points.Count, 64, stickHandle);
            if (currentlyDraggedPointIndex != -1)
            {
                var dragJob = new DragJob
                {
                    positions = this.positions,
                    oldPositions = this.oldPositions,
                    draggedIndex = this.currentlyDraggedPointIndex,
                    mouseTargetPos = this.mouseTargetPosition
                };
                solverHandle = dragJob.Schedule(collisionHandle);
            }
            else
            {
                solverHandle = collisionHandle;
            }
        }
        solverHandle.Complete();
    }
    private void UpdateMeshVisuals()
    {
        positions.CopyTo(verticesNative);
        for (int i = 0; i < points.Count; i++)
        {
            verticesManaged[i] = transform.InverseTransformPoint(verticesNative[i]);
        }

        mesh.vertices = verticesManaged;
        mesh.RecalculateNormals();
        mesh.RecalculateTangents();
        meshCollider.sharedMesh = mesh;
    }
    
    private int FindClosestPointIndex(Vector3 position)
    {
        int closestIndex = -1;
        float minDistanceSqr = Mathf.Infinity;
        for (int i = 0; i < points.Count; i++)
        {
            float distSqr = math.lengthsq((float3)position - positions[i]);
            if (distSqr < minDistanceSqr)
            {
                minDistanceSqr = distSqr;
                closestIndex = i;
            }
        }
        return closestIndex;
    }

    void CreateCube(Vector3 center, float size)
    {
        points.Clear();
        sticks.Clear();
        float s = size / 2f;

        points.Add(new Point(transform.TransformPoint(center + new Vector3(-s, -s, -s)))); // 0
        points.Add(new Point(transform.TransformPoint(center + new Vector3(s, -s, -s)))); // 1
        points.Add(new Point(transform.TransformPoint(center + new Vector3(s, s, -s)))); // 2
        points.Add(new Point(transform.TransformPoint(center + new Vector3(-s, s, -s)))); // 3
        points.Add(new Point(transform.TransformPoint(center + new Vector3(-s, -s, s)))); // 4
        points.Add(new Point(transform.TransformPoint(center + new Vector3(s, -s, s)))); // 5
        points.Add(new Point(transform.TransformPoint(center + new Vector3(s, s, s)))); // 6
        points.Add(new Point(transform.TransformPoint(center + new Vector3(-s, s, s)))); // 7

        sticks.Add(new Stick(points[0], points[1])); sticks.Add(new Stick(points[1], points[2])); sticks.Add(new Stick(points[2], points[3])); sticks.Add(new Stick(points[3], points[0]));
        sticks.Add(new Stick(points[4], points[5])); sticks.Add(new Stick(points[5], points[6])); sticks.Add(new Stick(points[6], points[7])); sticks.Add(new Stick(points[7], points[4]));
        sticks.Add(new Stick(points[0], points[4])); sticks.Add(new Stick(points[1], points[5])); sticks.Add(new Stick(points[2], points[6])); sticks.Add(new Stick(points[3], points[7]));
        sticks.Add(new Stick(points[0], points[2])); sticks.Add(new Stick(points[1], points[3])); sticks.Add(new Stick(points[4], points[6])); sticks.Add(new Stick(points[5], points[7]));
        sticks.Add(new Stick(points[0], points[5])); sticks.Add(new Stick(points[1], points[4])); sticks.Add(new Stick(points[2], points[7])); sticks.Add(new Stick(points[3], points[6]));
        sticks.Add(new Stick(points[0], points[7])); sticks.Add(new Stick(points[3], points[4])); sticks.Add(new Stick(points[1], points[6])); sticks.Add(new Stick(points[2], points[5]));
        sticks.Add(new Stick(points[0], points[6])); sticks.Add(new Stick(points[1], points[7])); sticks.Add(new Stick(points[2], points[4])); sticks.Add(new Stick(points[3], points[5]));
    }

    void InitializeMesh()
    {
        verticesManaged = new Vector3[points.Count];
        for (int i = 0; i < points.Count; i++)
        {
            verticesManaged[i] = transform.InverseTransformPoint(points[i].position);
        }

        trianglesManaged = new int[] {
            0, 2, 1, 0, 3, 2, 1, 2, 6, 1, 6, 5, 5, 6, 7, 5, 7, 4,
            4, 7, 3, 4, 3, 0, 3, 7, 6, 3, 6, 2, 4, 0, 1, 4, 1, 5
        };
        mesh.vertices = verticesManaged;
        mesh.triangles = trianglesManaged;
        mesh.RecalculateNormals();
        mesh.RecalculateTangents();

        verticesNative = new NativeArray<float3>(points.Count, Allocator.Persistent);
        trianglesNative = new NativeArray<int>(trianglesManaged, Allocator.Persistent);
    }


    [BurstCompile] 
    struct VerletJob : IJobParallelFor
    {
        public NativeArray<float3> positions;
        public NativeArray<float3> oldPositions;
        public float3 gravity;
        public float damping;
        public float dtSqr;

        [ReadOnly] public int draggedIndex;      
        [ReadOnly] public float3 mouseTargetPos; 

        public void Execute(int index)
        {
            if (index == draggedIndex) return;

            float3 velocity = (positions[index] - oldPositions[index]) * damping;
            oldPositions[index] = positions[index];
            positions[index] += velocity + gravity * dtSqr;
        }
    }

    [BurstCompile]
    struct StickSolverJob : IJobParallelFor
    {
        public NativeArray<float3> positions;
        [ReadOnly] public NativeArray<StickData> stickData;
        public float stiffnessFactor;

        public void Execute(int index)
        {
            StickData stick = stickData[index];
            float3 delta = positions[stick.indexB] - positions[stick.indexA];
            float currentLength = math.length(delta);
            float error = (currentLength - stick.length) / (currentLength + 0.0001f);

            float3 correction = delta * 0.5f * error * stiffnessFactor;

            positions[stick.indexA] += correction;
            positions[stick.indexB] -= correction;
        }
    }

    [BurstCompile]
    struct CollisionJob : IJobParallelFor
    {
        public NativeArray<float3> positions;
        public NativeArray<float3> oldPositions;
        public float floorFriction;
        public float floorY;

        public void Execute(int index)
        {
            float3 pos = positions[index];
            float3 oldPos = oldPositions[index];

            // Suelo
            if (pos.y < floorY)
            {
                pos.y = floorY;
                oldPos.y = floorY;

                float velX = pos.x - oldPos.x;
                float velZ = pos.z - oldPos.z;
                velX *= floorFriction;
                velZ *= floorFriction;
                oldPos.x = pos.x - velX;
                oldPos.z = pos.z - velZ;
            }

            positions[index] = pos;
            oldPositions[index] = oldPos;
        }
    }

    [BurstCompile]
    struct DragJob : IJob 
    {
        public NativeArray<float3> positions;
        public NativeArray<float3> oldPositions;
        public int draggedIndex;
        public float3 mouseTargetPos;

        public void Execute()
        {
            if (draggedIndex != -1)
            {
                positions[draggedIndex] = mouseTargetPos;
                oldPositions[draggedIndex] = mouseTargetPos;
            }
        }
    }
}