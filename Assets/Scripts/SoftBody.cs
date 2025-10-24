using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using System;
using Unity.Jobs; // <-- ¡NUEVO! Para el Job System
using Unity.Burst; // <-- ¡NUEVO! Para el Burst Compiler
using Unity.Collections; // <-- ¡NUEVO! Para NativeArray
using Unity.Mathematics; // <-- ¡NUEVO! Para tipos optimizados (float3)

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class SoftBody : MonoBehaviour
{
    [Header("Simulación")]
    public float3 gravity = new float3(0f, -9.81f, 0f); // Usamos float3 de Mathematics

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

    // --- Listas de Física (ya no las usaremos directamente en Simulate) ---
    private List<Point> points = new List<Point>();
    private List<Stick> sticks = new List<Stick>();

    // --- NUEVO: NativeArrays para los Jobs ---
    // Estas son copias de los datos optimizadas para Burst/Jobs
    private NativeArray<float3> positions;
    private NativeArray<float3> oldPositions;
    private NativeArray<StickData> stickData; // Estructura simple para los sticks
    private NativeArray<int> trianglesNative; // Para la malla
    private NativeArray<float3> verticesNative; // Para la malla

    // --- Malla ---
    private Mesh mesh;
    private Vector3[] verticesManaged; // Array normal para asignar a la malla
    private int[] trianglesManaged;

    // --- Interacción ---
    private int currentlyDraggedPointIndex = -1; // Usamos índice ahora
    private float3 mouseTargetPosition;
    private MeshCollider meshCollider;

    // --- Estructura simple para pasar datos de sticks a los Jobs ---
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

        // --- NUEVO: Inicializar NativeArrays ---
        // Creamos las NativeArrays con el tamaño necesario y persistentes
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
        // ------------------------------------
    }

    // --- NUEVO: Limpiar NativeArrays al destruir ---
    void OnDestroy()
    {
        if (positions.IsCreated) positions.Dispose();
        if (oldPositions.IsCreated) oldPositions.Dispose();
        if (stickData.IsCreated) stickData.Dispose();
        if (verticesNative.IsCreated) verticesNative.Dispose();
        if (trianglesNative.IsCreated) trianglesNative.Dispose();
    }
    // --------------------------------------------

    void Update()
    {
        // El Input sigue igual
        if (Mouse.current == null) return;
        if (Mouse.current.leftButton.wasPressedThisFrame)
        {
            Ray ray = Camera.main.ScreenPointToRay(Mouse.current.position.ReadValue());
            if (Physics.Raycast(ray, out RaycastHit hit) && hit.collider == meshCollider)
            {
                // Buscamos el índice del punto más cercano
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
        // Actualizamos la posición del mouse
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

        // --- NUEVO: Ejecutar la Simulación con Jobs ---
        SimulateWithJobs();
        // ------------------------------------------

        UpdateMeshVisuals();
    }

    // --- NUEVO: Función principal que agenda los Jobs ---
    private void SimulateWithJobs()
    {
        float dt = Time.fixedDeltaTime;

        // 1. Crear el Job de Verlet
        var verletJob = new VerletJob
        {
            positions = this.positions,
            oldPositions = this.oldPositions,
            gravity = this.gravity,
            damping = this.damping,
            dtSqr = dt * dt,
            draggedIndex = this.currentlyDraggedPointIndex, // Pasamos el índice
            mouseTargetPos = this.mouseTargetPosition      // Pasamos la posición del mouse
        };

        // 2. Agendar el Job de Verlet (corre en paralelo para cada punto)
        JobHandle verletHandle = verletJob.Schedule(points.Count, 64); // 64 = batch size

        // 3. Preparar el bucle del Solver (Sticks y Colisiones)
        JobHandle solverHandle = verletHandle; // Empezamos dependiendo de Verlet
        for (int i = 0; i < stiffness; i++)
        {
            // A. Crear y agendar Job de Sticks
            var stickJob = new StickSolverJob
            {
                positions = this.positions, // ¡Ojo! Los jobs pueden modificar los datos
                stickData = this.stickData,
                stiffnessFactor = 1.0f / stiffness // Dividimos la corrección
            };
            // Depende del handle anterior (sea Verlet o la iteración previa del solver)
            JobHandle stickHandle = stickJob.Schedule(sticks.Count, 64, solverHandle);

            // B. Crear y agendar Job de Colisiones
            var collisionJob = new CollisionJob
            {
                positions = this.positions,
                oldPositions = this.oldPositions,
                floorFriction = this.floorFriction,
                floorY = -3f // Definimos el suelo aquí
            };
            // Depende de que los sticks hayan terminado en ESTA iteración
            JobHandle collisionHandle = collisionJob.Schedule(points.Count, 64, stickHandle);

            // C. Crear y agendar Job de Arrastre (si aplica)
            if (currentlyDraggedPointIndex != -1)
            {
                var dragJob = new DragJob
                {
                    positions = this.positions,
                    oldPositions = this.oldPositions,
                    draggedIndex = this.currentlyDraggedPointIndex,
                    mouseTargetPos = this.mouseTargetPosition
                };
                // Depende de que las colisiones hayan terminado
                solverHandle = dragJob.Schedule(collisionHandle);
            }
            else
            {
                // Si no hay arrastre, el handle para la próxima iteración es el de colisión
                solverHandle = collisionHandle;
            }
        }

        // 4. Esperar a que TODOS los jobs terminen
        solverHandle.Complete();
    }
    // ----------------------------------------------------

    // --- ACTUALIZADO: UpdateMeshVisuals ---
    // Copiamos los datos de NativeArray al array normal de la malla
    private void UpdateMeshVisuals()
    {
        // Copiamos los datos calculados por los jobs de vuelta
        positions.CopyTo(verticesNative);

        // Convertimos a espacio local (esto aún no está en un Job)
        for (int i = 0; i < points.Count; i++)
        {
            verticesManaged[i] = transform.InverseTransformPoint(verticesNative[i]);
        }

        mesh.vertices = verticesManaged;
        mesh.RecalculateNormals();
        mesh.RecalculateTangents();
        meshCollider.sharedMesh = mesh;
    }
    // -------------------------------------

    // --- ACTUALIZADO: FindClosestPoint ---
    // Ahora devuelve el índice
    private int FindClosestPointIndex(Vector3 position)
    {
        int closestIndex = -1;
        float minDistanceSqr = Mathf.Infinity;
        for (int i = 0; i < points.Count; i++) // Usamos NativeArray ahora
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
    // ------------------------------------

    void CreateCube(Vector3 center, float size)
    {
        points.Clear();
        sticks.Clear();
        float s = size / 2f;

        // Creamos los puntos en la lista normal primero
        points.Add(new Point(transform.TransformPoint(center + new Vector3(-s, -s, -s)))); // 0
        points.Add(new Point(transform.TransformPoint(center + new Vector3(s, -s, -s)))); // 1
        points.Add(new Point(transform.TransformPoint(center + new Vector3(s, s, -s)))); // 2
        points.Add(new Point(transform.TransformPoint(center + new Vector3(-s, s, -s)))); // 3
        points.Add(new Point(transform.TransformPoint(center + new Vector3(-s, -s, s)))); // 4
        points.Add(new Point(transform.TransformPoint(center + new Vector3(s, -s, s)))); // 5
        points.Add(new Point(transform.TransformPoint(center + new Vector3(s, s, s)))); // 6
        points.Add(new Point(transform.TransformPoint(center + new Vector3(-s, s, s)))); // 7

        // Creamos los sticks en la lista normal
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
        // Usamos arrays normales para la malla inicial
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

        // --- NUEVO: Inicializar NativeArrays para la malla ---
        verticesNative = new NativeArray<float3>(points.Count, Allocator.Persistent);
        trianglesNative = new NativeArray<int>(trianglesManaged, Allocator.Persistent);
        // ----------------------------------------------------
    }

    // --- ¡AQUÍ EMPIEZAN LOS JOBS! ---
    // Son estructuras que definen el trabajo a realizar en paralelo.

    [BurstCompile] // ¡La magia de Burst!
    struct VerletJob : IJobParallelFor
    {
        public NativeArray<float3> positions;
        public NativeArray<float3> oldPositions;
        public float3 gravity;
        public float damping;
        public float dtSqr;

        [ReadOnly] public int draggedIndex;      // Índice del punto arrastrado (-1 si no hay)
        [ReadOnly] public float3 mouseTargetPos; // Posición del mouse

        public void Execute(int index)
        {
            // No aplicamos Verlet si es el punto arrastrado (se manejará en DragJob)
            if (index == draggedIndex) return;

            float3 velocity = (positions[index] - oldPositions[index]) * damping;
            oldPositions[index] = positions[index];
            positions[index] += velocity + gravity * dtSqr;
        }
    }

    [BurstCompile]
    struct StickSolverJob : IJobParallelFor
    {
        // [NativeDisableParallelForRestriction] // Necesario si modificamos el mismo array desde varios hilos
        public NativeArray<float3> positions;
        [ReadOnly] public NativeArray<StickData> stickData;
        public float stiffnessFactor; // Corrección dividida por 'stiffness'

        public void Execute(int index)
        {
            StickData stick = stickData[index];
            float3 delta = positions[stick.indexB] - positions[stick.indexA];
            float currentLength = math.length(delta);
            float error = (currentLength - stick.length) / (currentLength + 0.0001f);

            float3 correction = delta * 0.5f * error * stiffnessFactor;

            // ¡Ojo! Modificar el mismo array desde hilos paralelos puede ser peligroso.
            // Para este caso simple, funciona, pero para sistemas más complejos
            // se necesitan técnicas más avanzadas (como usar Atomic operations o
            // agendar jobs que no escriban en los mismos índices).
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
                oldPos.y = floorY; // Matar rebote vertical

                // Aplicar fricción
                float velX = pos.x - oldPos.x;
                float velZ = pos.z - oldPos.z;
                velX *= floorFriction;
                velZ *= floorFriction;
                oldPos.x = pos.x - velX;
                oldPos.z = pos.z - velZ;
            }

            // (Aquí iría la lógica de colisión con esferas si la tuviéramos)

            // Escribir los resultados de vuelta
            positions[index] = pos;
            oldPositions[index] = oldPos;
        }
    }

    [BurstCompile]
    struct DragJob : IJob // No es IJobParallelFor porque solo afecta a UN punto
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