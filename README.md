# 3D Soft Body Physics Simulation in Unity (Verlet + PBD)

![Made with Unity]

This project is a high-performance 3D soft body physics simulation I developed from scratch in Unity. It uses **Verlet Integration** for stable time stepping and a **Position Based Dynamics (PBD)** solver loop to handle constraints, resulting in a robust and visually appealing jelly-like cube. The simulation is heavily optimized using Unity's **C# Job System** and **Burst Compiler**.

---

## 🚀 Key Features

* **Verlet Integration:** I implemented Verlet integration for the primary physics step. This provides numerically stable motion based on current and previous positions, avoiding issues common with force-based integrators.
* **Position Based Dynamics (PBD) Solver:** Constraints (stick lengths, collisions) are enforced iteratively within a solver loop (`stiffness` parameter). This allows for direct control over the object's rigidity/gelatinousness and ensures constraints are met even under stress.
* **Interactive Dragging:** You can click and drag any vertex of the cube using the mouse. The interaction feels natural and stable thanks to integrating the drag constraint directly into the PBD solver loop.
* **Basic Collision & Friction:** The simulation includes simple plane collision (ground) with basic friction applied to points in contact, making the object realistically slow down when sliding.
* **High Performance with Jobs & Burst:** The core physics logic (Verlet, Stick Solving, Collisions, Drag) is implemented using Unity's C# Job System and accelerated with the Burst Compiler. This parallelizes calculations across multiple CPU cores and generates highly optimized machine code, allowing the simulation to scale efficiently.
* **Dynamic Mesh Deformation:** The visual mesh (`MeshFilter`, `MeshRenderer`) and the physics collider (`MeshCollider`) are updated in real-time based on the simulated point positions. Normals and tangents are recalculated each frame for correct lighting.

## 🛠️ How It Works

1.  **Initialization (`Start`)**:
    * A cube structure is defined using 8 `Point` objects and 28 `Stick` constraints (edges, face diagonals, internal diagonals) in world space.
    * `NativeArray`s are created to hold physics data (`positions`, `oldPositions`, `stickData`) optimized for the Job System.
    * The initial `Mesh` is created.
2.  **Input Handling (`Update`)**: Detects mouse clicks via the Input System to determine which point (`currentlyDraggedPointIndex`) to drag.
3.  **Physics Simulation (`FixedUpdate` -> `SimulateWithJobs`)**:
    * **Verlet Step (Job):** Applies gravity and inertia to predict the next position for all points. Runs once per `FixedUpdate`.
    * **Solver Loop (Jobs):** Iterates `stiffness` times:
        * **Stick Solver (Job):** Enforces stick length constraints, pulling/pushing connected points.
        * **Collision Solver (Job):** Resolves collisions with the floor and applies friction.
        * **Drag Solver (Job):** Teleports the dragged point to the mouse target position (hard constraint).
    * Jobs are scheduled with dependencies and completed, ensuring calculations happen in the correct order across multiple cores.
4.  **Visual Update (`FixedUpdate` -> `UpdateMeshVisuals`)**:
    * Physics positions (world space `NativeArray<float3>`) are copied back.
    * Positions are converted to mesh vertex positions (local space `Vector3[]`).
    * The `Mesh` vertices, normals, and tangents are updated.
    * The `MeshCollider` is updated with the deformed mesh.

## ⚙️ Setup

1.  Ensure the **Burst** and **Collections** packages are installed via the Unity Package Manager.
2.  Create an empty GameObject and attach the `SoftBody.cs` script.
3.  Create a basic Material and assign it to the `Mesh Renderer` component.
4.  Create a Plane GameObject at `Y = -3.1` to act as the floor.
5.  Press Play! Adjust `Stiffness` in the inspector (while running) to control the gelatinousness. Click and drag the cube.
