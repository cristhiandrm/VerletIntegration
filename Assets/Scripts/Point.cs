using UnityEngine;

public class Point
{
    public Vector3 position;
    public Vector3 oldPosition;
    public bool isLocked;

    /// <summary>
    /// Constructor para un nuevo punto de simulación.
    /// </summary>
    /// <param name="pos">Posición inicial del punto.</param>
    public Point(Vector3 pos)
    {
        position = pos;
        oldPosition = pos;
        isLocked = false;
    }
}