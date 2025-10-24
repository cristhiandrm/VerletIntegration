using UnityEngine;

public class Point
{
    public Vector3 position;
    public Vector3 oldPosition;
    public bool isLocked;

    /// <summary>
    /// Constructor para un nuevo punto de simulaci�n.
    /// </summary>
    /// <param name="pos">Posici�n inicial del punto.</param>
    public Point(Vector3 pos)
    {
        position = pos;
        oldPosition = pos;
        isLocked = false;
    }
}