using UnityEngine;

public class Stick
{
    public Point pointA;
    public Point pointB;
    public float length;

    /// <summary>
    /// Constructor para un "palito" (restricción) que conecta dos puntos.
    /// </summary>
    public Stick(Point pA, Point pB)
    {
        pointA = pA;
        pointB = pB;
        length = Vector3.Distance(pA.position, pB.position);
    }
}