using UnityEngine;

/// <summary>
/// Compute racket parameters (velocity vR, two angles ¦Á/¦Â, coefficients kv/kw/er)
/// Consistent with TTBall.ApplyRacketHit where RR = Ry(¦Â)*Rx(¦Á).
/// </summary>
[DisallowMultipleComponent]
public class RacketController : MonoBehaviour
{
    [Header("Racket Physical Coeffs")]
    [Range(0f, 1f)] public float kv = 0.10f;  // Tangential restitution
    [Range(0f, 1f)] public float kw = 0.05f;  // Rotation-linear coupling
    [Range(0f, 1f)] public float er = 0.85f;  // Normal restitution

    public enum FaceAxis { Forward, Back, Up, Down, Right, Left }

    [Header("Face Orientation")]
    public FaceAxis faceNormalAxis = FaceAxis.Forward; // Which local axis to use as racket face normal
    public Transform faceOrigin;                        // Racket face reference point (default is this object)

    [Header("Velocity")]
    public bool preferRigidbodyVelocity = true; // Prefer using Rigidbody velocity
    public bool smoothVelocity = true;
    [Range(0f, 1f)] public float velLerp = 0.2f; // Velocity smoothing

    [Header("Debug")]
    public bool drawGizmos = true;
    public float gizmoLen = 0.25f;

    // ---- Runtime ----
    Rigidbody rb;
    Vector3 lastPos;
    Vector3 vR;        // Racket world linear velocity
    float alphaX;      // Rotation around X-axis
    float betaY;       // Rotation around Y-axis

    public Vector3 VR => vR;
    public float Alpha => alphaX;
    public float Beta => betaY;
    public Vector3 FaceNormalWorld => GetAxisWorld(faceNormalAxis);

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        if (faceOrigin == null) faceOrigin = transform;
        lastPos = transform.position;
    }

    void FixedUpdate()
    {
        // 1) Velocity
        Vector3 rawVel;
        if (preferRigidbodyVelocity && rb != null)
            rawVel = rb.velocity;
        else
            rawVel = (transform.position - lastPos) / Mathf.Max(Time.fixedDeltaTime, 1e-6f);

        vR = smoothVelocity ? Vector3.Lerp(vR, rawVel, velLerp) : rawVel;
        lastPos = transform.position;

        // 2) Orientation ¡ú (¦Á, ¦Â)
        Vector3 n = FaceNormalWorld.normalized;

        alphaX = -Mathf.Asin(Mathf.Clamp(n.y, -1f, 1f));
        betaY = Mathf.Atan2(n.x, Mathf.Max(n.z, 1e-9f));
    }

    public TTBall.RacketParams BuildParams()
    {
        return new TTBall.RacketParams
        {
            kv = this.kv,
            kw = this.kw,
            er = this.er,
            vR = this.vR,
            alphaYaw = this.alphaX, // Name kept consistent with TTBall signature: ¦Á = rotation around X
            betaPitch = this.betaY  // ¦Â = rotation around Y
        };
    }

    Vector3 GetAxisWorld(FaceAxis axis)
    {
        switch (axis)
        {
            case FaceAxis.Forward: return transform.forward;
            case FaceAxis.Back: return -transform.forward;
            case FaceAxis.Up: return transform.up;
            case FaceAxis.Down: return -transform.up;
            case FaceAxis.Right: return transform.right;
            case FaceAxis.Left: return -transform.right;
        }
        return transform.forward;
    }

    void OnDrawGizmos()
    {
        if (!drawGizmos) return;
        Transform t = faceOrigin ? faceOrigin : transform;

        Gizmos.color = Color.yellow;
        Vector3 n = GetAxisWorld(faceNormalAxis).normalized;
        Gizmos.DrawLine(t.position, t.position + n * gizmoLen);
    }
}
