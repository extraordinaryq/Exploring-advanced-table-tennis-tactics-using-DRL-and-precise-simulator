using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Place on the racket's trigger zone (IsTrigger Collider).
/// Responsible for detecting "approaching racket face" and calling TTBall.ApplyRacketHit.
/// </summary>
[RequireComponent(typeof(Collider))]
[DisallowMultipleComponent]
public class RacketHitZone : MonoBehaviour
{
    [Header("Links")]
    public RacketController racket;   // Drag the controller on the Racket (parent object)

    [Header("Hit Conditions")]
    [Tooltip("Relative normal approach speed threshold (triggered if less than -threshold)")]
    public float approachSpeedThresh = 0.2f; // m/s
    [Tooltip("Cooldown time for the same ball to trigger again")]
    public float rehitCooldown = 0.06f; // s
    [Tooltip("After hit, slightly push the ball away along the normal to avoid continuous triggers")]
    public bool nudgeAfterHit = true;
    public float nudgeDistance = 0.005f; // 5mm

    [Header("Debug")]
    public bool drawGizmos = true;
    public Color zoneColor = new Color(0f, 1f, 0.5f, 0.25f);

    Collider col;
    readonly Dictionary<TTBall, float> lastHitTime = new Dictionary<TTBall, float>();

    void Reset()
    {
        // Try to automatically find the upper-level RacketController
        if (racket == null && transform.parent != null)
            racket = transform.parent.GetComponentInParent<RacketController>();
    }

    void Awake()
    {
        col = GetComponent<Collider>();
        col.isTrigger = true;
        if (racket == null && transform.parent != null)
            racket = transform.parent.GetComponentInParent<RacketController>();
        if (racket == null)
            Debug.LogWarning("[RacketHitZone] RacketController not found, please bind it in the Inspector.");
    }

    void OnTriggerStay(Collider other)
    {
        var ball = other.attachedRigidbody
            ? other.attachedRigidbody.GetComponent<TTBall>()
            : other.GetComponent<TTBall>();

        if (ball == null || racket == null) return;

        // Cooldown debounce
        float now = Time.time;
        if (lastHitTime.TryGetValue(ball, out float tLast) && (now - tLast) < rehitCooldown)
            return;

        // Calculate the relative velocity component along the racket face normal
        Vector3 n = racket.FaceNormalWorld.normalized;           // Racket face normal (world)
        TTBall.RacketParams rp = racket.BuildParams();           // vR, ¦Á, ¦Â, kv/kw/er
        Vector3 vRel = ball.Velocity - rp.vR;                    // Ball velocity relative to racket
        float approachAlongN = Vector3.Dot(vRel, n);             // <0 means towards the racket face

        // Condition: approaching racket face (along normal) with speed above threshold
        if (approachAlongN < -approachSpeedThresh)
        {
            // Trigger a hit
            ball.ApplyRacketHit(rp);
            lastHitTime[ball] = now;

            // After hit, push slightly to avoid continuous triggers (optional)
            if (nudgeAfterHit)
            {
                Vector3 p = ball.Position + n * nudgeDistance;
                ball.ResetState(p, ball.Velocity, ball.Omega);
            }
        }
    }

    void OnDrawGizmos()
    {
        if (!drawGizmos) return;
        var c = GetComponent<Collider>();
        if (c == null) return;
        Gizmos.color = zoneColor;
        var m = c.transform.localToWorldMatrix;

        // Simple drawing: only draw bounding box (enough even for Capsule/Sphere)
        if (c is BoxCollider b)
        {
            Matrix4x4 old = Gizmos.matrix;
            Gizmos.matrix = transform.localToWorldMatrix;
            Gizmos.DrawCube(b.center, b.size);
            Gizmos.matrix = old;
        }
        else if (c is SphereCollider s)
        {
            Gizmos.DrawSphere(s.bounds.center, s.radius * Mathf.Max(transform.lossyScale.x, Mathf.Max(transform.lossyScale.y, transform.lossyScale.z)));
        }
        else
        {
            // In other cases, draw bounds
            Gizmos.DrawCube(c.bounds.center, c.bounds.size);
        }

        // Draw racket face normal
        if (racket != null)
        {
            Vector3 o = (racket.faceOrigin ? racket.faceOrigin.position : racket.transform.position);
            Vector3 n = racket.FaceNormalWorld.normalized;
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(o, o + n * 0.25f);
        }
    }
}
