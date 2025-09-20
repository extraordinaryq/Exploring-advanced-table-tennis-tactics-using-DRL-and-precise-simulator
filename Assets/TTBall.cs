using UnityEngine;

/// <summary>
/// TTBall: Custom dynamics-based ping-pong ball (Unity: y-up)
/// 1) Flight: gravity + drag + Magnus (optional RK4)
/// 2) Table collision: sliding/rolling bifurcation matrix
/// 3) Racket hit: inelastic + tangential coupling (for external HitZone call)
/// </summary>
[RequireComponent(typeof(Transform))]
[DisallowMultipleComponent]
// Inside TTBall class, add these two lines in the field region


public class TTBall : MonoBehaviour
{
    // ---------- Inspector: Physical constants ----------
    [Header("Ball & Air (SI Units)")]
    [SerializeField] float m = 0.0027f;              // kg
    [SerializeField] float r = 0.02f;                // m
    [SerializeField] float rho = 1.29f;              // kg/m^3
    [SerializeField] float CD = 0.40f;               // drag coeff
    [SerializeField] float CM = 0.60f;               // lift coeff (Magnus)
    [SerializeField] float g = 9.8f;                // m/s^2

    // ---------- Inspector: Initial state ----------
    [Header("Initial State")]
    [SerializeField] Vector3 initPos = new Vector3(-0.58f, 0.694f, 0);
    [SerializeField] Vector3 initVel = new Vector3(0f, 0, 0.2f);
    [SerializeField] Vector3 initOmega = new Vector3(0, 0, 0); // rad/s

    // ---------- Inspector: Table ----------
    [Header("Table (y-up)")]
    [SerializeField] float tableY = 0f;  // Top surface world y
    [SerializeField, Range(0f, 1f)] float muTable = 0.22f;
    [SerializeField, Range(0f, 1f)] float enTable = 0.90f;

    // ---------- Inspector: Numerical integration ----------
    [Header("Integration & Damping")]
    [SerializeField, Range(1, 8)] int substeps = 2;
    [SerializeField, Tooltip("Spin exponential decay coefficient c_omega, unit s^-1")]
    float spinDamping = 0.0f; // e.g. 0.15f slight decay
    [SerializeField, Tooltip("Hard velocity cap, avoid numerical blow-up (0 = no limit)")]
    float maxSpeed = 0f;
    [SerializeField] bool useRK4 = true;

    // ---------- Inspector: Debug ----------
    [Header("Debug")]
    [SerializeField] bool drawGizmos = true;
    [SerializeField] float gizmoScale = 0.1f;

    [Header("Safety Limits")]
    [SerializeField] float vmax = 18f;      // Hard velocity cap (m/s), 18~22 is stable
    [SerializeField] float omegamax = 250f; // Spin cap (rad/s), reasonable initial value
    [SerializeField] float minBounceVy = 0.05f; // Minimum upward velocity after bounce, prevent "sticking jitter"

    // ---------- Derived constants ----------
    float A;       // ¦Ð r^2
    float kd;      // 0.5*CD*rho*A / m
    float km;      // 0.5*CM*rho*A*r / m

    // ---------- Runtime state ----------
    Vector3 pos, vel, omega;
    Rigidbody rb; // Optional; if exists, recommended isKinematic=true

    // Public read-only access
    public Vector3 Position => pos;
    public Vector3 Velocity => vel;
    public Vector3 Omega => omega;
    public float Radius => r;
    public float Mass => m;
    public event System.Action<Vector3, Vector3> TableBounced;                 // (position, velocity)
    public event System.Action<Vector3, Vector3, Vector3> RacketHitFired;      // (position, velocity, omega)

    // ========== Lifecycle ==========
    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.useGravity = false;
            rb.isKinematic = true; // Driven by script
        }
        RecalcDerived();
    }

    void Start()
    {
        ResetState(initPos, initVel, initOmega);
    }

    void OnValidate()
    {
        // Update derived constants when Inspector changes
        if (m <= 0f) m = 0.0027f;
        if (r <= 0f) r = 0.02f;
        RecalcDerived();
    }

    void RecalcDerived()
    {
        A = Mathf.PI * r * r;
        kd = 0.5f * CD * rho * A / Mathf.Max(m, 1e-9f);
        km = 0.5f * CM * rho * A * r / Mathf.Max(m, 1e-9f);
    }

    public void ResetState(Vector3 p, Vector3 v, Vector3 w)
    {
        pos = p; vel = v; omega = w;
        transform.position = pos;
    }

    // ========== main loop ==========
    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        int n = Mathf.Max(1, substeps);
        float h = dt / n;

        for (int i = 0; i < n; i++)
        {
            // 1) Flight step
            if (useRK4) RK4Step(ref pos, ref vel, omega, h);
            else EulerStep(ref pos, ref vel, omega, h);

            // 2) Table collision
            if (pos.y <= tableY + r && vel.y < 0f)
            {
                pos.y = tableY + r; // Return to contact position
                TableBounce(ref vel, ref omega);
            }

            // 3) Spin damping (optional)
            if (spinDamping > 0f)
            {
                float k = Mathf.Exp(-spinDamping * h);
                omega *= k;
            }

            // 4) Speed limit (optional)
            if (maxSpeed > 0f && vel.sqrMagnitude > maxSpeed * maxSpeed)
            {
                vel = vel.normalized * maxSpeed;
            }

            if (omegamax > 0f)
            {
                float w = omega.magnitude;
                if (w > omegamax) omega *= (omegamax / w);
            }
            if (vmax > 0f)
            {
                float s = vel.magnitude;
                if (s > vmax) vel *= (vmax / s);
            }
        }

        // Update Transform/Rigidbody
        transform.position = pos;
        if (rb != null && rb.isKinematic) rb.MovePosition(pos);
    }

    // ========== Dynamics ==========
    Vector3 Acc(Vector3 v, Vector3 w)
    {
        float speed = v.magnitude;
        Vector3 drag = -kd * speed * v;
        Vector3 magnus = km * Vector3.Cross(w, v);
        Vector3 gravity = new Vector3(0f, -g, 0f);
        return drag + magnus + gravity;
    }

    void EulerStep(ref Vector3 p, ref Vector3 v, Vector3 w, float h)
    {
        Vector3 a = Acc(v, w);
        p += v * h;
        v += a * h;
    }

    void RK4Step(ref Vector3 p, ref Vector3 v, Vector3 w, float h)
    {
        Vector3 k1v = Acc(v, w);
        Vector3 k1p = v;

        Vector3 v2 = v + 0.5f * h * k1v;
        Vector3 k2v = Acc(v2, w);
        Vector3 k2p = v + 0.5f * h * k1v;

        Vector3 v3 = v + 0.5f * h * k2v;
        Vector3 k3v = Acc(v3, w);
        Vector3 k3p = v + 0.5f * h * k2v;

        Vector3 v4 = v + h * k3v;
        Vector3 k4v = Acc(v4, w);
        Vector3 k4p = v + h * k3v;

        p += (h / 6f) * (k1p + 2f * k2p + 2f * k3p + k4p);
        v += (h / 6f) * (k1v + 2f * k2v + 2f * k3v + k4v);
    }

    // ========== Table collision (sliding/rolling branches) ==========
    void TableBounce(ref Vector3 v, ref Vector3 w)
    {
        float speed = Mathf.Max(v.magnitude, 1e-6f);
        float vy = v.y;

        // Determine sliding/rolling
        float vs = 1f - 2.5f * muTable * (1f + enTable) * Mathf.Abs(vy) / speed;
        float alpha = (vs > 0f)
            ? muTable * (1f + enTable) * Mathf.Abs(vy) / speed
            : 0.4f; // 2/5

        // Paper matrix (y as normal), expanded by components
        Vector3 vOld = v, wOld = w;
        TableBounced?.Invoke(pos, v);  // pos is a TTBall member (ball center in world coordinates)

        float vxp = (1f - alpha) * vOld.x + (alpha * r) * wOld.z;
        float vyp = -enTable * vOld.y;
        float vzp = (1f - alpha) * vOld.z - (alpha * r) * wOld.x;

        float wxp = (1f - 1.5f * alpha) * wOld.x - (1.5f * alpha / r) * vOld.z;
        float wyp = wOld.y; // This model does not change y spin
        float wzp = (1f - 1.5f * alpha) * wOld.z + (1.5f * alpha / r) * vOld.x;

        v = new Vector3(vxp, vyp, vzp);
        w = new Vector3(wxp, wyp, wzp);
    }

    // ========== Racket hit (for HitZone call) ==========
    [System.Serializable]
    public struct RacketParams
    {
        public float kv;          // Tangential restitution
        public float kw;          // Rotation-linear coupling
        public float er;          // Normal restitution
        public Vector3 vR;        // Racket centroid velocity (world frame)
        public float alphaYaw;    // Yaw ¦Á (rad)
        public float betaPitch;   // Pitch ¦Â (rad)
    }

    public void ApplyRacketHit(RacketParams rp)
    {
        // Construct R_R from paper formula (15)
        float ca = Mathf.Cos(rp.alphaYaw), sa = Mathf.Sin(rp.alphaYaw);
        float cb = Mathf.Cos(rp.betaPitch), sb = Mathf.Sin(rp.betaPitch);

        // Row-major 3x3
        Matrix3x3 RR = new Matrix3x3(
            cb, sb * sa, sb * ca,
            0f, ca, -sa,
           -sb, cb * sa, cb * ca
        );

        // Block matrices
        float kv = rp.kv, kw = rp.kw, er = rp.er;

        Matrix3x3 Avv = Matrix3x3.Diag(1f - kv, 1f - kv, -er);
        Matrix3x3 Aww = Matrix3x3.Diag(1f - kw * r * r, 1f - kw * r * r, 1f);
        Matrix3x3 Avw = new Matrix3x3(
             0f, kv * r, 0f,
            -kv * r, 0f, 0f,
             0f, 0f, 0f
        );
        Matrix3x3 Awv = new Matrix3x3(
             0f, -kw * r, 0f,
             kw * r, 0f, 0f,
             0f, 0f, 0f
        );

        // Transform to racket local frame
        Vector3 vRel = vel - rp.vR;
        Vector3 vLoc = RR.TransposeMultiply(vRel);
        Vector3 wLoc = RR.TransposeMultiply(omega);

        // Apply block matrices
        Vector3 vLocP = Avv * vLoc + Avw * wLoc;
        Vector3 wLocP = Awv * vLoc + Aww * wLoc;

        // Transform back to world and add racket velocity
        vel = RR * vLocP + rp.vR;
        omega = RR * wLocP;
        RacketHitFired?.Invoke(pos, vel, omega);

    }

    // ========== Debug visualization ==========
    void OnDrawGizmos()
    {
        if (!drawGizmos) return;

        Vector3 p = Application.isPlaying ? pos : transform.position;

        Gizmos.color = Color.white;
        Gizmos.DrawWireSphere(p, r);

        // Velocity arrow
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(p, p + (Application.isPlaying ? vel : Vector3.zero) * gizmoScale);

        // Magnus direction (¦Ø ¡Á v)
        if (Application.isPlaying)
        {
            Gizmos.color = Color.magenta;
            Vector3 magnusDir = Vector3.Cross(omega, vel);
            Gizmos.DrawLine(p, p + magnusDir.normalized * gizmoScale * 2f);
        }

        // Table outline
        Gizmos.color = Color.green;
        Gizmos.DrawLine(new Vector3(-2, tableY, -2), new Vector3(2, tableY, -2));
        Gizmos.DrawLine(new Vector3(2, tableY, -2), new Vector3(2, tableY, 2));
        Gizmos.DrawLine(new Vector3(2, tableY, 2), new Vector3(-2, tableY, 2));
        Gizmos.DrawLine(new Vector3(-2, tableY, 2), new Vector3(-2, tableY, -2));
    }

    // ========== Simple 3x3 ==========
    struct Matrix3x3
    {
        // Row-major
        public float m00, m01, m02;
        public float m10, m11, m12;
        public float m20, m21, m22;

        public Matrix3x3(
            float m00, float m01, float m02,
            float m10, float m11, float m12,
            float m20, float m21, float m22)
        {
            this.m00 = m00; this.m01 = m01; this.m02 = m02;
            this.m10 = m10; this.m11 = m11; this.m12 = m12;
            this.m20 = m20; this.m21 = m21; this.m22 = m22;
        }

        public static Matrix3x3 Diag(float a, float b, float c)
            => new Matrix3x3(a, 0, 0, 0, b, 0, 0, 0, c);

        public static Vector3 operator *(Matrix3x3 M, Vector3 v)
        {
            return new Vector3(
                M.m00 * v.x + M.m01 * v.y + M.m02 * v.z,
                M.m10 * v.x + M.m11 * v.y + M.m12 * v.z,
                M.m20 * v.x + M.m21 * v.y + M.m22 * v.z
            );
        }

        public Vector3 TransposeMultiply(Vector3 v)
        {
            // M^T * v
            return new Vector3(
                m00 * v.x + m10 * v.y + m20 * v.z,
                m01 * v.x + m11 * v.y + m21 * v.z,
                m02 * v.x + m12 * v.y + m22 * v.z
            );
        }
    }
}