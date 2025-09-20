using UnityEngine;

public class TableTennisBall : MonoBehaviour
{
    [SerializeField] private float InitialSpeed = 2f;
    [SerializeField] private TableTennisEnvControl envController;

    // 状态与判罚
    [HideInInspector] public PaddleState mPaddleState = PaddleState.NoPaddle;
    [HideInInspector] public AreaState mAreaState = AreaState.NoArea;

    // 记录起点（用世界坐标 & 欧拉角更直观）
    private Vector3 startingPosition;
    private Vector3 startingEuler;

    // 引用
    private TTBall sim;           // 我们的自写动力学
    private Rigidbody rb;         // 仅当“载体”，不再施力

    // —— 初始化 —— //
    private void Awake()
    {
        startingPosition = transform.position;
        startingEuler = transform.eulerAngles;

        sim = GetComponent<TTBall>();
        rb = GetComponent<Rigidbody>();

        // 关键：让 PhysX 不再解算这颗球
        if (rb != null)
        {
            rb.useGravity = false;
            rb.isKinematic = true;    // 位置速度都由 TTBall 驱动
        }

        // 订阅 TTBall 的事件（替代 OnCollisionEnter/击球判定）
        if (sim != null)
        {
            sim.TableBounced += OnTableBounced;
            sim.RacketHitFired += OnRacketHit;
        }
        else
        {
            Debug.LogError("[TableTennisBall] 找不到 TTBall，请把 TTBall.cs 挂在同一物体上。");
        }
    }

    // —— 事件回调 —— //
    private void OnTableBounced(Vector3 pos, Vector3 vel)
    {
        // 你原逻辑用 localPosition.z 判左右场区，这里保持一致
        float z = transform.localPosition.z;

        if (z > 0f)
        {
            if (mPaddleState == PaddleState.LeftPaddle)
            {
                if (mAreaState == AreaState.LeftArea)
                {
                    envController.HitSuccess(TTHit.LeftPaddleHit);
                    mAreaState = AreaState.RightArea;
                }
                else if (mAreaState == AreaState.RightArea)
                {
                    mPaddleState = PaddleState.NoPaddle;
                    mAreaState = AreaState.NoArea;
                    envController.ResolveEvent(TTEvent.LeftPaddleGoal);
                }
                else
                {
                    mAreaState = AreaState.RightArea;
                }
            }
            else if (mPaddleState == PaddleState.RightPaddle)
            {
                mPaddleState = PaddleState.NoPaddle;
                mAreaState = AreaState.NoArea;
                envController.Foul(TTFoul.RightPaddleFoul);
            }
            else
            {
                mAreaState = AreaState.RightArea;
            }
        }
        else if (z < 0f)
        {
            if (mPaddleState == PaddleState.RightPaddle)
            {
                if (mAreaState == AreaState.RightArea)
                {
                    envController.HitSuccess(TTHit.RightPaddleHit);
                    mAreaState = AreaState.LeftArea;
                }
                else if (mAreaState == AreaState.LeftArea)
                {
                    mPaddleState = PaddleState.NoPaddle;
                    mAreaState = AreaState.NoArea;
                    envController.ResolveEvent(TTEvent.RightPaddleGoal);
                }
                else
                {
                    mAreaState = AreaState.LeftArea;
                }
            }
            else if (mPaddleState == PaddleState.LeftPaddle)
            {
                mPaddleState = PaddleState.NoPaddle;
                mAreaState = AreaState.NoArea;
                envController.Foul(TTFoul.LeftPaddleFoul);
            }
            else
            {
                mAreaState = AreaState.LeftArea;
            }
        }
    }

    private void OnRacketHit(Vector3 pos, Vector3 vel, Vector3 omega)
    {
        // 这里不必改动你的判罚逻辑；“是谁打的”仍由 OnTriggerEnter 决定（见下）
        // 如需基于击球事件做额外逻辑，可在此扩展。
    }

    // —— 只保留“是谁碰到拍”的触发（但它不再影响物理） —— //
    private void OnTriggerEnter(Collider collision)
    {
        if (collision.gameObject.CompareTag("RightPaddle"))
        {
            if (mAreaState == AreaState.RightArea)
            {
                if (mPaddleState == PaddleState.RightPaddle)
                {
                    mPaddleState = PaddleState.NoPaddle;
                    mAreaState = AreaState.NoArea;
                    envController.Foul(TTFoul.RightPaddleFoul);
                }
                else
                {
                    mPaddleState = PaddleState.RightPaddle;
                    envController.GetSuccess(TTGet.RightPaddleGet);
                }
            }
            else
            {
                mPaddleState = PaddleState.NoPaddle;
                mAreaState = AreaState.NoArea;
                envController.Foul(TTFoul.RightPaddleFoul);
            }
        }
        else if (collision.gameObject.CompareTag("LeftPaddle"))
        {
            if (mAreaState == AreaState.LeftArea)
            {
                if (mPaddleState == PaddleState.LeftPaddle)
                {
                    mPaddleState = PaddleState.NoPaddle;
                    mAreaState = AreaState.NoArea;
                    envController.Foul(TTFoul.LeftPaddleFoul);
                }
                else
                {
                    mPaddleState = PaddleState.LeftPaddle;
                    envController.GetSuccess(TTGet.LeftPaddleGet);
                }
            }
            else
            {
                mPaddleState = PaddleState.NoPaddle;
                mAreaState = AreaState.NoArea;
                envController.Foul(TTFoul.LeftPaddleFoul);
            }
        }
    }

    // —— 彻底停用 PhysX 碰撞落台判定（交给 TTBall.TableBounced） —— //
    private void OnCollisionEnter(Collision collision) { /* 不再使用。保留空函数避免误触。 */ }

    // —— 发球（不再 AddForce；直接设 TTBall 的速度） —— //
    public void Launch()
    {
        transform.position = startingPosition;
        transform.eulerAngles = startingEuler;

        if (sim == null) sim = GetComponent<TTBall>();

        float x, z;
        if (Random.value < 0.5f)
        {
            z = 2f; mPaddleState = PaddleState.LeftPaddle; x = Random.Range(-1f, 0f);
        }
        else
        {
            z = -2f; mPaddleState = PaddleState.RightPaddle; x = Random.Range(0f, 1f);
        }

        Vector3 dir = new Vector3(x, 0f, z).normalized;
        Vector3 v0 = dir * InitialSpeed;
        sim.ResetState(startingPosition, v0, Vector3.zero);

        // 注意：不要再用 rb.velocity / rb.AddForce
        if (rb != null) { rb.velocity = Vector3.zero; rb.angularVelocity = Vector3.zero; }
        mAreaState = AreaState.NoArea;
    }
}
