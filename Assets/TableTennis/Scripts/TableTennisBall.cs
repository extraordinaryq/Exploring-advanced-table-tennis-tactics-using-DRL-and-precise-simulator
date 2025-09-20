using UnityEngine;

public class TableTennisBall : MonoBehaviour
{
    [SerializeField] private float InitialSpeed = 2f;
    [SerializeField] private TableTennisEnvControl envController;

    // ״̬���з�
    [HideInInspector] public PaddleState mPaddleState = PaddleState.NoPaddle;
    [HideInInspector] public AreaState mAreaState = AreaState.NoArea;

    // ��¼��㣨���������� & ŷ���Ǹ�ֱ�ۣ�
    private Vector3 startingPosition;
    private Vector3 startingEuler;

    // ����
    private TTBall sim;           // ���ǵ���д����ѧ
    private Rigidbody rb;         // ���������塱������ʩ��

    // ���� ��ʼ�� ���� //
    private void Awake()
    {
        startingPosition = transform.position;
        startingEuler = transform.eulerAngles;

        sim = GetComponent<TTBall>();
        rb = GetComponent<Rigidbody>();

        // �ؼ����� PhysX ���ٽ��������
        if (rb != null)
        {
            rb.useGravity = false;
            rb.isKinematic = true;    // λ���ٶȶ��� TTBall ����
        }

        // ���� TTBall ���¼������ OnCollisionEnter/�����ж���
        if (sim != null)
        {
            sim.TableBounced += OnTableBounced;
            sim.RacketHitFired += OnRacketHit;
        }
        else
        {
            Debug.LogError("[TableTennisBall] �Ҳ��� TTBall����� TTBall.cs ����ͬһ�����ϡ�");
        }
    }

    // ���� �¼��ص� ���� //
    private void OnTableBounced(Vector3 pos, Vector3 vel)
    {
        // ��ԭ�߼��� localPosition.z �����ҳ��������ﱣ��һ��
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
        // ���ﲻ�ظĶ�����з��߼�������˭��ġ����� OnTriggerEnter ���������£�
        // ������ڻ����¼��������߼������ڴ���չ��
    }

    // ���� ֻ��������˭�����ġ��Ĵ�������������Ӱ������ ���� //
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

    // ���� ����ͣ�� PhysX ��ײ��̨�ж������� TTBall.TableBounced�� ���� //
    private void OnCollisionEnter(Collision collision) { /* ����ʹ�á������պ��������󴥡� */ }

    // ���� ���򣨲��� AddForce��ֱ���� TTBall ���ٶȣ� ���� //
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

        // ע�⣺��Ҫ���� rb.velocity / rb.AddForce
        if (rb != null) { rb.velocity = Vector3.zero; rb.angularVelocity = Vector3.zero; }
        mAreaState = AreaState.NoArea;
    }
}
