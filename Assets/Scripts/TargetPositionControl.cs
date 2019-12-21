using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

public class TargetPositionControl : MonoBehaviour
{

    [SerializeField]
    float speed = 5f;

    [SerializeField]
    float angularSpeed = 60f;

    [SerializeField]
    Toggle physicsToggle;

    public Vector3 movement;
    public float degree;
    public bool triggleGripping;
    public bool rigorousPhysicsSimulation = false;

    float timeThreshold = 0f;

    // Use this for initialization
    void Start()
    {
        triggleGripping = false;
    }

    private void Update()
    {
        EventSystem.current.SetSelectedGameObject(null);
        rigorousPhysicsSimulation = physicsToggle.isOn;
    }

    public void ProcessInput(int axisX, int axisY, int axisZ, int axisW, int space)
    {
        Transform camTras = Camera.main.transform;

        //Avoid continuously toggle the gripper
        if (timeThreshold <= 0f)
        {
            if (space == 1)
            {
                triggleGripping = !triggleGripping;
                timeThreshold = 0.5f;
            }
        }
        else
        {
            timeThreshold -= Time.fixedDeltaTime;
        }

        //Record the current keyboard instruction

        //The target movement of the head
        movement = speed * Time.fixedDeltaTime * (new Vector3(camTras.forward.x, 0, camTras.forward.z) * axisZ + new Vector3(camTras.right.x, 0, camTras.right.z) * axisX + Vector3.up * axisY);

        degree = axisW * angularSpeed * Time.fixedDeltaTime;
    }

    public void UpdateTargetPointMovement(Vector3 targetPointMovement)
    {
        transform.position += targetPointMovement;
    }
}
