using UnityEngine;
using UnityEngine.UI;

public class RewardCalculation : MonoBehaviour
{

    float total_reward = 0f;
    [SerializeField]
    Text rotateAngleText;
    [SerializeField]
    Text XTranslationDistanceText;
    [SerializeField]
    Text YTranslationDistanceText;
    [SerializeField]
    Text forwardDistanceText;

    [SerializeField]
    Camera playerCamera;

    [SerializeField]
    Transform centralPoint;

    float rotAngle;
    float rot_reward;

    float translationDistance;
    float translate_reward;
    Vector3 translation;

    float middleDistance;
    float forward_reward;

    private void FixedUpdate()
    {
        if (HighlightBlock.highlightedBlockId == 0)
            return;

        Vector3 middleForward = Vector3.ProjectOnPlane(playerCamera.transform.forward, Vector3.up);

        //Reward 1
        Transform highlightedBlockTransform = SceneController.GetBlock(HighlightBlock.highlightedBlockId).transform;
        Ray highlightedBlockAxisRay = new Ray(highlightedBlockTransform.position, Vector3.ProjectOnPlane(highlightedBlockTransform.forward, Vector3.up));
        Debug.DrawRay(highlightedBlockAxisRay.origin, highlightedBlockAxisRay.direction, Color.red);
        Vector3 highlightedBlockAxis = highlightedBlockAxisRay.direction;
        //Ray pointing from middle point
        Ray centralRay = new Ray(centralPoint.position, middleForward);
        Debug.DrawRay(centralRay.origin, centralRay.direction, Color.red);
        Vector3 centralAxis = centralRay.direction;
        float rot1 = Vector3.Angle(centralAxis, highlightedBlockAxis);
        float rot2 = Vector3.Angle(centralAxis, -highlightedBlockAxis);
        //Between 0 to 90 degree
        rotAngle = rot1 < rot2 ? rot1 : rot2;
        rot_reward = 1 - Mathf.Abs(rotAngle / 90f);


        //Reward 2
        //Distance in camera plane
        Vector3 targetPosition = ProjectPointOnPlane(middleForward, centralPoint.position, highlightedBlockTransform.position);
        translationDistance = Vector3.Distance(centralPoint.position, targetPosition);
        translation = targetPosition - centralPoint.position;
        translate_reward = 1 - Mathf.Abs(Mathf.Sqrt((translation.x / 6f) * (translation.x / 6f) + (translation.y / 12f) * (translation.y / 12f)));


        //Reward 3
        middleDistance = 5f;
        RaycastHit hit;
        if (Physics.Raycast(new Ray(centralPoint.position, middleForward), out hit, 5f))
        {
            if (hit.transform != null)
            {
                if (hit.transform.GetComponent<Block>() != null)
                {
                    if (hit.transform.GetComponent<Block>().blockId == HighlightBlock.highlightedBlockId)
                    {
                        middleDistance = hit.distance;
                        //Debug.Log(hit.distance);
                    }
                }
            }
        }
        middleDistance = Mathf.Abs(middleDistance - 0.1f);
        forward_reward = 1 - Mathf.Abs(middleDistance / 5f);


        total_reward = (0.3f * rot_reward + 0.3f * translate_reward + 0.3f * forward_reward);

        //Update UI element
        rotateAngleText.text = ObtainRotateAngle().ToString();
        XTranslationDistanceText.text = ObtainXTranslationDistance().ToString();
        YTranslationDistanceText.text = ObtainYTranslationDistance().ToString();
        forwardDistanceText.text = ObtainForwardDistance().ToString();
    }

    // Update is called once per frame
    public float ObtainTotalReward()
    {
        return total_reward;
    }

    public float ObtainRotateAngle()
    {
        return rotAngle;
    }

    public float ObtainRotReward()
    {
        return rot_reward;
    }

    public float ObtainYTranslationDistance()
    {
        return Mathf.Abs(translation.y);
    }

    public float ObtainXTranslationDistance()
    {
        return Mathf.Abs(translation.x);
    }

    public float ObtainTranslationReward()
    {
        return translate_reward;
    }

    public float ObtainForwardDistance()
    {
        return middleDistance;
    }

    public float ObtainForwardReward()
    {
        return forward_reward;
    }

    public Vector3 ProjectPointOnPlane(Vector3 planeNormal, Vector3 planePoint, Vector3 point)
    {

        float distance;
        Vector3 translationVector;

        //First calculate the distance from the point to the plane:
        distance = SignedDistancePlanePoint(planeNormal, planePoint, point);

        //Reverse the sign of the distance
        distance *= -1;

        //Get a translation vector
        translationVector = SetVectorLength(planeNormal, distance);

        //Translate the point to form a projection
        return point + translationVector;
    }



    //Get the shortest distance between a point and a plane. The output is signed so it holds information
    //as to which side of the plane normal the point is.
    public float SignedDistancePlanePoint(Vector3 planeNormal, Vector3 planePoint, Vector3 point)
    {
        return Vector3.Dot(planeNormal, (point - planePoint));
    }


    //create a vector of direction "vector" with length "size"
    public Vector3 SetVectorLength(Vector3 vector, float size)
    {
        //normalize the vector
        Vector3 vectorNormalized = Vector3.Normalize(vector);

        //scale the vector
        return vectorNormalized *= size;
    }
}
