using UnityEngine;
using System.Linq;

public class FABRIK : MonoBehaviour
{
    //Make references of all the joints
    [SerializeField]
    Transform[] jointTrans = new Transform[7];
    //Store positions of all the joints
    Vector3[] jointPos = new Vector3[3];
    //Store the transform of endEffector and targetPoint
    [SerializeField]
    Transform targetPoint;
    //Store the distance between each joint
    float[] distance = new float[2];
    float[] lambda = new float[2];
    [SerializeField]
    float threshold;
    [SerializeField]
    int maxIteration;
    Vector3 rootPos;
    float diff;
    int iter;
    //Allow input the angle constraint
    public float a0;
    public float a1;
    public float a2;

    Vector3 targetPos;
    Vector3 lastEndPos;

    float[] angleConstraint =  new float[3];

    int globalCount = 0;

    void Start()
    {
        targetPos = targetPoint.position;

        //Default angle constraint
        angleConstraint[0] = 90f;
        angleConstraint[1] = 120f;
        angleConstraint[2] = 120f;

        //Record other joints' postions
        jointPos[0] = jointTrans[1].position;
        jointPos[1] = jointTrans[3].position;
        jointPos[2] = jointTrans[5].position;

        //Calculate the distance (Do NOT combine with last one!)
        for (int i = 0; i < jointPos.Length - 1; i++)
        {
            distance[i] = Vector3.Distance(jointPos[i + 1], jointPos[i]);
        }
        //Record the root position
        rootPos = jointPos[0];
        //Set threshold to 0.01f
        threshold = 0.001f;
        //Set max iteration to 50
        maxIteration = 15;
        //Set angle constraint based on user input
        angleConstraint[0] = a0;
        angleConstraint[1] = a1;
        angleConstraint[2] = a2;
    }

    public Vector3 UpdateRoboticArm(Vector3 movement)
    {
        globalCount++;

        Vector3 returnMovement = movement;
        lastEndPos = jointPos[jointPos.Length - 1];
        targetPos += movement;

        //Apply FABRIK
        #region FABRIK
        //The distance between root and target
        float ReachDistance = Vector3.Distance(jointPos[0], targetPos);
        //If the target is not reachable
        if (ReachDistance > distance.Sum())
        {
            for (int i = 0; i < jointPos.Length - 1; i++)
            {
                //Calculate lambda from distance between target and the joint
                lambda[i] = distance[i] / Vector3.Distance(targetPos, jointPos[i]);
                //Find the new joint position
                jointPos[i + 1] = (1 - lambda[i]) * jointPos[i] + lambda[i] * targetPos;
            }
        }
        //If the target is reachable
        else
        {
            //calculate the difference between end effector and target position
            diff = Vector3.Distance(jointPos[jointPos.Length - 1], targetPos);
            //Record interation
            iter = 0;
            //While the distance is larger than threshold
            while (diff > threshold && iter < maxIteration)
            {
                /////////////////////////////////////////////////////
                //STAGE 1: Backward reaching

                //Set the End Effector to the Target Position
                jointPos[jointPos.Length - 1] = targetPos;

                for (int i = jointPos.Length - 2; i >= 0; i--)
                {
                    lambda[i] = distance[i] / Vector3.Distance(jointPos[i + 1], jointPos[i]);
                    //Store the calculated new point into a temp vector3
                    Vector3 targetJoint = (1 - lambda[i]) * jointPos[i + 1] + lambda[i] * jointPos[i];
                    jointPos[i] = CalculateClosestPoint(jointPos[i+1], jointPos[i] - jointPos[i + 1], angleConstraint[i], targetJoint);
                }
                /////////////////////////////////////////////////////
                //STAGE 2: Forward reaching
                //Set back to initial position
                jointPos[0] = rootPos;
                for (int i = 0; i < jointPos.Length - 1; i++)
                {
                    lambda[i] = distance[i] / Vector3.Distance(jointPos[i + 1], jointPos[i]);

                    //Store the calculated new point into a temp vector3
                    Vector3 targetJoint = (1 - lambda[i]) * jointPos[i] + lambda[i] * jointPos[i + 1];
                    if(i == 0)
                        jointPos[i + 1] = CalculateClosestPoint(jointPos[i], Vector3.up, angleConstraint[i], targetJoint);
                    else
                        jointPos[i + 1] = CalculateClosestPoint(jointPos[i], jointPos[i] - jointPos[i-1], angleConstraint[i], targetJoint);
                }
                diff = Vector3.Distance(jointPos[jointPos.Length - 1], targetPos);
                iter++;
            }
        }
        #endregion

        DrawDebugLine();

        //If the target is not reachable
        //if (ReachDistance > distance.Sum())
        //{
        if (globalCount == 1)
        {
            returnMovement = movement;
        }
        else
        {
            returnMovement = jointPos[jointPos.Length - 1] - lastEndPos;

        }
        //}
        //else if (iter >= maxIteration)
        //{
        //    returnMovement = jointPos[jointPos.Length - 1] - targetPos;
        //}
        //else
        //{
        //    returnMovement = movement;
        //}

        return returnMovement;
    }

    private void LateUpdate()
    {
         MapMeshRotation();
    }

    //Test the virtual position after iteration
    void DrawDebugLine()
    {
        for (int i = 0; i < jointPos.Length - 1; i++)
        {
            //For debug only
            Debug.DrawLine(jointPos[i], jointPos[i + 1], Color.red);
        }
    }

    //Map the rotation to mesh
    void MapMeshRotation()
    {
        Vector3 localForward = Vector3.ProjectOnPlane((Vector3.zero - rootPos), Vector3.up).normalized;

        Vector3 targetVector1 = jointPos[1] - jointPos[0];
        jointTrans[0].rotation = Quaternion.FromToRotation(Vector3.right ,new Vector3(targetVector1.x, 0, targetVector1.z));
        jointTrans[1].localRotation = Quaternion.AngleAxis(-Vector3.Angle(Vector3.up, targetVector1), Vector3.forward);

        Vector3 targetVector2 = jointPos[2] - jointPos[1];
        if (Vector3.ProjectOnPlane(targetVector2, targetVector1) != Vector3.zero)
            jointTrans[2].rotation = Quaternion.LookRotation(Vector3.Cross(Vector3.ProjectOnPlane(targetVector2, targetVector1), targetVector1), targetVector1);
        jointTrans[3].localRotation = Quaternion.AngleAxis(-Vector3.Angle(targetVector1, targetVector2), Vector3.forward);

        Vector3 targetVector3 = localForward;
        if (Vector3.ProjectOnPlane(targetVector3, targetVector2) != Vector3.zero)
            jointTrans[4].rotation = Quaternion.LookRotation(Vector3.Cross(Vector3.ProjectOnPlane(targetVector3, targetVector2), targetVector2), targetVector2);
        jointTrans[5].localRotation = Quaternion.AngleAxis(-Vector3.Angle(targetVector2, targetVector3), Vector3.forward);

        jointTrans[6].rotation = Quaternion.Euler(90f, 90f, 90f);
        jointTrans[6].localEulerAngles = new Vector3(jointTrans[6].localEulerAngles.x, jointTrans[6].localEulerAngles.y, 0f);

    }

    //Function to calculate the closest point within reach
    Vector3 CalculateClosestPoint(Vector3 _origin, Vector3 _dir, float _theta, Vector3 _target)
    {
        Vector3 newTarget = Vector3.zero;

        //Test the range of the valid theta
        if (_theta < 90f)
        {
            //Pointing from origin to target
            Vector3 v1 = _target - _origin;
            //A ray pointing from origin to direction
            Ray _ray = new Ray(_origin, _dir);
            //v2 to is the projection of v1 on ray
            Vector3 v2 = Vector3.Project(v1, _ray.direction);
            //v3 points from end of project to target
            Vector3 v3 = v1 - v2;
            if (v2 == Vector3.zero)
            {
                //Not sure here
                newTarget = _origin;
            }
            else
            {
                //Within reach
                if (v3.magnitude <= v2.magnitude * Mathf.Tan(_theta))
                {
                    newTarget = _target;
                }
                //Out of reach
                else
                {
                    //If the projection is at the same direction as ray
                    if (Vector3.Dot(v2, _ray.direction) > 0)
                    {
                        newTarget = v2.magnitude * Mathf.Tan(_theta) * v3.normalized + _origin + v2;
                    }
                    //If the projection is at the opposite direction of ray
                    else if (Vector3.Dot(v2, _ray.direction) < 0)
                    {
                        newTarget = v2.magnitude * Mathf.Tan(_theta) * v3.normalized + _origin - v2;
                    }
                }
            }
        }
        else if (_theta == 90f)
        {
            //Pointing from origin to target
            Vector3 v1 = _target - _origin;
            //A ray pointing from origin to direction
            Ray _ray = new Ray(_origin, _dir);
            //If within reach
            if (Vector3.Dot(v1, _ray.direction) >= 0)
            {
                newTarget = _target;
            }
            else
            {
                newTarget = Vector3.ProjectOnPlane(v1, _ray.direction) + _origin;
            }
        }
        else if(_theta > 90f)
        {
            //Pointing from origin to target
            Vector3 v1 = _target - _origin;
            //A ray pointing from origin to direction
            Ray _ray = new Ray(_origin, _dir);
            //v2 to is the projection of v1 on ray
            Vector3 v2 = Vector3.Project(v1, _ray.direction);
            //v3 points from end of project to target
            Vector3 v3 = v1 - v2;
            if (v2 == Vector3.zero)
            {
                //Not sure here
                newTarget = _target;
            }
            else
            {
                if (Vector3.Dot(v2, _ray.direction) > 0)
                {
                    newTarget = _target;
                }
                else
                {
                    //Within reach
                    if (v3.magnitude >= v2.magnitude * Mathf.Tan(180 - _theta))
                    {
                        newTarget = _target;
                    }
                    //Out of reach
                    else
                    {
                        newTarget = v2.magnitude * Mathf.Tan(180 - _theta) * v3.normalized + _origin + v2;
                    }
                }
            }
        }
        return newTarget;
    }

    //Rotate robotic arm by certain degree
    public void RoateRoboticArm(float degree)
    {
        transform.RotateAround(Vector3.zero, Vector3.up, degree);
        jointPos[0] = RotatePointAroundPivot(jointPos[0], Vector3.Project(jointPos[0], Vector3.up), new Vector3(0f, degree, 0f));
        //Update Root position
        rootPos = jointPos[0];
        UpdateRoboticArm(targetPoint.position - targetPos);
    }

    //Rotate a point around a privot point
    Vector3 RotatePointAroundPivot(Vector3 point, Vector3 pivot, Vector3 angles)
    {
        return Quaternion.Euler(angles) * (point - pivot) + pivot;
    }

    public void RecordJointPosi()
    {
        transform.position = new Vector3(0, 0, -10);
        transform.rotation = Quaternion.Euler(0, 0, 0);

        //Record other joints' postions
        jointPos[0] = jointTrans[1].position;
        jointPos[1] = jointTrans[3].position;
        jointPos[2] = jointTrans[5].position;
        rootPos = jointPos[0];

        targetPos = targetPoint.position;
        UpdateRoboticArm(Vector3.zero);
        RoateRoboticArm(0f);
    }
}

