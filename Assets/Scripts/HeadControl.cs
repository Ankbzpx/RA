using UnityEngine;
using System.Collections;

public class HeadControl : MonoBehaviour {

    public enum GripperState {idle, gripping, gripped, releasing};

    public static int gripCount = 0;

    bool hasGripped = false;
    const int blockLayerMaskIndex = 8;
    const int grippedBlockLayerMaskIndex = 9;

	const float CollisionOffset = 0.001f;
	const int MaxBufferSize = 32;

	//Reference for the gripped objects
	public BoxCollider[] grippedObjectsBoxCollider = new BoxCollider[MaxBufferSize];

    //The movement of current update along x and z axis
    Vector3 currentMovement = Vector3.zero;
    //The movement of current update along y axis
    Vector3 currentMovementDownward = Vector3.zero;
    //The speed of current update
    Vector3 currentVelocity = Vector3.zero;

    Vector3 leftGripperMovement;
    Vector3 rightGripperMovement;

    public GripperState gripperState;
    bool leftGripperGripped = false;
    bool rightGripperGripped = false;
    bool leftGripperReleased = false;
    bool rightGripperReleased = false;


    [SerializeField]
    TargetPositionControl targetPositionController;
    [SerializeField]
    FABRIK fabrik;

    [SerializeField]
    Transform headTransform;

    //Make references for the two gripper
    [SerializeField]
    public Transform leftGripper;
    [SerializeField]
    public Transform rightGripper;

    BoxCollider leftGripperBoxCollider;
    BoxCollider rightGripperBoxCollider;

    public Rigidbody leftGripperRigidbody;
    public Rigidbody rightGripperRigidbody;

    [SerializeField]
    Transform leftPoint;
    [SerializeField]
    Transform middlePoint;
    [SerializeField]
    Transform rightPoint;
    [SerializeField]
    Transform centralPoint;

    Vector3 lastLeftPos, lastRightPos;

    Vector3 originLeftGripperPos, originRightGripperPos, originLeftPointPos, originRightPointPos, originMiddlePointPos, originCentralPointPos;

    // Use this for initialization
    void Start ()
    {
        leftGripperBoxCollider = leftGripper.GetComponent<BoxCollider>();
        rightGripperBoxCollider = rightGripper.GetComponent<BoxCollider>();

        leftGripperRigidbody = leftGripper.GetComponent<Rigidbody>();
        rightGripperRigidbody = rightGripper.GetComponent<Rigidbody>();

        //transform.position = targetPositionController.transform.position + new Vector3(0, 0, 2.2f);
        //transform.rotation = headTransform.rotation;

        originLeftGripperPos = leftGripper.position;
        originRightGripperPos = rightGripper.position;
        originLeftPointPos = leftPoint.position;
        originRightPointPos = rightPoint.position;
        originMiddlePointPos = middlePoint.position;
        originCentralPointPos = centralPoint.position;

        gripperState = GripperState.idle;
    }

    private void FixedUpdate()
    {
        #region Calculate Movement
        //Update rotation
        transform.rotation = headTransform.rotation;

        //Obtain the current movement and speed for this update and decomposites it into two
        currentMovement = Vector3.Lerp(Vector3.zero, targetPositionController.movement, Mathf.Exp(10f * Time.fixedDeltaTime)) * Time.deltaTime;

        //Limit the movement within range
        Vector3 potentialPos = currentMovement + centralPoint.localPosition;

        if ((potentialPos.x > 5f && currentMovement.x > 0f) || (potentialPos.x < -5f && currentMovement.x < 0f))
        {
            currentMovement.x = 0f;
        }

        if ((potentialPos.y > 6f && currentMovement.y > 0f) || (potentialPos.y < -6f && currentMovement.y < 0f))
        {
            currentMovement.y = 0f;
        }

        if((potentialPos.z > 6f && currentMovement.z > 0f) || (potentialPos.z < -2f && currentMovement.z < 0f))
        {
            currentMovement.z = 0f;
        }

        currentMovementDownward = Vector3.zero;

        if (currentMovement.y < 0)
        {
            currentMovementDownward = new Vector3(0, currentMovement.y, 0);
            currentMovement = new Vector3(currentMovement.x, 0, currentMovement.z);
        }

        //Handle downward movement
        if (currentMovementDownward.y != 0f)
        {
            HandleDownwardMovement();
        }

        //Calculate movement
        GripperAndRoboticArmMove();

        //Combine and Update the movement
        currentMovement = currentMovement + currentMovementDownward;
        #endregion

        #region Apply movement
        //The target movement may not be reachable in FABRIK (May cause issues for downward movement)
        currentMovement = fabrik.UpdateRoboticArm(currentMovement);

        targetPositionController.UpdateTargetPointMovement(currentMovement);

        leftPoint.Translate(currentMovement, Space.World);
        rightPoint.Translate(currentMovement, Space.World);
        middlePoint.Translate(currentMovement, Space.World);
        centralPoint.Translate(currentMovement, Space.World);

        leftGripperRigidbody.MovePosition(leftGripper.position + currentMovement);
        rightGripperRigidbody.MovePosition(rightGripper.position + currentMovement);

        for (int i = 0; i < grippedObjectsBoxCollider.Length; i++)
        {
            if (grippedObjectsBoxCollider[i] != null && grippedObjectsBoxCollider[i].GetComponent<Rigidbody>() != null)
            {
                //Move position enable the resistance during translation
                if (targetPositionController.rigorousPhysicsSimulation)
                    grippedObjectsBoxCollider[i].GetComponent<Rigidbody>().MovePosition(grippedObjectsBoxCollider[i].transform.position + currentMovement);
                else
                    //No resistance applied during translation
                    grippedObjectsBoxCollider[i].transform.Translate(currentMovement, Space.World);
            }
        }

        #endregion

    }

    void LateUpdate()
    {
        DebugGrippedObjects();

        //Do not move gripper, wait for keyboard input
        if (gripperState == GripperState.idle)
        {
            leftGripperGripped = false;
            rightGripperGripped = false;
            leftGripperReleased = false;
            rightGripperReleased = false;

            if (targetPositionController.triggleGripping)
            {
                gripperState = GripperState.gripping;
                targetPositionController.triggleGripping = false;
            }
        }

        //Start gripping until finishes
        if (gripperState == GripperState.gripping)
        {
            GripperGripping();
        }

        //Wait for keyboard input for releasing
        if (gripperState == GripperState.gripped)
        {
            if (targetPositionController.triggleGripping)
            {
                gripperState = GripperState.releasing;
                targetPositionController.triggleGripping = false;
            }

			GripObjects();
        }

        //Keep releasing until finished
        if (gripperState == GripperState.releasing)
        {
            GripperReleasing();

            ReleaseObjects();
        }

        //Handle rotation
        if (targetPositionController.degree != 0f)
        {
            fabrik.RoateRoboticArm(targetPositionController.degree);

            //Rotate the selected block
            for (int i = 0; i < grippedObjectsBoxCollider.Length; i++)
            {
                if (grippedObjectsBoxCollider[i] != null && grippedObjectsBoxCollider[i].GetComponent<Rigidbody>() != null)
                {
                    grippedObjectsBoxCollider[i].transform.RotateAround(Vector3.zero, Vector3.up, targetPositionController.degree);
                }
            }
        }

    }

    //Return the array of valid overlap colliders
    Collider[] GripperCollisionOverlap(BoxCollider targetBoxCollider, Vector3 targetMovement)
    {
        //Initialize a Collider array for storing overlaps
        Collider[] overlapColliders = new Collider[MaxBufferSize];

        //Total number of overlap colliders
        int nHits = Physics.OverlapBoxNonAlloc(targetBoxCollider.transform.position + targetMovement,         //Box center is measured in local coordinate
            Vector3.Scale(targetBoxCollider.size, targetBoxCollider.transform.localScale)/2f,
            overlapColliders,
            targetBoxCollider.transform.rotation);

        //Count for invalid colliders
        int validCount = nHits;

        for (int i = 0; i < nHits; i++)
        {
            if (overlapColliders[i] == null || overlapColliders[i] == leftGripperBoxCollider || overlapColliders[i] == rightGripperBoxCollider || overlapColliders[i].gameObject.layer == grippedBlockLayerMaskIndex)
            {
                validCount--;
            }
        }

        //Store valid colliders in an return array
        Collider[] validOverlapColliders = new Collider[validCount];
        int returnCount = 0;
        for (int i = 0; i < validCount; i++)
        {
            bool hasSaved = false;

            while (!hasSaved)
            {
                //If this collider is valid
                if (overlapColliders[returnCount] != null && overlapColliders[returnCount] != leftGripperBoxCollider && overlapColliders[returnCount] != rightGripperBoxCollider && overlapColliders[returnCount].gameObject.layer != grippedBlockLayerMaskIndex)
                {
                    //Store the valid collider in the return array
                    validOverlapColliders[i] = overlapColliders[returnCount];

                    hasSaved = true;
                }
                returnCount++;
            }
        }

        return validOverlapColliders;
    }

    //Sweep collider to target position to find closest hit
    Vector3 GripperCollisionSweep(BoxCollider targetBoxCollider, Vector3 targetMovement, out RaycastHit closestHit)
    {
        Vector3 sweepMovement = Vector3.zero;
        closestHit = new RaycastHit();
        //Store all the raycast hits
        RaycastHit[] allHits = new RaycastHit[MaxBufferSize];

        //Cast box raycast with target movement to find all hits
        int nHits = Physics.BoxCastNonAlloc(targetBoxCollider.transform.position,
            Vector3.Scale(targetBoxCollider.transform.localScale, targetBoxCollider.size) / 2f,
            targetMovement.normalized,
            allHits,
            targetBoxCollider.transform.rotation,
            targetMovement.magnitude + CollisionOffset,
            grippedBlockLayerMaskIndex);

        //Obtain the closet hits
        float closestDistance = Mathf.Infinity;

        for (int i = 0; i < nHits; i++)
        {
            //May consider handling hit another gripper, not likely happens due to small delta movement
            if (allHits[i].distance < closestDistance && allHits[i].distance > 0f && allHits[i].collider != targetBoxCollider)
            {
                if (allHits[i].collider.attachedRigidbody == null || (allHits[i].collider.attachedRigidbody != null && !allHits[i].collider.attachedRigidbody.isKinematic))
                {
                    closestHit = allHits[i];
                    closestDistance = allHits[i].distance;
                }

            }
        }

        //If hit something and the closest hit is valid
        if (nHits <= 0 || closestHit.transform == null)
        {
            //Sweep movement will be the same as the target movement if nothing has been hit
            sweepMovement = targetMovement;
        }
        else
        {
            sweepMovement = targetMovement.normalized * (closestHit.distance - CollisionOffset);
        }

        return sweepMovement;
    }

    //Process hit during movement (Only hit the closest object)
    //Push once for the specific object
    void OnMovementHit(RaycastHit closestHit)
    {
        //Make a reference to the hit collider rigidbody
        if (closestHit.collider == null)
            return;

        //Disable physics for the selected block
        if (closestHit.transform.gameObject.layer == grippedBlockLayerMaskIndex)
            return;

        if (closestHit.collider.attachedRigidbody != null)
        {
            Rigidbody hitRigidBody = closestHit.collider.attachedRigidbody;

            Vector3 velocity = GetVelocityFromRigidBody(hitRigidBody, closestHit.point, Time.fixedDeltaTime);
            Vector3 relativeVelocity = Vector3.Project(currentVelocity, closestHit.normal) - velocity;
            //Apply force to the hit point
            hitRigidBody.AddForceAtPosition(0.4f * relativeVelocity.normalized, closestHit.point, ForceMode.VelocityChange);
        }
    }

    //Move gripper with the physics into consideration
    //It will change the current movement
    void GripperAndRoboticArmMove()
    {
        //Find the closest hit
        RaycastHit leftClosestHit, rightClosestHit;

        //Obtain the sweep movement for both gripper colliders
        Vector3 leftGripperSweepMovement = GripperCollisionSweep(leftGripperBoxCollider, currentMovement, out leftClosestHit);
        Vector3 rightGripperSweepMovement = GripperCollisionSweep(rightGripperBoxCollider, currentMovement, out rightClosestHit);

        //The oversweep movement is the smaller one
        Vector3 sweepMovement = (leftGripperSweepMovement.magnitude < rightGripperSweepMovement.magnitude) ? leftGripperSweepMovement : rightGripperSweepMovement;

        RaycastHit[] grippedObjectClosestHit = new RaycastHit[grippedObjectsBoxCollider.Length];

        if (hasGripped)
        {
            //For gripped block
            Vector3[] grippedBlockSweep = new Vector3[grippedObjectsBoxCollider.Length];

            for (int i = 0; i < grippedObjectsBoxCollider.Length; i++)
            {
                if (grippedObjectsBoxCollider[i] != null)
                {
                    grippedBlockSweep[i] = GripperCollisionSweep(grippedObjectsBoxCollider[i], currentMovement, out grippedObjectClosestHit[i]);
                }
                else
                {
                    grippedBlockSweep[i] = Vector3.positiveInfinity;
                }
            }

            //Find Closest movement
            Vector3 grippedObjectSweepMovement = Vector3.positiveInfinity;
            foreach (Vector3 sweep in grippedBlockSweep)
            {
                if (sweep.magnitude < grippedObjectSweepMovement.magnitude)
                {
                    grippedObjectSweepMovement = sweep;
                }
            }

            //Compare to the gripped block and check the closest movement
            sweepMovement = (sweepMovement.magnitude < grippedObjectSweepMovement.magnitude) ? sweepMovement : grippedObjectSweepMovement;

        }

        //Update current movement and current velocity
        currentMovement = sweepMovement;
        currentVelocity = currentMovement / Time.fixedDeltaTime;

        //Calculate overlap and apply force if it overlaps
        Collider[] leftOverlapAfterSweep = GripperCollisionOverlap(leftGripperBoxCollider, currentMovement);
        if (leftOverlapAfterSweep.Length > 0)
        {
            OnMovementHit(leftClosestHit);
        }

        Collider[] rightOverlapAfterSweep = GripperCollisionOverlap(rightGripperBoxCollider, currentMovement);
        if (rightOverlapAfterSweep.Length > 0)
        {
            OnMovementHit(rightClosestHit);
        }

        if (hasGripped)
        {
            //For gripped objects
            for (int i = 0; i < grippedObjectsBoxCollider.Length; i++)
            {
                if (grippedObjectsBoxCollider[i] != null)
                {
                    Collider[] grippedBlockOverlap = GripperCollisionOverlap(grippedObjectsBoxCollider[i], currentMovement);

                    if (grippedBlockOverlap.Length > 0)
                    {
                        OnMovementHit(grippedObjectClosestHit[i]);
                    }
                }
            }
        }
    }

    //Get the velocity from rigidbody
    Vector3 GetVelocityFromRigidBody(Rigidbody rb, Vector3 atPoint, float deltaTime)
    {
        if (deltaTime > 0f)
        {
            Vector3 velocity = rb.velocity;

            if (rb.angularVelocity != Vector3.zero)
            {
                Vector3 centerOfRotation = rb.position;
                Vector3 centerOfRotationToPoint = atPoint - centerOfRotation;
                Quaternion rotationFromInteractiveRigidbody = Quaternion.Euler(Mathf.Rad2Deg * rb.angularVelocity * deltaTime);
                Vector3 finalPointPosition = centerOfRotation + (rotationFromInteractiveRigidbody * centerOfRotationToPoint);
                velocity += (finalPointPosition - atPoint) / deltaTime;
            }

            return velocity;
        }
        else
        {
            return Vector3.zero;
        }
    }

    //Handle downward movement to avoid pressing the tower
    void HandleDownwardMovement()
    {
        if (GripperCollisionOverlap(leftGripperBoxCollider, Vector3.zero).Length != 0 || GripperCollisionOverlap(rightGripperBoxCollider, Vector3.zero).Length != 0)
        {
            currentMovementDownward = Vector3.zero;
            return;
        }

        //Find the closest hit
        RaycastHit leftClosestHit, rightClosestHit;

        //Obtain the sweep movement for both gripper colliders
        Vector3 leftGripperSweepMovement = GripperGroundSweep(leftGripperBoxCollider, currentMovementDownward, out leftClosestHit);
        Vector3 rightGripperSweepMovement = GripperGroundSweep(rightGripperBoxCollider, currentMovementDownward, out rightClosestHit);

        //Sync 1: The oversweep movement is the smaller one
        Vector3 sweepMovement = (leftGripperSweepMovement.magnitude < rightGripperSweepMovement.magnitude) ? leftGripperSweepMovement : rightGripperSweepMovement;
        //For gripped block
        Vector3[] grippedBlockSweep = new Vector3[grippedObjectsBoxCollider.Length];
        Vector3 grippedObjectSweepMovement = Vector3.positiveInfinity;

        if (hasGripped)
        {

            for (int i = 0; i < grippedObjectsBoxCollider.Length; i++)
            {
                if (grippedObjectsBoxCollider[i] != null)
                {
                    RaycastHit grippedBlockCloseHit;
                    grippedBlockSweep[i] = GripperGroundSweep(grippedObjectsBoxCollider[i], currentMovementDownward, out grippedBlockCloseHit);
                }
                else
                {
                    grippedBlockSweep[i] = Vector3.positiveInfinity;
                }
            }

            //Find Closest movement
            foreach (Vector3 sweep in grippedBlockSweep)
            {
                if (sweep.magnitude < grippedObjectSweepMovement.magnitude)
                {
                    grippedObjectSweepMovement = sweep;
                }
            }
            //Compare to the gripped block and check the closest movement
            sweepMovement = (sweepMovement.magnitude < grippedObjectSweepMovement.magnitude) ? sweepMovement : grippedObjectSweepMovement;
        }


        Vector3 clampedSweepMovement = sweepMovement.normalized * (sweepMovement.magnitude - CollisionOffset);

        //Update Sweep movement to the least
        Vector3 leftSweepAfterOverlap = clampedSweepMovement;
        Vector3 rightSweepAfterOverlap = clampedSweepMovement;

        if (hasGripped)
        {
            for (int i = 0; i < grippedBlockSweep.Length; i++)
            {
                grippedBlockSweep[i] = clampedSweepMovement;
            }
        }


        //Find possible overlap
        Collider[] internalOverlapCollidersLeft = GripperCollisionOverlap(leftGripperBoxCollider, clampedSweepMovement);

        foreach (Collider overlapCollider in internalOverlapCollidersLeft)
        {
            Vector3 translationDirection = Vector3.zero;
            float translationDistance = 0f;
            //Computer the translation to solve the overlap
            if (Physics.ComputePenetration(leftGripperBoxCollider,
                leftGripper.position + leftSweepAfterOverlap,
                leftGripper.rotation,
                overlapCollider,
                overlapCollider.transform.position,
                overlapCollider.transform.rotation,
                out translationDirection,
                out translationDistance
                ))
            {
                //Translation to solve the overlap
                leftSweepAfterOverlap += translationDirection * (translationDistance + CollisionOffset);
            }
        }

        Collider[] internalOverlapCollidersRight = GripperCollisionOverlap(rightGripperBoxCollider, clampedSweepMovement);

        foreach (Collider overlapCollider in internalOverlapCollidersRight)
        {
            Vector3 translationDirection = Vector3.zero;
            float translationDistance = 0f;
            //Computer the translation to solve the overlap
            if (Physics.ComputePenetration(rightGripperBoxCollider,
                rightGripper.position + rightSweepAfterOverlap,
                rightGripper.rotation,
                overlapCollider,
                overlapCollider.transform.position,
                overlapCollider.transform.rotation,
                out translationDirection,
                out translationDistance
                ))
            {
                //Translation to solve the overlap
                rightSweepAfterOverlap += translationDirection * (translationDistance + CollisionOffset);
            }
        }

        Vector3 sweepMovementAfterOverlap = (leftSweepAfterOverlap.magnitude < rightSweepAfterOverlap.magnitude) ? leftSweepAfterOverlap : rightSweepAfterOverlap;

        if (hasGripped)
        {
            //For gripped objects
            for (int i = 0; i < grippedObjectsBoxCollider.Length; i++)
            {
                if (grippedObjectsBoxCollider[i] != null)
                {
                    Collider[] grippedBlockOverlap = GripperCollisionOverlap(grippedObjectsBoxCollider[i], clampedSweepMovement);

                    foreach (Collider overlapCollider in grippedBlockOverlap)
                    {
                        if (overlapCollider != leftGripperBoxCollider && overlapCollider != rightGripperBoxCollider)
                        {
                            Vector3 translationDirection = Vector3.zero;
                            float translationDistance = 0f;
                            //Computer the translation to solve the overlap
                            if (Physics.ComputePenetration(grippedObjectsBoxCollider[i],
                                grippedObjectsBoxCollider[i].transform.position + grippedBlockSweep[i],
                                grippedObjectsBoxCollider[i].transform.rotation,
                                overlapCollider,
                                overlapCollider.transform.position,
                                overlapCollider.transform.rotation,
                                out translationDirection,
                                out translationDistance
                                ))
                            {
                                //Translation to solve the overlap
                                grippedBlockSweep[i] += translationDirection * (translationDistance + CollisionOffset);
                            }
                        }
                    }
                }
                else
                {
                    grippedBlockSweep[i] = Vector3.positiveInfinity;
                }
            }

            //Find Closest movement again
            grippedObjectSweepMovement = Vector3.positiveInfinity;
            foreach (Vector3 sweep in grippedBlockSweep)
            {
                if (sweep.magnitude < grippedObjectSweepMovement.magnitude)
                {
                    grippedObjectSweepMovement = sweep;
                }
            }

            //Compare to the gripped block and check the closest movement
            sweepMovementAfterOverlap = (sweepMovementAfterOverlap.magnitude < grippedObjectSweepMovement.magnitude) ? sweepMovementAfterOverlap : grippedObjectSweepMovement;

        }

        if (GripperCollisionOverlap(leftGripperBoxCollider, sweepMovementAfterOverlap).Length == 0 && GripperCollisionOverlap(rightGripperBoxCollider, sweepMovementAfterOverlap).Length == 0)
        {
            currentMovementDownward = sweepMovementAfterOverlap;

            if (hasGripped)
            {
                //For gripped objects
                for (int i = 0; i < grippedObjectsBoxCollider.Length; i++)
                {
                    if (grippedObjectsBoxCollider[i] != null)
                    {
                        if (GripperCollisionOverlap(grippedObjectsBoxCollider[i], sweepMovementAfterOverlap).Length != 0)
                        {
                            BoxCollider box = GripperCollisionOverlap(grippedObjectsBoxCollider[i], sweepMovementAfterOverlap)[0].GetComponent<BoxCollider>();

                            ExtDebug.DrawBox(box.transform.position,
                                Vector3.Scale(box.size, box.transform.localScale) / 2f,
                                box.transform.rotation,
                                Color.red);
                            currentMovementDownward = Vector3.zero;
                        }
                    }
                }
            }
        }
        else
        {
            currentMovementDownward = Vector3.zero;
        }

    }

    //Sweep collider to target position to find closest hit
    Vector3 GripperGroundSweep(BoxCollider targetBoxCollider, Vector3 targetMovement, out RaycastHit closestHit)
    {
        Vector3 sweepMovement = Vector3.zero;
        closestHit = new RaycastHit();
        //Store all the raycast hits
        RaycastHit[] allHits = new RaycastHit[MaxBufferSize];

        //Cast box raycast with target movement to find all hits
        int nHits = Physics.BoxCastNonAlloc(targetBoxCollider.transform.position,
            Vector3.Scale(targetBoxCollider.transform.localScale, targetBoxCollider.size) / 2f,
            targetMovement.normalized,
            allHits,
            targetBoxCollider.transform.rotation,
            targetMovement.magnitude + CollisionOffset);

        ExtDebug.DrawBoxCastBox(targetBoxCollider.transform.position,
            Vector3.Scale(targetBoxCollider.size, targetBoxCollider.transform.localScale) / 2f,
            targetBoxCollider.transform.rotation,
            targetMovement.normalized,
            targetMovement.magnitude + CollisionOffset,
            Color.red);

        //Obtain the closet hits
        float closestDistance = Mathf.Infinity;

        for (int i = 0; i < nHits; i++)
        {
            //May consider handling hit another gripper, not likely happens due to small delta movement
            if (allHits[i].distance < closestDistance && allHits[i].distance > 0f && allHits[i].collider != targetBoxCollider)
            {
                if (allHits[i].collider.attachedRigidbody == null || (allHits[i].collider.attachedRigidbody != null && !allHits[i].collider.attachedRigidbody.isKinematic)|| allHits[i].transform.gameObject.layer != grippedBlockLayerMaskIndex)
                {
                    closestHit = allHits[i];
                    closestDistance = allHits[i].distance;
                }

            }
        }
        //If hit something and the closest hit is valid
        if (nHits <= 0 || closestHit.transform == null)
        {
            //Sweep movement will be the same as the target movement if nothing has been hit
            sweepMovement = targetMovement;
        }
        else
        {
            sweepMovement = targetMovement.normalized * (closestHit.distance - CollisionOffset);
        }

        return sweepMovement;
    }

    //Sweep collider to target position to find closest hit
    Vector3 GripperLeftRightSweep(BoxCollider targetBoxCollider, Vector3 targetMovement, out RaycastHit closestHit)
    {
        Vector3 sweepMovement = Vector3.zero;
        closestHit = new RaycastHit();
        //Store all the raycast hits
        RaycastHit[] allHits = new RaycastHit[MaxBufferSize];

        //Cast box raycast with target movement to find all hits
        int nHits = Physics.BoxCastNonAlloc(targetBoxCollider.transform.position,
            Vector3.Scale(targetBoxCollider.transform.localScale, targetBoxCollider.size) / 2f,
            targetMovement.normalized,
            allHits,
            targetBoxCollider.transform.rotation,
            targetMovement.magnitude + CollisionOffset);

        //Obtain the closet hits
        float closestDistance = Mathf.Infinity;

        for (int i = 0; i < nHits; i++)
        {
            //May consider handling hit another gripper, not likely happens due to small delta movement
            if (allHits[i].distance < closestDistance && allHits[i].distance > 0f && allHits[i].collider != targetBoxCollider)
            {
                if (allHits[i].collider.attachedRigidbody == null || (allHits[i].collider.attachedRigidbody != null && !allHits[i].collider.attachedRigidbody.isKinematic))
                {
                    closestHit = allHits[i];
                    closestDistance = allHits[i].distance;
                }

            }
        }

        //If hit something and the closest hit is valid
        if (nHits <= 0 || closestHit.transform == null)
        {
            //Sweep movement will be the same as the target movement if nothing has been hit
            sweepMovement = targetMovement;
        }
        else
        {
            sweepMovement = targetMovement.normalized * (closestHit.distance - CollisionOffset);
        }

        return sweepMovement;
    }

    //Start gripping the block
    void GripperGripping()
    {
        Vector3 leftGripperSweepMovement = Vector3.zero, rightGripperSweepMovement = Vector3.zero;
        float displacement = Time.fixedDeltaTime;

        if (!leftGripperGripped)
        {
            //Declare the direction for gripping and releasing
            Vector3 leftGrippingMovement = Vector3.MoveTowards(leftGripper.position, middlePoint.position, displacement) - leftGripper.position;

            //Find the closest hit
            RaycastHit leftClosestHit;
            leftGripperSweepMovement = GripperLeftRightSweep(leftGripperBoxCollider, leftGrippingMovement, out leftClosestHit);

            //Find possible overlap
            Collider[] internalOverlapCollidersLeft = GripperCollisionOverlap(leftGripperBoxCollider, leftGripperSweepMovement);

            foreach (Collider overlapCollider in internalOverlapCollidersLeft)
            {
                Vector3 translationDirection = Vector3.zero;
                float translationDistance = 0f;
                //Computer the translation to solve the overlap
                if (Physics.ComputePenetration(leftGripperBoxCollider,
                    leftGripper.position + leftGripperSweepMovement,
                    leftGripper.rotation,
                    overlapCollider,
                    overlapCollider.transform.position,
                    overlapCollider.transform.rotation,
                    out translationDirection,
                    out translationDistance
                    ))
                {
                    //Translation to solve the overlap
                    leftGripperSweepMovement += translationDirection * (translationDistance + CollisionOffset);
                }
            }

            if ((leftGripper.position - middlePoint.position).magnitude <= CollisionOffset)
            {
                leftGripperSweepMovement = Vector3.zero;
            }
        }

        if (!rightGripperGripped)
        {
            Vector3 rightGrippingMovement = Vector3.MoveTowards(rightGripper.position, middlePoint.position, displacement) - rightGripper.position;
            lastLeftPos = leftGripper.position;
            lastRightPos = rightGripper.position;
            RaycastHit rightClosestHit;

            //Obtain the sweep movement for both gripper colliders
            rightGripperSweepMovement = GripperLeftRightSweep(rightGripperBoxCollider, rightGrippingMovement, out rightClosestHit);

            Collider[] internalOverlapCollidersRight = GripperCollisionOverlap(rightGripperBoxCollider, rightGripperSweepMovement);

            foreach (Collider overlapCollider in internalOverlapCollidersRight)
            {
                Vector3 translationDirection = Vector3.zero;
                float translationDistance = 0f;
                //Computer the translation to solve the overlap
                if (Physics.ComputePenetration(rightGripperBoxCollider,
                    rightGripper.position + rightGripperSweepMovement,
                    rightGripper.rotation,
                    overlapCollider,
                    overlapCollider.transform.position,
                    overlapCollider.transform.rotation,
                    out translationDirection,
                    out translationDistance
                    ))
                {
                    //Translation to solve the overlap
                    rightGripperSweepMovement += translationDirection * (translationDistance + CollisionOffset);
                }
            }

            if ((rightGripper.position - middlePoint.position).magnitude <= CollisionOffset)
            {
                rightGripperSweepMovement = Vector3.zero;
            }
        }


        //Keep moving until gripped
        if (GripperCollisionOverlap(leftGripperBoxCollider, leftGripperSweepMovement).Length == 0)
        {
            leftGripper.position += leftGripperSweepMovement.normalized * (leftGripperSweepMovement.magnitude - CollisionOffset);
        }
        if (GripperCollisionOverlap(rightGripperBoxCollider, rightGripperSweepMovement).Length == 0)
        {
            rightGripper.position += rightGripperSweepMovement.normalized * (rightGripperSweepMovement.magnitude - CollisionOffset);
        }

        if ((leftGripper.position - lastLeftPos).magnitude < 0.001f && (rightGripper.position - lastRightPos).magnitude < 0.001f)
        {
            gripperState = GripperState.gripped;
        }
    }

    //Start releasing the block
    void GripperReleasing()
    {
        float displacement = Time.fixedDeltaTime;
        Vector3 leftGripperSweepMovement = Vector3.zero, rightGripperSweepMovement = Vector3.zero;

        if (!leftGripperReleased)
        {
            Vector3 leftReleasingMovement = Vector3.MoveTowards(leftGripper.position, leftPoint.position, displacement) - leftGripper.position;

            //Find the closest hit
            RaycastHit leftClosestHit;
            //Obtain the sweep movement for both gripper colliders
            leftGripperSweepMovement = GripperLeftRightSweep(leftGripperBoxCollider, leftReleasingMovement, out leftClosestHit);

            //Calculate overlap and apply force if it overlaps
            Collider[] leftOverlapAfterSweep = GripperCollisionOverlap(leftGripperBoxCollider, leftGripperSweepMovement);
            if (leftOverlapAfterSweep.Length > 0)
            {
                OnMovementHit(leftClosestHit);
            }

            if ((leftGripper.position - leftPoint.position).magnitude <= 0.05f)
            {
                leftGripperReleased = true;
            }
        }

        if (!rightGripperReleased)
        {
            Vector3 rightReleasingMovement = Vector3.MoveTowards(rightGripper.position, rightPoint.position, displacement) - rightGripper.position;

            RaycastHit rightClosestHit;
            rightGripperSweepMovement = GripperCollisionSweep(rightGripperBoxCollider, rightReleasingMovement, out rightClosestHit);

            //Calculate overlap and apply force if it overlaps
            Collider[] rightOverlapAfterSweep = GripperCollisionOverlap(rightGripperBoxCollider, rightGripperSweepMovement);
            if (rightOverlapAfterSweep.Length > 0)
            {
                OnMovementHit(rightClosestHit);
            }

            if ((rightGripper.position - rightPoint.position).magnitude <= 0.05f)
            {
                rightGripperReleased = true;
            }
        }

        if (leftGripperReleased && rightGripperReleased)
        {
            gripperState = GripperState.idle;
        }
        else
        {
            //Keep moving until released
            leftGripper.position += leftGripperSweepMovement.normalized * (leftGripperSweepMovement.magnitude - CollisionOffset);
            rightGripper.position += rightGripperSweepMovement.normalized * (rightGripperSweepMovement.magnitude - CollisionOffset);
        }
    } 

	//Function to make a reference of the gripped objects
	void GripObjects()
	{
        if (hasGripped)
            return;

        //Store all the raycast hits
        RaycastHit[] allHits = new RaycastHit[MaxBufferSize];

		int count = 0;

		Vector3 targetMovement = rightGripper.position - leftGripper.position;

		//Cast box raycast with target movement to find all hits
		Physics.BoxCastNonAlloc(leftGripper.position + targetMovement.normalized* Vector3.Scale(leftGripper.localScale, leftGripperBoxCollider.size).x,
            Vector3.Scale(leftGripper.localScale, leftGripperBoxCollider.size) / 2f,
			targetMovement.normalized,
			allHits,
			leftGripper.transform.rotation,
			CollisionOffset);

        foreach (RaycastHit ray in allHits) 
		{
			if(ray.transform != null)
			{
                if (ray.transform != leftGripper && ray.transform != rightGripper)
                {
                    if (ray.transform.GetComponent<Rigidbody>() != null && ray.transform.GetComponent<BoxCollider>() != null)
                    {
                        if (Vector3.Dot(ray.transform.position - leftGripper.position, Camera.main.transform.right) > 0f && 
                            Vector3.Dot(ray.transform.position - rightGripper.position, Camera.main.transform.right) < 0f)
                        {
                            BoxCollider rayBoxCollider = ray.transform.GetComponent<BoxCollider>();

                            //Still buggy
                            if (Vector3.Project(ray.transform.position - leftGripper.position, Vector3.up).magnitude < (Vector3.Scale(leftGripperBoxCollider.size, leftGripperBoxCollider.transform.localScale).y/2f+ Vector3.Scale(rayBoxCollider.size, ray.transform.localScale).x / 2f) && 
                                Vector3.Project(ray.transform.position - rightGripper.position, Vector3.up).magnitude < (Vector3.Scale(rightGripperBoxCollider.size, rightGripperBoxCollider.transform.localScale).y / 2f + Vector3.Scale(rayBoxCollider.size, ray.transform.localScale).x / 2f))
                            {
                                if (ray.transform.gameObject.layer != grippedBlockLayerMaskIndex)
                                    ray.transform.gameObject.layer = grippedBlockLayerMaskIndex;
                                grippedObjectsBoxCollider[count] = rayBoxCollider;
                                ray.transform.GetComponent<Rigidbody>().isKinematic = true;


                                count++;
                            }
                        }
                    }
                }
			}	
		}

        //From Right to left
        targetMovement = leftGripper.position - rightGripper.position;

        allHits = new RaycastHit[MaxBufferSize];

        //Cast box raycast with target movement to find all hits
        Physics.BoxCastNonAlloc(rightGripper.position + targetMovement.normalized * Vector3.Scale(rightGripper.localScale, rightGripperBoxCollider.size).x,
            Vector3.Scale(rightGripper.localScale, rightGripperBoxCollider.size) / 2f,
            targetMovement.normalized,
            allHits,
            rightGripper.transform.rotation,
            CollisionOffset);

        foreach (RaycastHit ray in allHits)
        {
            bool hasExist = false;
            if (ray.transform != null)
            {
                if (ray.transform != leftGripper && ray.transform != rightGripper)
                {
                    if (ray.transform.GetComponent<Rigidbody>() != null && ray.transform.GetComponent<BoxCollider>() != null)
                    {
                        if (Vector3.Dot(ray.transform.position - leftGripper.position, Camera.main.transform.right) > 0f &&
                            Vector3.Dot(ray.transform.position - rightGripper.position, Camera.main.transform.right) < 0f)
                        {
                            BoxCollider rayBoxCollider = ray.transform.GetComponent<BoxCollider>();
                            if (Vector3.Project(ray.transform.position - leftGripper.position, Vector3.up).magnitude < (Vector3.Scale(leftGripperBoxCollider.size, leftGripperBoxCollider.transform.localScale).y / 2f + Vector3.Scale(rayBoxCollider.size, ray.transform.localScale).x / 2f) &&
                                Vector3.Project(ray.transform.position - rightGripper.position, Vector3.up).magnitude < (Vector3.Scale(rightGripperBoxCollider.size, rightGripperBoxCollider.transform.localScale).y / 2f + Vector3.Scale(rayBoxCollider.size, ray.transform.localScale).x / 2f))
                            {
                                for (int i = 0; i < grippedObjectsBoxCollider.Length; i++)
                                {
                                    if (grippedObjectsBoxCollider[i] == rayBoxCollider)
                                    {
                                        hasExist = true;
                                    }
                                }

                                if (!hasExist)
                                {
                                    if (ray.transform.gameObject.layer != grippedBlockLayerMaskIndex)
                                        ray.transform.gameObject.layer = grippedBlockLayerMaskIndex;
                                    grippedObjectsBoxCollider[count] = rayBoxCollider;
                                    ray.transform.GetComponent<Rigidbody>().isKinematic = true;

                                    count++;
                                }

                            }
                        }
                    }
                }
            }
        }

        bool gripped = false;
        foreach (BoxCollider b in grippedObjectsBoxCollider)
        {
            if (b != null)
            {
                gripped = true;
            }
        }

        if (gripped)
        {
            gripCount++;
        }
        else
        {
            SceneController.gripNothing = true;
        }

        hasGripped = true;
    }

    //Function to release the block
    void ReleaseObjects()
    {
        if (!hasGripped)
            return;

        bool gripped = false;
        foreach (BoxCollider b in grippedObjectsBoxCollider)
        {
            if (b != null)
            {
                gripped = true;
            }
        }

        if (gripped)
        {
            gripCount++;
        }

        foreach (BoxCollider t in grippedObjectsBoxCollider)
        {
            if (t != null)
            {
                if (t.GetComponent<Rigidbody>() != null)
                {
                    Rigidbody rb = t.GetComponent<Rigidbody>();
                    if (t.gameObject.layer != blockLayerMaskIndex)
                        t.gameObject.layer = blockLayerMaskIndex;



                    if (rb.GetComponent<Block>() != null)
                    {
                        if (rb.GetComponent<Block>().GetId() == HighlightBlock.highlightedBlockId)
                        {

                            HighlightBlock.allowSwitch = true;
                            //Debug.Log("Allow switch!"); 
                            //SceneController.isTaskFinished = true;

                            StartCoroutine(StepIncrement());
                        }
                    }

                    rb.isKinematic = false;
                }
            }
        }

        grippedObjectsBoxCollider = new BoxCollider[MaxBufferSize];
        SceneController.gripNothing = false;
        hasGripped = false;
    }

    void DebugGrippedObjects()
    {
        for (int i = 0; i < grippedObjectsBoxCollider.Length; i++)
        {
            if (grippedObjectsBoxCollider[i] != null)
            {
                ExtDebug.DrawBox(grippedObjectsBoxCollider[i].transform.position,
                    Vector3.Scale(grippedObjectsBoxCollider[i].size, grippedObjectsBoxCollider[i].transform.localScale) / 2f,
                    grippedObjectsBoxCollider[i].transform.rotation,
                    Color.red);
            }

        }
    }

    //Used fot reset the gripper
    public void ResetMovement()
    {
        leftPoint.position = originLeftPointPos;
        rightPoint.position = originRightPointPos;
        middlePoint.position = originMiddlePointPos;
        centralPoint.position = originCentralPointPos;

        leftGripper.position = originLeftGripperPos;
        rightGripper.position = originRightGripperPos;

        ReleaseObjects();
        gripperState = GripperState.idle;

        //Update rotation
        transform.rotation = Quaternion.Euler(0,0,0);

        gripCount = 0;
    }

    //increase step by 1 after 0.5 second
    IEnumerator StepIncrement()
    {
        yield return new WaitForSeconds(0.5f);

        //SceneController.stepCount++;
    }

}
