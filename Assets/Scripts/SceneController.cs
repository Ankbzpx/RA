using System.Linq;
using System.Collections.Generic;
using UnityEngine;

public class SceneController : MonoBehaviour {

    public static Dictionary<int, Block> blocks = new Dictionary<int, Block>();
    static Dictionary<int, Block> extractableBlocks = new Dictionary<int, Block>();
    static Dictionary<int, Block> onGroundBlocks = new Dictionary<int, Block>();
    public static int blockNum = 48;
    static Vector3[] blockPositions = new Vector3[blockNum];
    static Quaternion[] blockRotations = new Quaternion[blockNum];
    public static int blockOnGround;
    static int length;

    public static bool isTaskFinished = false;
    public static bool gripNothing = false;

    public static int stepCount = 0;

    [SerializeField]
    HeadControl headController;
    [SerializeField]
    TargetPositionControl targetPosControl;
    [SerializeField]
    FABRIK fabrik;

    static HeadControl _headController;
    static TargetPositionControl _targetPosControl;
    static FABRIK _fabrik;

    void Start()
    {
        _headController = headController;
        _targetPosControl = targetPosControl;
        _fabrik = fabrik;

        blockOnGround = 0;
    }

    public static void RegisterBlock(int _id, Block _block, Vector3 originalPos, Quaternion originalRot)
    {
        blocks.Add(_id, _block);
        extractableBlocks.Add(_id, _block);
        blockPositions[_id - 1] = originalPos;
        blockRotations[_id - 1] = originalRot;
    }

    public static Block GetBlock(int _id)
    {
        return blocks[_id];
    }

    public static int RandomExtractableBlockID()
    {
        if (extractableBlocks.Count == 0)
            return 0;
        else
        {
            System.Random rand = new System.Random();
            return extractableBlocks.ElementAt(rand.Next(0, extractableBlocks.Count)).Key;
        }
    }

    public static void SetNonExtractable(int _id)
    {
        if (extractableBlocks.ContainsKey(_id))
        {
            extractableBlocks.Remove(_id);
        }
    }

    public static bool CheckExtractable(int _id)
    {
        return extractableBlocks.ContainsKey(_id);
    }

    public static void BlockDropGround(int _id)
    {
        if (!onGroundBlocks.ContainsKey(_id))
        {
            onGroundBlocks.Add(_id, blocks[_id]);
        }

        blockOnGround = onGroundBlocks.Count;
    }

    public static void ResetAllBlocks()
    {

        HighlightBlock.highlightedBlockId = 0;

        extractableBlocks.Clear();
        onGroundBlocks.Clear();

        foreach (int key in blocks.Keys)
        {
            extractableBlocks.Add(key, GetBlock(key));
        }

        for (int i = 0; i < blockNum; i++)
        {
            if (blocks[i + 1] != null)
            {
                Rigidbody rb = blocks[i + 1].GetComponent<Rigidbody>();
                Transform tf = blocks[i + 1].transform;

                //necessary?
                rb.isKinematic = true;

                tf.position = blockPositions[i];
                tf.rotation = blockRotations[i];
                rb.isKinematic = false;
                rb.velocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
        }

        blockOnGround = 3;

    }

    public static void ResetRoboticArm()
    {
        _targetPosControl.transform.localPosition = new Vector3(0, 5, 4);
        _fabrik.RecordJointPosi();
        _headController.ResetMovement();
        isTaskFinished = false;
        gripNothing = false;

        stepCount++;

    }


}
