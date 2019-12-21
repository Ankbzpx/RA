using UnityEngine;

public class HighlightBlock : MonoBehaviour {

    //enum Direction { Left, Right, Up, Down };

    public static bool allowSwitch = true;

    public static int highlightedBlockId = 0;

    int tempHighlightedBlockId;

    //Block highlightedBlock;

    //List<Block> potentialBlocks;

    //float longPressTime;
    //float switchCubeTime;
    //float blockThreshold = 0.25f;
    //float heightThreshold = 0.125f;

    //float H_input, V_input;

    void Update()
    {
        //SelectBlock();
        if (allowSwitch)
        {
            if (highlightedBlockId != 0)
            {
                tempHighlightedBlockId = highlightedBlockId;
            }

            int temp = SceneController.RandomExtractableBlockID();

            while (temp == tempHighlightedBlockId && temp == 0)
            {
                temp = SceneController.RandomExtractableBlockID();
            }

            highlightedBlockId = temp;
            allowSwitch = false;
        }

        if (!SceneController.CheckExtractable(highlightedBlockId))
        {
            allowSwitch = true;
        }
    }

    ////the function for the player to select the cube
    //void SelectBlock()
    //{
    //    if (highlightedBlockId == 0)
    //    {
    //        H_input = 0;
    //        V_input = 0;

    //        if (Input.GetKey(KeyCode.J))
    //        {
    //            H_input = -1;
    //        }
    //        else if (Input.GetKey(KeyCode.L))
    //        {
    //            H_input = 1;
    //        }
    //        else if (Input.GetKey(KeyCode.I))
    //        {
    //            V_input = 1;
    //        }
    //        else if (Input.GetKey(KeyCode.K))
    //        {
    //            V_input = -1;
    //        }

    //        Debug.Log("V_input :" + V_input);
    //        Debug.Log("H_input :" + H_input);


    //        int tempHighlightID = highlightedBlockId;

    //        if (H_input != 0f || V_input != 0f)
    //        {
    //            //longPressTime += Time.deltaTime;

    //            //if (longPressTime > 0.3f)
    //                allowSwitch = true;

    //            Debug.Log("Allow switch");
    //        }
    //        //else
    //        //{
    //        //    longPressTime = 0f;
    //        //}

    //        if (allowSwitch)
    //        {
    //            //switchCubeTime += Time.deltaTime;

    //            //if (switchCubeTime < 0.1f)
    //            //    return;

    //            if (H_input != 0f || V_input != 0f)
    //            {
    //                if (H_input == 1)
    //                {
    //                    SwitchCubes(Direction.Up);
    //                }
    //                else if (H_input == -1)
    //                {
    //                    SwitchCubes(Direction.Down);
    //                }
    //                else if (V_input == 1)
    //                {
    //                    SwitchCubes(Direction.Right);
    //                }
    //                else if (V_input == -1)
    //                {
    //                    SwitchCubes(Direction.Left);
    //                }

    //                //if (Mathf.Abs(H_input) >= Mathf.Abs(V_input))
    //                //{
    //                //    //left
    //                //    if (H_input <= 0)
    //                //    {

    //                //        //function to switch highlighted cube
    //                //        SwitchCubes(Direction.Left);
    //                //    }
    //                //    //right
    //                //    else if (H_input > 0)
    //                //    {

    //                //        //function to switch highlighted cube
    //                //        SwitchCubes(Direction.Right);
    //                //    }
    //                //}
    //                //else if (Mathf.Abs(H_input) < Mathf.Abs(V_input))
    //                //{
    //                //    //up
    //                //    if (V_input >= 0)
    //                //    {

    //                //        //function to switch highlighted cube
    //                //        SwitchCubes(Direction.Up);
    //                //    }
    //                //    //down
    //                //    else if (V_input < 0)
    //                //    {

    //                //        //function to switch highlighted cube
    //                //        SwitchCubes(Direction.Down);
    //                //    }
    //                //}

    //            }

    //            if (highlightedBlockId != tempHighlightID)
    //            {
    //                allowSwitch = false;
    //                //switchCubeTime = 0;
    //            }
    //        }
    //        else
    //        {
    //            if (H_input == 0f && V_input == 0f)
    //            {
    //                allowSwitch = true;
    //            }
    //        }
    //    }
    //}


    //void SwitchCubes(Direction _dir)
    //{
    //    if (highlightedBlockId == 0)
    //        return;

    //    highlightedBlock = SceneController.GetBlock(highlightedBlockId);

    //    potentialBlocks = new List<Block>();

    //    if (SceneController.blocks.Values.Count != 0)
    //    {
    //        switch (_dir)
    //        {
    //            case Direction.Left:

    //                foreach (Block _block in SceneController.blocks.Values)
    //                {
    //                    Vector3 _cDir = (_block.transform.position - highlightedBlock.transform.position).normalized;

    //                    if (Vector3.Dot(_cDir, new Vector3(-Camera.main.transform.right.x, 0, -Camera.main.transform.right.z).normalized) > 0f)
    //                    {
    //                        if (Mathf.Abs(_block.transform.position.y - highlightedBlock.transform.position.y) <= blockThreshold)
    //                        {
    //                            potentialBlocks.Add(_block);
    //                        }
    //                    }
    //                }
    //                highlightedBlockId = GetClosestCubeID(potentialBlocks);
    //                break;

    //            case Direction.Right:

    //                foreach (Block _block in SceneController.blocks.Values)
    //                {
    //                    Vector3 _cDir = (_block.transform.position - highlightedBlock.transform.position).normalized;

    //                    if (Vector3.Dot(_cDir, new Vector3(Camera.main.transform.right.x, 0, Camera.main.transform.right.z).normalized) > 0f)
    //                    {
    //                        if (Mathf.Abs(_block.transform.position.y - highlightedBlock.transform.position.y) <= blockThreshold)
    //                        {
    //                            potentialBlocks.Add(_block);
    //                        }
    //                    }
    //                }

    //                highlightedBlockId = GetClosestCubeID(potentialBlocks);

    //                break;
    //            case Direction.Up:

    //                foreach (Block _cube in SceneController.blocks.Values)
    //                {
    //                    Vector3 _cDir = (_cube.transform.position - highlightedBlock.transform.position).normalized;

    //                    if (Vector3.Dot(_cDir, Vector3.up) > 0f)
    //                    {
    //                        if (Mathf.Abs(_cube.transform.position.y - highlightedBlock.transform.position.y) > blockThreshold && Mathf.Abs(_cube.transform.position.y - highlightedBlock.transform.position.y) < heightThreshold)
    //                        {
    //                            potentialBlocks.Add(_cube);
    //                        }
    //                    }
    //                }

    //                if (potentialBlocks.Count == 0)
    //                {
    //                    foreach (Block _block in SceneController.blocks.Values)
    //                    {
    //                        Vector3 _cDir = (_block.transform.position - highlightedBlock.transform.position).normalized;

    //                        if (Vector3.Dot(_cDir, Vector3.up) > 0f)
    //                        {
    //                            if (Mathf.Abs(_block.transform.position.y - highlightedBlock.transform.position.y) > blockThreshold)
    //                            {
    //                                potentialBlocks.Add(_block);
    //                            }
    //                        }
    //                    }
    //                }

    //                highlightedBlockId = GetClosestCubeID(potentialBlocks, true);

    //                break;
    //            case Direction.Down:

    //                foreach (Block _block in SceneController.blocks.Values)
    //                {
    //                    Vector3 _cDir = (_block.transform.position - highlightedBlock.transform.position).normalized;

    //                    if (Vector3.Dot(_cDir, Vector3.down) > 0f)
    //                    {
    //                        if (Mathf.Abs(_block.transform.position.y - highlightedBlock.transform.position.y) > blockThreshold && Mathf.Abs(_block.transform.position.y - highlightedBlock.transform.position.y) < heightThreshold)
    //                        {
    //                            potentialBlocks.Add(_block);
    //                        }
    //                    }
    //                }

    //                if (potentialBlocks.Count == 0)
    //                {
    //                    foreach (Block _block in SceneController.blocks.Values)
    //                    {
    //                        Vector3 _cDir = (_block.transform.position - highlightedBlock.transform.position).normalized;

    //                        if (Vector3.Dot(_cDir, Vector3.down) > 0f)
    //                        {
    //                            if (Mathf.Abs(_block.transform.position.y - highlightedBlock.transform.position.y) > blockThreshold)
    //                            {
    //                                potentialBlocks.Add(_block);
    //                            }
    //                        }
    //                    }
    //                }

    //                highlightedBlockId = GetClosestCubeID(potentialBlocks, true);

    //                break;
    //            default:
    //                break;
    //        }
    //    }

    //    potentialBlocks.Clear();
    //}

    //int GetClosestCubeID(List<Block> _potentialBlocks, bool isNearCamera = false)
    //{
    //    if (_potentialBlocks.Count == 0)
    //        return highlightedBlockId;

    //    int id = highlightedBlockId;

    //    float minDistance = 50f;

    //    if (isNearCamera)
    //    {
    //        for (int i = 0; i < _potentialBlocks.Count; i++)
    //        {
    //            Vector3 refPoint = new Vector3(Camera.main.transform.position.x, _potentialBlocks[i].transform.position.y, Camera.main.transform.position.z);

    //            float _tempDis = (_potentialBlocks[i].transform.position - highlightedBlock.transform.position).magnitude + 2 * (refPoint - _potentialBlocks[i].GetComponent<Collider>().ClosestPoint(refPoint)).magnitude;
    //            if (_tempDis < minDistance && highlightedBlockId != _potentialBlocks[i].transform.GetComponent<Block>().GetId())
    //            {
    //                id = _potentialBlocks[i].transform.GetComponent<Block>().GetId();
    //                minDistance = _tempDis;
    //            }
    //        }
    //    }
    //    else
    //    {
    //        for (int i = 0; i < _potentialBlocks.Count; i++)
    //        {
    //            float _tempDis = (_potentialBlocks[i].transform.position - highlightedBlock.transform.position).magnitude;
    //            if (_tempDis < minDistance && highlightedBlockId != _potentialBlocks[i].transform.GetComponent<Block>().GetId())
    //            {
    //                id = _potentialBlocks[i].transform.GetComponent<Block>().GetId();
    //                minDistance = _tempDis;
    //            }
    //        }
    //    }


    //    return id;
    //}

}
