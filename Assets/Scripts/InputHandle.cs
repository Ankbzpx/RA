using UnityEngine;

public class InputHandle : MonoBehaviour {

    public int auto_axisX, auto_axisY, auto_axisZ, auto_axisW, auto_space;
    public int isTraining;

    [SerializeField]
    TargetPositionControl targetPosControl;

    int axisX, axisY, axisZ, axisW, space;

    // Update is called once per frame
    void Update()
    {
        //Apply input from player
        if (Input.GetKey(KeyCode.Q) || Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.E) || Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.S) ||
            Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.Z) || Input.GetKey(KeyCode.X) || Input.GetKey(KeyCode.Space))
        {
            isTraining = 1;
            ManualInput();

            FileManager.RecordInstruction(axisX, axisY, axisZ, axisW, space);

            if (FileManager.isAllowSampling == false)
                FileManager.isAllowSampling = true;


            auto_axisX = axisX;
            auto_axisY = axisY;
            auto_axisZ = axisZ;
            auto_axisW = axisW;
            auto_space = space;

            targetPosControl.ProcessInput(axisX, axisY, axisZ, axisW, space);


        }
        //Apply input from server
        else
        {
            if (FileManager.isAllowSampling == true)
                FileManager.isAllowSampling = false;

            isTraining = 0;
            targetPosControl.ProcessInput(auto_axisX, auto_axisY, auto_axisZ, auto_axisW, auto_space);
        }

        DebugPanel.UpdateDebugText(auto_axisX, auto_axisY, auto_axisZ, auto_axisW, auto_space);

        ResetAutoInput();
    }

    void ManualInput()
    {
        //Limit the input to the binary for classification purpose
        float tempX, tempZ, tempY, tempW;

        tempX = Input.GetAxis("Axis X");
        tempZ = Input.GetAxis("Axis Z");
        tempY = Input.GetAxis("Axis Y");
        tempW = Input.GetAxis("Axis W");


        if (Input.GetKeyDown(KeyCode.Space))
        {
            space = 1;
        }
        else
        {
            space = 0;
        }
        if (tempX > 0f)
        {
            axisX = 1;
        }
        else if (tempX < 0f)
        {
            axisX = -1;
        }

        if (tempZ > 0f)
        {
            axisZ = 1;
        }
        else if (tempZ < 0f)
        {
            axisZ = -1;
        }

        if (tempY > 0f)
        {
            axisY = 1;
        }
        else if (tempY < 0f)
        {
            axisY = -1;
        }

        if (tempW > 0f)
        {
            axisW = 1;
        }
        else if (tempW < 0f)
        {
            axisW = -1;
        }

    }

    void ResetAutoInput()
    {
        axisX = 0;
        axisY = 0;
        axisZ = 0;
        axisW = 0;
        space = 0;

        auto_axisX = 0;
        auto_axisY = 0;
        auto_axisZ = 0;
        auto_axisW = 0;
        auto_space = 0;
    }
}
