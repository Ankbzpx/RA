using UnityEngine;
using UnityEngine.UI;

public class UIControl : MonoBehaviour {

    private bool recording = false;
    private bool saveRecording;
    [SerializeField]
    FileManager fileManager;
    [SerializeField]
    Text percentageText;
    [SerializeField]
    Text countText;
    [SerializeField]
    Text stepCountText;
    [SerializeField]
    Text timestepCountText;

    public void Restart()
    {
        SceneController.ResetAllBlocks();
        SceneController.ResetRoboticArm();
    }

    public void Quit()
    {
        Application.Quit();
    }

    public void Record()
    {
        if (!recording)
        {
            if (fileManager.CheckSaveLocation())
            {
                recording = true;
                percentageText.text = "Recording...";
                fileManager.IsRecording = true;
            }
        }
        else
        {
            saveRecording = true;
            fileManager.IsRecording = false;
        }
    }

    private void Update()
    {
        if (!recording)
        {
            if (!fileManager.GetSaveStatus())
            {
                percentageText.text = "";
            }
        }
        else if (fileManager.GetSaveStatus())
        {
            percentageText.text = "Saving  " + ((int)(100 * fileManager.GetSavePercent())).ToString() + "%";
            //SaveStatus_Text.text = "Capturing Data: " + (int)(100 * fileManager.GetSavePercent()) + "%";
            Debug.Log("save percent is: " + fileManager.GetSavePercent());
        }
        else if (saveRecording)
        {
            recording = false;
            saveRecording = false;
        }

        countText.text = fileManager.GetSampleCount().ToString();

        stepCountText.text = SceneController.stepCount.ToString();

        timestepCountText.text = FileManager.timestepCurrentStep.ToString();
    }
}
