using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class FileManager : MonoBehaviour {

    public const string CSVFileName = "RA_log.csv";
    public const string DirFrames = "IMG";
    private string m_saveLocation = "";

    static int axisX, axisY, axisZ, axisW, space;
    static string currentTimeStamp;
    private Queue<Sample> samples;
    //private Queue<Texture2D> leftCameraTextures;
    private Queue<Texture2D> frontCameraTextures;
    private Queue<Texture2D> gripperCameraTextures;

    public static bool isAllowSampling = false;
    public static int timestepCurrentStep = 0;
    int cachedSteps = 0;
    bool firstRecord = false;


    private int TotalSamples;
    private bool isSaving;

    //[SerializeField]
    //Camera leftCamera;
    [SerializeField]
    Camera frontCamera;
    [SerializeField]
    Camera gripperCamera;
    [SerializeField]
    RewardCalculation rewardCalculation;

    private bool m_isRecording = false;
    public bool IsRecording
    {
        get
        {
            return m_isRecording;
        }
        set
        {
            m_isRecording = value;
            if (value == true)
            {
                Debug.Log("Starting to record");
                if (!firstRecord)
                {
                    SceneController.stepCount = 0;
                    firstRecord = true;
                }
                samples = new Queue<Sample>();
                //leftCameraTextures = new Queue<Texture2D>();
                frontCameraTextures = new Queue<Texture2D>();
                gripperCameraTextures = new Queue<Texture2D>();
                StartCoroutine(Sampling());
            }
            else if(value == false)
            {
                Debug.Log("Stopping record");
                StopCoroutine(Sampling());
                Debug.Log("Writing to disk");
                //see how many samples we captured use this to show save percentage in UISystem script
                TotalSamples = samples.Count;
                isSaving = true;
                StartCoroutine(WriteSamplesToDisk());
            };
        }
    }

    private void Update()
    {
        if (samples != null)
        {
            if (samples.Count >= 5000)
            {
                IsRecording = false;
            }
        }

    }

    public bool CheckSaveLocation()
    {
        if (m_saveLocation != "")
        {
            return true;
        }
        else
        {
            SimpleFileBrowser.ShowSaveDialog(OpenFolder, null, true, null, "Select Output Folder", "Select");
        }
        return false;
    }



    public static void RecordInstruction(int _axisX, int _axisY, int _axisZ, int _axisW, int _space)
    {
        axisX = _axisX;
        axisY = _axisY;
        axisZ = _axisZ;
        axisW = _axisW;
        space = _space;
    }

    internal class Sample
    {
        //public int axisX, axisY, axisZ, axisW, space;
        public float rotateAngle, translationXDistance, translationYDistance;
        public int step, timestepCurrentStep;
    }

    //Changed the WriteSamplesToDisk to a IEnumerator method that plays back recording along with percent status from UISystem script 
    //instead of showing frozen screen until all data is recorded
    public IEnumerator WriteSamplesToDisk()
    {
        yield return new WaitForSeconds(0.000f); //retrieve as fast as we can but still allow communication of main thread to screen and UISystem
        if (samples.Count > 0)
        {
            //pull off a sample from the que
            Sample sample = samples.Dequeue();

            // Capture and Persist Image
            //string frontImg = WriteImage(leftCameraTextures.Dequeue(), "left", sample.step + "_" + sample.timestepCurrentStep);
            string frontImg = WriteImage(frontCameraTextures.Dequeue(), "front", sample.step + "_" + sample.timestepCurrentStep);
            string gripperImg = WriteImage(gripperCameraTextures.Dequeue(), "gripper", sample.step + "_" + sample.timestepCurrentStep);

            string row = $"{sample.timestepCurrentStep}, {frontImg}, {gripperImg}," +
                $" {sample.rotateAngle}, {sample.translationXDistance},{sample.translationYDistance},{sample.step}\n";/*,{ sample.axisZ}, {sample.axisW},{sample.space},{sample.step}*/
            File.AppendAllText(Path.Combine(m_saveLocation, CSVFileName), row);
        }
        if (samples.Count > 0)
        {
            //request if there are more samples to pull
            StartCoroutine(WriteSamplesToDisk());
        }
        else
        {
            //all samples have been pulled
            StopCoroutine(WriteSamplesToDisk());
            isSaving = false;
        }
    }

    public float GetSavePercent()
    {
        return (float)(TotalSamples - samples.Count) / TotalSamples;
    }

    public bool GetSaveStatus()
    {
        return isSaving;
    }

    public int GetSampleCount()
    {
        if (samples == null)
        {
            return 0;
        }
        else
        {
            return samples.Count;
        }
    }


    public IEnumerator Sampling()
    {
        // Start the Coroutine to Capture Data Every Second.
        // Persist that Information to a CSV and Perist the Camera Frame
        yield return new WaitForSeconds(0.0666666666666667f);

        if (isAllowSampling)
        {
            if (cachedSteps != SceneController.stepCount)
            {
                timestepCurrentStep = 0;
                cachedSteps = SceneController.stepCount;
            }

            //Record time steps
            timestepCurrentStep++;

            if (m_saveLocation != "")
            {
                Sample sample = new Sample
                {
                    //axisX = axisX,
                    //axisY = axisY,
                    //axisZ = axisZ,
                    //axisW = axisW,
                    //space = (space == 1) ? 1 : 0,
                    rotateAngle = rewardCalculation.ObtainRotateAngle(),
                    translationXDistance = rewardCalculation.ObtainRotateAngle() <= 5f ? rewardCalculation.ObtainXTranslationDistance() : -1f,
                    translationYDistance = rewardCalculation.ObtainRotateAngle() <= 5f ? rewardCalculation.ObtainYTranslationDistance() : -1f,
                    step = cachedSteps,
                    timestepCurrentStep = timestepCurrentStep,
                };

                samples.Enqueue(sample);
                //leftCameraTextures.Enqueue(GetCameraTexture(leftCamera));
                frontCameraTextures.Enqueue(GetCameraTexture(frontCamera));
                gripperCameraTextures.Enqueue(GetCameraTexture(gripperCamera));

                sample = null;
            }
        }

        // Only reschedule if the button hasn't toggled
        if (IsRecording)
        {
            StartCoroutine(Sampling());

        }
    }

    private void OpenFolder(string location)
    {
        m_saveLocation = location;
        Directory.CreateDirectory(Path.Combine(m_saveLocation, DirFrames));
    }

    private string WriteImage(Texture2D texture2D, string prepend, string timestamp)
    {
        //needed to force camera update 
        //camera.Render();
        //RenderTexture targetTexture = camera.targetTexture;
        //RenderTexture.active = targetTexture;
        //Texture2D texture2D = new Texture2D(targetTexture.width, targetTexture.height, TextureFormat.RGB24, false);
        //texture2D.ReadPixels(new Rect(0, 0, texture2D.width, texture2D.height), 0, 0);
        //texture2D.Apply();
        byte[] image = texture2D.EncodeToJPG();
        UnityEngine.Object.DestroyImmediate(texture2D);
        string directory = Path.Combine(m_saveLocation, DirFrames);
        string path = Path.Combine(directory, prepend + "_" + timestamp + ".jpg");
        File.WriteAllBytes(path, image);
        image = null;
        return path;
    }

    Texture2D GetCameraTexture(Camera camera)
    {
        camera.Render();
        RenderTexture targetTexture = camera.targetTexture;
        RenderTexture.active = targetTexture;
        Texture2D texture2D = new Texture2D(targetTexture.width, targetTexture.height, TextureFormat.RGB24, false);
        texture2D.ReadPixels(new Rect(0, 0, texture2D.width, texture2D.height), 0, 0);
        texture2D.Apply();
        return texture2D;
    }
}

