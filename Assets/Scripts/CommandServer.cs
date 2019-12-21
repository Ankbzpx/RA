using UnityEngine;
using System.Collections.Generic;
using System;
using SocketIO;

public class CommandServer : MonoBehaviour
{
	public Camera frontCamera;
    public Camera upperCamera;
    public Camera playerCamera;
	private SocketIOComponent _socket;
    static int count = 0;

    [SerializeField]
    InputHandle inputHandle;

    [SerializeField]
    RewardCalculation rewardCalculation;

	// Use this for initialization
	void Start()
	{
		_socket = GameObject.Find("SocketIO").GetComponent<SocketIOComponent>();
		_socket.On("open", OnOpen);
        _socket.On("auto", OnAuto);
        _socket.On("manual", OnManual);
    }

	// Update is called once per frame
	void Update()
	{

    }

	void OnOpen(SocketIOEvent obj)
	{
		Debug.Log("Connection Open");
		EmitTelemetry(obj);
	}

	// 
	void OnManual(SocketIOEvent obj)
	{
		EmitTelemetry (obj);
	}

	void OnAuto(SocketIOEvent obj)
	{
		JSONObject jsonObject = obj.data;

        //Receive data from server
        //float auto_X,auto_Y, auto_Z, auto_W, auto_Space;

        float rotAngle, translationDistance;

        count++;

        float.TryParse(jsonObject.GetField("RotAngle").str, out rotAngle);
        float.TryParse(jsonObject.GetField("TransDis").str, out translationDistance);

        Debug.Log("Rotate Angle: " + rotAngle);
        Debug.Log("Translation Distance: " + translationDistance);

        //Receive 9 binary output
        //float.TryParse(jsonObject.GetField("X").str, out auto_X);
        //float.TryParse(jsonObject.GetField("Y").str, out auto_Y);
        //float.TryParse(jsonObject.GetField("Z").str, out auto_Z);
        //float.TryParse(jsonObject.GetField("W").str, out auto_W);
        //float.TryParse(jsonObject.GetField("SPACE").str, out auto_Space);

        //inputHandle.auto_space = auto_Space > 0.5f? 1 : 0;
        //inputHandle.auto_axisX = auto_X > 0.5f ? 1:0;
        //inputHandle.auto_axisY = auto_Y > 0.5f ? 1 : 0;
        //inputHandle.auto_axisZ = auto_Z > 0.5f ? 1 : 0;
        //inputHandle.auto_axisW = auto_W > 0.5f ? 1 : 0;

        //inputHandle.auto_axisX = auto_X < -0.5f ? -1 : 0;
        //inputHandle.auto_axisY = auto_Y < -0.5f ? -1 : 0;
        //inputHandle.auto_axisZ = auto_Z < -0.5f ? -1 : 0;
        //inputHandle.auto_axisW = auto_W < -0.5f ? -1 : 0;

        EmitTelemetry(obj);
	}

	void EmitTelemetry(SocketIOEvent obj)
	{
        UnityMainThreadDispatcher.Instance().Enqueue(() =>
        {
            // Collect Data from the Car
            Dictionary<string, string> data = new Dictionary<string, string>
            {
                //{ "X", inputHandle.auto_axisX.ToString("N4") },
                //{ "Y", inputHandle.auto_axisY.ToString("N4") },
                //{ "Z", inputHandle.auto_axisZ.ToString("N4") },
                //{ "W", inputHandle.auto_axisW.ToString("N4") },
                //{ "SPACE", inputHandle.auto_space.ToString("N4") },

                { "RotAngle", rewardCalculation.ObtainRotateAngle().ToString("N4") },
                { "TransDis", rewardCalculation.ObtainXTranslationDistance().ToString("N4") },

                { "ISTRAINING", inputHandle.isTraining.ToString("N4") },
                { "FRONT", Convert.ToBase64String(CameraHelper.CaptureFrame(frontCamera)) },
                { "UPPER", Convert.ToBase64String(CameraHelper.CaptureFrame(upperCamera)) },
                { "PLAYER", Convert.ToBase64String(CameraHelper.CaptureFrame(playerCamera)) }
            };

            _socket.Emit("telemetry", new JSONObject(data));

        });

        //    UnityMainThreadDispatcher.Instance().Enqueue(() =>
        //    {
        //      	
        //      
        //
        //		// send only if it's not being manually driven
        //		if ((Input.GetKey(KeyCode.W)) || (Input.GetKey(KeyCode.S))) {
        //			_socket.Emit("telemetry", new JSONObject());
        //		}
        //		else {
        //			// Collect Data from the Car
        //			Dictionary<string, string> data = new Dictionary<string, string>();
        //			data["steering_angle"] = _carController.CurrentSteerAngle.ToString("N4");
        //			data["throttle"] = _carController.AccelInput.ToString("N4");
        //			data["speed"] = _carController.CurrentSpeed.ToString("N4");
        //			data["image"] = Convert.ToBase64String(CameraHelper.CaptureFrame(FrontFacingCamera));
        //			_socket.Emit("telemetry", new JSONObject(data));
        //		}
        //      
        ////      
        //    });
    }
}