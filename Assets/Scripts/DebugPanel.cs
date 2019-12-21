using UnityEngine;
using UnityEngine.UI;

public class DebugPanel : MonoBehaviour {

    [SerializeField]
    InputHandle inputHandle;

    [SerializeField]
    Text XValueText;

    [SerializeField]
    Text YValueText;

    [SerializeField]
    Text ZValueText;

    [SerializeField]
    Text WValueText;

    [SerializeField]
    Text SPACEValueText;

    static int x, y, z, w, space;

    // Update is called once per frame
    void Update ()
    {
        XValueText.text = x.ToString();
        YValueText.text = y.ToString();
        ZValueText.text = z.ToString();
        WValueText.text = w.ToString();
        SPACEValueText.text = space.ToString();
    }

    public static void UpdateDebugText(int _x, int _y, int _z, int _w, int _space)
    {
        x = _x;
        y = _y;
        z = _z;
        w = _w;
        space = _space;
    }

}
