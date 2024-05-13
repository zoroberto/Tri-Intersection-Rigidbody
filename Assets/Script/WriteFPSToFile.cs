using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;

public class WriteFPSToFile : MonoBehaviour
{
    [Header("Compute FPS")]
    public string fileNameFPS = "fps.csv";
    public int MaxFrame = 400;

    private int frame = 0;
    private List<float> arrFps = new List<float>();

    private float deltatime = 0.0f;
    private float avgFPS = 0.0f;

    void writeCSVfps()
    {

        List<string[]> rowData = new List<string[]>();
        string[] rowDataTemp = new string[2];
        rowDataTemp[0] = "#frame";
        rowDataTemp[1] = "fps";
        rowData.Add(rowDataTemp);

        for (int i = 0; i < arrFps.Count; i++)
        {

            rowDataTemp = new string[2];
            rowDataTemp[0] = i.ToString();
            rowDataTemp[1] = arrFps[i].ToString();
            rowData.Add(rowDataTemp);
        }
        string[][] output = new string[rowData.Count][];

        for (int i = 0; i < output.Length; i++)
        {
            output[i] = rowData[i];
        }
        int length = output.GetLength(0);
        string delimiter = ",";

        StringBuilder sb = new StringBuilder();

        for (int index = 0; index < length; index++)
            sb.AppendLine(string.Join(delimiter, output[index]));
        string filePath = fileNameFPS;
        StreamWriter outStream = System.IO.File.CreateText(filePath);
        outStream.WriteLine(sb);
        outStream.Close();
    }
    // Start is called before the first frame update
    void Start()
    {

    }
    // Update is called once per frame
    void Update()
    {


        deltatime += (Time.unscaledDeltaTime - deltatime) * 0.1f;
        if (frame < MaxFrame)
        {
            //c = new Controller();
            //pair = c.collidablePairIndex;

            float fps = 1.0f / deltatime;
            avgFPS += fps;
            arrFps.Add(fps);
        }
        else if (frame == MaxFrame)
        {
            writeCSVfps();
            print("avg. FPS :" + (avgFPS / (float)MaxFrame));
            UnityEditor.EditorApplication.isPlaying = false;
        }
        frame++;
    }
}
