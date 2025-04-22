using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using Unity.Collections;
using UnityEngine;
using UnityEngine.InputSystem.Android;
using UnityEngine.UI;

public class PointCloudVisualizer : MonoBehaviour
{
    public GameObject pointPrefab;
    public string resourceFile;
    private int counter = 0;
    void Start()
    {
        if (false)
        {
            TextAsset textAsset = Resources.Load<TextAsset>(resourceFile);

            if (textAsset != null)
            {
                string[] lines = textAsset.text.Split('\n');
                List<Vector3> pointList = new List<Vector3>();

                foreach (string line in lines)
                {
                    if (string.IsNullOrWhiteSpace(line)) continue;

                    string[] tokens = line.Trim().Split((char[])null, System.StringSplitOptions.RemoveEmptyEntries);

                    if (tokens.Length == 3 &&
                        float.TryParse(tokens[0], NumberStyles.Float, CultureInfo.InvariantCulture, out float x) &&
                        float.TryParse(tokens[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float y) &&
                        float.TryParse(tokens[2], NumberStyles.Float, CultureInfo.InvariantCulture, out float z))
                    {
                        pointList.Add(new Vector3(x, y, z));
                    }
                    else
                    {
                        Debug.LogWarning($"Invalid line skipped: {line}");
                    }
                }

                Vector3[] pointData = pointList.ToArray();
                Debug.Log($"Loaded {pointData.Length} points.");
                DisplayPoints(pointData);
            }
            else
            {
                Debug.LogError("File not found in Resources folder.");
            }
        }
    }

    public void DisplayPoints(Vector3[] pData)
    {
        foreach (Vector3 point in pData)
        {
            Instantiate(pointPrefab, point, Quaternion.identity);
        }
    }
}
