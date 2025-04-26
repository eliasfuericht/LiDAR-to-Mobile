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

    public void DisplayPoints(Vector3[] pData)
    {
        foreach (Vector3 point in pData)
        {
            Instantiate(pointPrefab, point, Quaternion.identity);
        }
    }
}
