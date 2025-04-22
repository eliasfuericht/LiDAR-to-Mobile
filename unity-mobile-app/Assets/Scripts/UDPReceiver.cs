using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class UDPReceiver : MonoBehaviour
{
    private UdpClient udpClient;
    private Thread receiveThread;
    private const int listenPort = 8888;

    PointCloudVisualizer pointCloudVisualizer = new PointCloudVisualizer();
    void Start()
    {
        udpClient = new UdpClient(new IPEndPoint(IPAddress.Any, listenPort));
        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();
        Debug.Log($"UDPReceiver started on port {listenPort}");
    }

    private void ReceiveData()
    {
        IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, listenPort);

        try
        {
            while (true)
            {
                byte[] data = udpClient.Receive(ref remoteEndPoint);
                if (data.Length != 0)
                {
                    int numDoubles = data.Length / sizeof(double);
                    int numPoints = numDoubles / 3;
                    Debug.Log($"{numPoints} points received");
                    Vector3[] pointData = new Vector3[numPoints];

                    for (int i = 0; i < numPoints; i++)
                    {
                        pointData[i].x = (float)BitConverter.ToDouble(data, i * 3 * sizeof(double));
                        pointData[i].y = (float)BitConverter.ToDouble(data, i * 3 + 1 * sizeof(double));
                        pointData[i].z = (float)BitConverter.ToDouble(data, i * 3 + 2 * sizeof(double));
                    }

                    pointCloudVisualizer.DisplayPoints(pointData);
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"UDPReceiver error: {e}");
        }
    }

    void Update()
    {
        
    }

    void OnApplicationQuit()
    {
        receiveThread?.Abort();
        udpClient?.Close();
    }
}
