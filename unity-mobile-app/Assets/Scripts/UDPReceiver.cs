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
    public PointCloudVisualizer visualizer;
    private int m_counter = 0;

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
                    int numInts = data.Length / sizeof(int);
                    
                    m_counter += numInts;

                    int numPoints = numInts / 3;
                    
                    Vector3Int[] pointData = new Vector3Int[numPoints];

                    for (int i = 0; i < numPoints; i++)
                    {
                        int baseIndex = i * 3 * sizeof(int);

                        int x = BitConverter.ToInt32(data, baseIndex);
                        int y = BitConverter.ToInt32(data, baseIndex + sizeof(int));
                        int z = BitConverter.ToInt32(data, baseIndex + 2 * sizeof(int));

                        pointData[i] = new Vector3Int(x, y, z);
                    }
                    // visualizing is the bottleneck rn
                    
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"UDPReceiver error: {e}");
        }
    }

    void OnApplicationQuit()
    {
        receiveThread?.Abort();
        udpClient?.Close();
    }
}
