using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using TMPro;

public class UDPReceiver : MonoBehaviour
{
    private UdpClient udpClient;
    private Thread receiveThread;
    private const int listenPort = 8888;

    public TextMeshProUGUI textDisplay;  // Assign this in the Unity Inspector

    private string latestMessage = "";
    private bool messageUpdated = false;

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
                string message = Encoding.UTF8.GetString(data);
                Debug.Log($"Received: {message}");

                // Store data for UI update in main thread
                latestMessage = message;
                messageUpdated = true;
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"UDPReceiver error: {e}");
        }
    }

    void Update()
    {
        if (messageUpdated)
        {
            if (textDisplay != null)
            {
                textDisplay.text = latestMessage;
            }
            messageUpdated = false;
        }
    }

    void OnApplicationQuit()
    {
        receiveThread?.Abort();
        udpClient?.Close();
    }
}
