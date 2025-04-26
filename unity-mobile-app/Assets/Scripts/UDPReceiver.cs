using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using System.Collections.Generic;
using static UnityEngine.ParticleSystem;

public class UDPReceiver : MonoBehaviour
{
    private UdpClient udpClient;
    private Thread receiveThread;
    private const int listenPort = 8888;

    private ParticleSystem particleSystem;
    private static int maxPointCount = 19968;
    private ParticleSystem.Particle[] particles = new ParticleSystem.Particle[maxPointCount];
    private int index = 0;

    void Awake()
    {
        particleSystem = gameObject.AddComponent<ParticleSystem>();

        var main = particleSystem.main;
        main.loop = false;
        main.playOnAwake = false;
        main.startLifetime = Mathf.Infinity;
        main.startSpeed = 0f;
        main.maxParticles = maxPointCount;
        main.simulationSpace = ParticleSystemSimulationSpace.World;

        var emission = particleSystem.emission;
        emission.enabled = false;

        var shape = particleSystem.shape;
        shape.enabled = false;
    }

    void Start()
    {
        udpClient = new UdpClient(new IPEndPoint(IPAddress.Any, listenPort));
        receiveThread = new Thread(ReceiveData);
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
                    int numPoints = numInts / 3;

                    if (index > maxPointCount - numPoints)
                    {
                        index = 0;
                    }
                    
                    for (int i = 0; i < numPoints; i++)
                    {
                        int baseIndex = i * 3 * sizeof(int);

                        float x = BitConverter.ToInt32(data, baseIndex) / 100.0f;
                        float y = BitConverter.ToInt32(data, baseIndex + sizeof(int)) / 100.0f;
                        float z = BitConverter.ToInt32(data, baseIndex + 2 * sizeof(int)) / 100.0f;

                        particles[index].position = new Vector3(x, y, z);
                        particles[index].startSize = 0.1f;
                        particles[index].startColor = Color.white;
                        particles[index].remainingLifetime = Mathf.Infinity;
                        index++;
                    }

                    particleSystem.SetParticles(particles, maxPointCount);
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
