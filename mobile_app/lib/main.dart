import 'dart:io';
import 'dart:typed_data';
import 'package:flutter/material.dart';

void main() {
  runApp(UDPListenerApp());
}

class UDPListenerApp extends StatefulWidget {
  @override
  _UDPListenerAppState createState() => _UDPListenerAppState();
}

class _UDPListenerAppState extends State<UDPListenerApp> {
  String receivedData = "Waiting for UDP data...";

  @override
  void initState() {
    super.initState();
    startListening();
  }

  void startListening() async {
    // Set up a UDP socket
    RawDatagramSocket socket = await RawDatagramSocket.bind(InternetAddress.anyIPv4, 8888);
    print("Listening for UDP packets on port 8888...");

    socket.listen((RawSocketEvent event) {
      if (event == RawSocketEvent.read) {
        Datagram? datagram = socket.receive();
        if (datagram != null) {
          String message = String.fromCharCodes(datagram.data);
          print("Received: $message");

          // Update UI with received data
          setState(() {
            receivedData = message;
          });
        }
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: Text("UDP Listener")),
        body: Center(
          child: Padding(
            padding: const EdgeInsets.all(16.0),
            child: Text(
              receivedData,
              style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
              textAlign: TextAlign.center,
            ),
          ),
        ),
      ),
    );
  }
}
