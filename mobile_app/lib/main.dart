import 'dart:io';
import 'dart:typed_data';
import 'package:flutter/material.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      home: Scaffold(
        appBar: AppBar(title: const Text("LiDAR UDP Stream")),
        body: const LiDARReceiver(),
      ),
    );
  }
}

class LiDARReceiver extends StatefulWidget {
  const LiDARReceiver({super.key});

  @override
  _LiDARReceiverState createState() => _LiDARReceiverState();
}

class _LiDARReceiverState extends State<LiDARReceiver> {
  List<List<int>> receivedPoints = []; // Store received XYZ points

  @override
  void initState() {
    super.initState();
    startListening();
  }

  void startListening() async {
    // Bind to UDP socket on port 8888
    RawDatagramSocket socket = await RawDatagramSocket.bind(InternetAddress.anyIPv4, 8888);
    print("Listening for LiDAR data on port 8888...");

    socket.listen((RawSocketEvent event) {
      if (event == RawSocketEvent.read) {
        Datagram? dg = socket.receive();
        if (dg != null) {
          ByteData byteData = ByteData.sublistView(Uint8List.fromList(dg.data));

          List<List<int>> points = [];
          for (int i = 0; i < byteData.lengthInBytes; i += 12) {
            if (i + 12 <= byteData.lengthInBytes) {
              int x = byteData.getInt32(i, Endian.little);
              int y = byteData.getInt32(i + 4, Endian.little);
              int z = byteData.getInt32(i + 8, Endian.little);
              points.add([x, y, z]);
            }
          }

          // Update UI with new points
          setState(() {
            receivedPoints = points;
          });

          print("Received ${points.length} LiDAR points");
        }
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: ListView.builder(
        itemCount: receivedPoints.length,
        itemBuilder: (context, index) {
          return ListTile(
            title: Text(
                "Point ${index + 1}: X=${receivedPoints[index][0]}, Y=${receivedPoints[index][1]}, Z=${receivedPoints[index][2]}"
            ),
          );
        },
      ),
    );
  }
}
