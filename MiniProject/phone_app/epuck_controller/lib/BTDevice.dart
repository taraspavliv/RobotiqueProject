import 'package:flutter/material.dart';
import 'dart:convert';
import 'dart:typed_data';
import 'package:epuck_controller/controller.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';

enum Status {
  none,
  paired,
  connected,
  controlled
}

class BTDevice{
  //String deviceName;
  Status deviceStatus;
  BluetoothDevice bluedevice;

  BluetoothConnection connection;
  bool isConnecting = true;
  bool get isConnected => connection != null && connection.isConnected;
  bool isDisconnecting = false;

  BTDevice(BluetoothDevice bluedevice){
    this.deviceStatus = Status.paired;
    this.bluedevice = bluedevice;
    //this.deviceName = deviceName;
  }

  void dispose(){
    if(isConnecting){
      isDisconnecting = true;
      connection.dispose();
      connection = null;
    }
    connection.dispose();
    connection = null;
  }

  _getBTConnection(){
    BluetoothConnection.toAddress(bluedevice.address).then((_connection){
      connection = _connection;
      isConnecting = false;
      isDisconnecting = false;
      connection.input.listen(_onDataReceived).onDone((){
        if(isDisconnecting){
          print('Disconnecting locally');
        }else{
          print('Disconnecting remotely');
        }

      });
    }).catchError((error) {
      print('Cannot connect, exception occured');
      print(error);
    });
  }

  void _onDataReceived(Uint8List data){
    int backspacesCounter = 0;
    data.forEach((byte) {
      if (byte == 8 || byte == 127) {
        backspacesCounter++;
      }
    });
    Uint8List buffer = Uint8List(data.length - backspacesCounter);
    int bufferIndex = buffer.length;

    backspacesCounter = 0;
    for (int i = data.length - 1; i >= 0; i--) {
      if (data[i] == 8 || data[i] == 127) {
        backspacesCounter++;
      } else {
        if (backspacesCounter > 0) {
          backspacesCounter--;
        } else {
          buffer[--bufferIndex] = data[i];
        }
      }
    }
    var decoded = utf8.decode(buffer);
    print(decoded);
  }

}



class DeviceCard extends StatefulWidget {

  BTDevice device;
  DeviceCard(this.device);

  @override
  _DeviceCardState createState() => _DeviceCardState();
}

class _DeviceCardState extends State<DeviceCard> {

  String buttonText = 'Connect';

  void _sendMessage(String text) async {
    text = text.trim();
    if(text.length > 0){
      try {
        widget.device.connection.output.add(utf8.encode(text));
        await widget.device.connection.output.allSent;
      } catch (e) {
        print("error");
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    if(widget.device.deviceStatus == Status.none){
      buttonText = 'Pair';
    }else if(widget.device.deviceStatus == Status.paired){
      buttonText = 'Connect';
    }else if(widget.device.deviceStatus == Status.connected){
      buttonText = 'Take Control';
    }
    return Card(
      margin: EdgeInsets.fromLTRB(10.0, 20.0, 10.0, 0.0),
      child: Container(
        padding: EdgeInsets.all(10.0),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                SizedBox(height: 5.0),
                Text(widget.device.bluedevice.name == null? 'null' : widget.device.bluedevice.name),
                SizedBox(height: 15.0),
                Text(
                  widget.device.bluedevice.address == null? 'null' : widget.device.bluedevice.address,
                  style: TextStyle(color: Colors.grey[500]),
                ),
              ],
            ),
            FlatButton(
              onPressed: () {
                setState(() {
                  if(widget.device.deviceStatus == Status.none){
                    //pair
                    widget.device.deviceStatus = Status.paired;
                  }else if(widget.device.deviceStatus == Status.paired){
                    //connect
                    widget.device._getBTConnection();
                    setState(() {});
                    widget.device.deviceStatus = Status.connected;
                  }else if(widget.device.deviceStatus == Status.connected){
                    //set as controlled
                    Navigator.push(
                        context,
                        MaterialPageRoute(
                          builder: (context) => Controller(widget.device),
                        ));
                    //widget.device.deviceStatus = Status.controlled;
                  }
                });
              },
              child: Text(
                  buttonText,
                  style: TextStyle(color:Colors.green[800])
              ),
            ),
          ],
        ),
      ),
    );
  }
}