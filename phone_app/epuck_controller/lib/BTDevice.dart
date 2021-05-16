import 'package:flutter/material.dart';
import 'dart:convert';
import 'dart:typed_data';
import 'dart:async';
import 'globals.dart' as globals;
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

  int ctrlAngle = 0;
  int ctrlDist = 0;
  bool ctrlHit = false;
  bool calibrate = false;
  Timer periodicCalib;
  Timer periodicCtrl;
  int positionX = 0;
  int positionY = 0;
  int numPlayer = 0;

  BluetoothConnection connection;
  bool isConnecting = true;
  bool get isConnected => connection != null && connection.isConnected;
  bool isDisconnecting = false;

  BTDevice(BluetoothDevice bluedevice){
    this.deviceStatus = Status.paired;
    this.bluedevice = bluedevice;
    //this.deviceName = deviceName;
    periodicCtrl = Timer.periodic(Duration(milliseconds: 70), (timer) {
      if(this.deviceStatus == Status.connected || this.deviceStatus == Status.controlled){
        sendCtrlData();
      }
    });
    Timer.periodic(Duration(seconds: 120), (timer) {
      if(this.deviceStatus == Status.connected || this.deviceStatus == Status.controlled){
        this.calibrate = true;
      }
    });
  }

  void dispose(){
    this.periodicCtrl.cancel();
    if(this.isConnecting){
      this.isDisconnecting = true;
      this.connection.dispose();
      this.connection = null;
    }
    this.connection.dispose();
    this.connection = null;
    globals.connectedDevices = globals.connectedDevices - 1;
  }

  _getBTConnection(){
    BluetoothConnection.toAddress(bluedevice.address).then((_connection){
      this.connection = _connection;
      this.isConnecting = false;
      this.isDisconnecting = false;
      this.connection.input.listen(_onDataReceived).onDone((){
        if(this.isDisconnecting){
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
    //print("recieved:$decoded");

    for(int i=0; i<decoded.length; i++) {
      if((decoded[i] == 'x' || decoded[i] == 'y') && decoded[i+1] == ':'){
        for(int j=i+1; j<i+10; j++){
          if(decoded[j] == ' '){
            if(decoded[i] == 'x'){
              positionX = int.parse(decoded.substring(i+2, j));
            }else if(decoded[i] == 'y'){
              positionY = int.parse(decoded.substring(i+2, j));
            }
            break;
          }
        }
      }
    }
    print("x:$positionX");
    print("y:$positionY");

  }

  void _sendMessage(String text) async {
    text = text.trim();
    if(text.length > 0){
      try {
        this.connection.output.add(utf8.encode(text));
        await this.connection.output.allSent;
      } catch (e) {
        print("error");
      }
    }
  }

  void sendCtrlData(){
    String ctrlSendString = " ";
    if(this.deviceStatus == Status.controlled){
      ctrlSendString = ctrlSendString + ' a:${this.ctrlAngle} d:${this.ctrlDist}';
      //_sendMessage(' a:${this.ctrlAngle} d:${this.ctrlDist} ');
      //print(' a:${this.ctrlAngle} d:${this.ctrlDist} ');
      if(this.ctrlHit == true){
        this.ctrlHit = false;
        ctrlSendString = ctrlSendString + ' h';
      }
    }
    if(this.calibrate == true){
      this.calibrate = false;
      ctrlSendString = ctrlSendString + ' c';
      print('calibrate');
    }
    if(this.numPlayer == 0 && globals.connectedDevices == 2){
      ctrlSendString = ctrlSendString + ' u:${globals.player1.positionX} v:${globals.player1.positionY}';
    }else if(this.numPlayer == 1){
      ctrlSendString = ctrlSendString + ' u:${globals.player0.positionX} v:${globals.player0.positionY}';
    }
    ctrlSendString =  ctrlSendString + ' -';
    _sendMessage(ctrlSendString);
    //numPlayer == 0? print("player0"):print("player1");
    //print(ctrlSendString);
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
                    if(globals.connectedDevices < 2){
                      if(globals.connectedDevices == 0){
                        globals.player0 = widget.device;
                        widget.device.numPlayer = 0;
                      }else{
                        globals.player1 = widget.device;
                        widget.device.numPlayer = 1;
                      }
                      widget.device._getBTConnection();
                      globals.connectedDevices = globals.connectedDevices + 1;
                      //print("${globals.connectedDevices}");
                      setState(() {});
                      widget.device.deviceStatus = Status.connected;
                    }
                  }else if(widget.device.deviceStatus == Status.connected){
                    //set as controlled
                    Navigator.push(
                        context,
                        MaterialPageRoute(
                          builder: (context) => Controller(widget.device.numPlayer),
                        ));
                    widget.device.deviceStatus = Status.controlled;
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