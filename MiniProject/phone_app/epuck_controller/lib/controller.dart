import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'dart:convert';
import 'dart:typed_data';
import 'package:control_pad/control_pad.dart';
import 'package:epuck_controller/BTDevice.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';

class Controller extends StatefulWidget {

  BTDevice device;
  Controller(this.device);

  @override
  _ControllerState createState() => _ControllerState();
}

class _ControllerState extends State<Controller> {

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
  dispose(){
    SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
    super.dispose();
  }
  Widget build(BuildContext context) {
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);

    return Scaffold(
      appBar: AppBar(
        backgroundColor: Colors.lightGreen,
      ),
      body: SafeArea(
        child: Row(
          children: [
            Expanded(
              child: Row(
                children: [
                  SizedBox(
                    width:70,
                  ),
                  SizedBox(
                    height: 210.0,
                    width: 210.0,
                    child: RaisedButton(
                      shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(100),
                      ),
                      onPressed: () {
                        _sendMessage('h');
                        //print("${widget.device.bluedevice.name} - shoot!");
                      },
                      child: Text("Shoot!",
                          style: TextStyle(fontSize:30.0)),
                    ),
                  ),
                  SizedBox(
                    width:10,
                  ),
                ],
              ),
            ),
            Expanded(
              child: JoystickView(
                onDirectionChanged: (double degrees, double distance) {
                  _sendMessage('a:${degrees.toStringAsFixed(0)} d:${(distance*100).toStringAsFixed(0)} ');
                  //print('$degrees $distance');
                },
                showArrows: false,
                innerCircleColor: Colors.grey[500],
                backgroundColor: Colors.grey[300],
                size: 250,
                opacity: 0.5,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
