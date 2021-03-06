import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'dart:convert';
import 'dart:typed_data';
import 'globals.dart' as globals;
import 'package:control_pad/control_pad.dart';
import 'package:epuck_controller/BTDevice.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';

class Controller extends StatefulWidget {

  int numPlayerControlled = 0;
  Controller(this.numPlayerControlled);
  /*BTDevice device;
  Controller(this.device);*/

  @override
  _ControllerState createState() => _ControllerState();
}

class _ControllerState extends State<Controller> {

  @override
  dispose(){
    SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
    //widget.device.deviceStatus = Status.connected;
    widget.numPlayerControlled == 0 ? globals.player0.deviceStatus = Status.connected : globals.player1.deviceStatus = Status.connected;
    super.dispose();
  }
  Widget build(BuildContext context) {
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);

    return Scaffold(
      appBar: AppBar(
        backgroundColor: Colors.lightGreen[600],
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
                        //widget.device.ctrlHit = true;
                        widget.numPlayerControlled == 0 ? globals.player0.ctrlHit = true : globals.player1.ctrlHit = true;
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
                  /*widget.device.ctrlAngle = int.parse(degrees.toStringAsFixed(0));
                  widget.device.ctrlDist = int.parse((distance*100).toStringAsFixed(0));*/
                  widget.numPlayerControlled == 0 ? globals.player0.ctrlAngle = int.parse(degrees.toStringAsFixed(0)) : globals.player1.ctrlAngle = int.parse(degrees.toStringAsFixed(0));
                  widget.numPlayerControlled == 0 ? globals.player0.ctrlDist = int.parse((distance*100).toStringAsFixed(0)) : globals.player1.ctrlDist = int.parse((distance*100).toStringAsFixed(0));
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
