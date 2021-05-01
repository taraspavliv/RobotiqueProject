import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
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
                        //TODO: send command
                        print("${widget.device.bluedevice.name} - shoot!");
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
                  //TODO: send commands
                  print('$degrees $distance');
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
