import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:control_pad/control_pad.dart';

class Controller extends StatefulWidget {
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
                      onPressed: () {},
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
