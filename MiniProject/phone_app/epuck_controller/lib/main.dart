//import 'package:control_pad/control_pad.dart'
import 'package:control_pad/control_pad.dart';
import 'package:flutter/material.dart';

void main() => runApp(MaterialApp(
  home: Scaffold(
    appBar: AppBar(
      title: Text('Epuck Football Tool'),
      centerTitle: true,
      backgroundColor: Colors.amber,
    ),
    body: Center(
      //child: Text('Scanning for epucks....'),
      child: JoystickView(
        showArrows: false,
        innerCircleColor: Colors.grey[500],
        backgroundColor: Colors.grey[300],
        size: 250,
        opacity: 0.5,
      ),
    ),
    floatingActionButton: FloatingActionButton(
      child: Icon(
        Icons.settings_input_antenna_sharp,
        color: Colors.black,
      ),
      backgroundColor: Colors.amber,
    ),
  ),
));

