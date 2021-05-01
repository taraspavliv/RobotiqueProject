import 'package:flutter/material.dart';
import 'package:epuck_controller/home.dart';
import 'package:epuck_controller/controller.dart';
import 'package:epuck_controller/BTDevice.dart';

void main() => runApp(MaterialApp(
  debugShowCheckedModeBanner: false,
  initialRoute: '/',
  routes: {
    '/': (context) => Home(),
    '/controller': (context) => Controller(BTDevice(null)),
  }
));
