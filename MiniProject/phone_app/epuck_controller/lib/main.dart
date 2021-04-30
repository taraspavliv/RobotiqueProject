import 'package:flutter/material.dart';
import 'package:epuck_controller/home.dart';
import 'package:epuck_controller/controller.dart';

void main() => runApp(MaterialApp(
  initialRoute: '/',
  routes: {
    '/': (context) => Home(),
    '/controller': (context) => Controller(),
  }
));
