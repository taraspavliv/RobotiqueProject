import 'package:flutter/material.dart';
import 'package:flutter/painting.dart';
import 'package:flutter/services.dart';
import 'BTDevice.dart';

class Home extends StatefulWidget {
  @override
  _HomeState createState() => _HomeState();
}

class _HomeState extends State<Home> {

  List<BTDevice> devices = [BTDevice('Device1'), BTDevice('Device2'),];

  Widget deviceTemplate(BTDevice device){
    return DeviceCard(device);
  }

  @override
  Widget build(BuildContext context) {
    SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
    return Scaffold(
      appBar: AppBar(
        title: Text('E-puck  Football  Tool', style: TextStyle(color: Colors.black,)),
        centerTitle: true,
        backgroundColor: Colors.lightGreen,
      ),
      body: Column(
        children: devices.map((device) =>  deviceTemplate(device)).toList(),
      ),
      floatingActionButton: FloatingActionButton(
        child: Icon(
          Icons.bluetooth_searching,
          color: Colors.black,
        ),
        backgroundColor: Colors.lightGreen[400],
        onPressed: (){
          print("yo!");
          //create cards of devices
        },
      ),
    );
  }
}