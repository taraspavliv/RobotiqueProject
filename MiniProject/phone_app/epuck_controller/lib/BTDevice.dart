import 'package:flutter/material.dart';
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

  BTDevice(BluetoothDevice bluedevice){
    this.deviceStatus = Status.paired;
    this.bluedevice = bluedevice;
    //this.deviceName = deviceName;
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