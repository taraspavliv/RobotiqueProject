import 'package:flutter/material.dart';

enum Status {
  none,
  paired,
  connected,
  controlled
}

class BTDevice {
  String deviceName;
  Status deviceStatus;

  BTDevice(String deviceName){
    this.deviceStatus = Status.none;
    this.deviceName = deviceName;
  }
}



class DeviceCard extends StatefulWidget {

  BTDevice device;
  DeviceCard(this.device);

  @override
  _DeviceCardState createState() => _DeviceCardState();
}

class _DeviceCardState extends State<DeviceCard> {
  @override
  Widget build(BuildContext context) {
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
                Text(widget.device.deviceName),
                SizedBox(height: 15.0),
                Text(
                  '012-23123--5135--34854',
                  style: TextStyle(color: Colors.grey[500]),
                ),
              ],
            ),
            FlatButton(
              onPressed: () {
                setState(() {
                  Navigator.pushNamed(context, '/controller');
                });
              },
              child: Text(
                  'Pair',
                  style: TextStyle(color:Colors.green[800])
              ),
            ),
          ],
        ),
      ),
    );
  }
}