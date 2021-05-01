import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter/painting.dart';
import 'package:flutter/services.dart';
import 'package:epuck_controller/BTDevice.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';

enum _DeviceAvailability {
  no,
  maybe,
  yes,
}

class _DeviceWithAvailability extends BluetoothDevice {
  BluetoothDevice device;
  _DeviceAvailability availability;
  int rssi;

  _DeviceWithAvailability(this.device, this.availability, [this.rssi]);
}

class Home extends StatefulWidget {

  final bool checkAvailability;

  const Home({this.checkAvailability = true});

  @override
  _HomeState createState() => _HomeState();
}

class _HomeState extends State<Home> with WidgetsBindingObserver{

  List<BTDevice> BTdevices = [];
  BluetoothState _bluetoothState = BluetoothState.UNKNOWN;
  List<BluetoothDevice> devices = List<BluetoothDevice>();


  //-discovered
  List<_DeviceWithAvailability> AVdevices = List<_DeviceWithAvailability>();

  // Availability
  StreamSubscription<BluetoothDiscoveryResult> _discoveryStreamSubscription;
  bool _isDiscovering;

  @override
  void initState(){
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    //_getBTState();
    _stateChangeListener();
    //_listBondedDevices();

    _isDiscovering = widget.checkAvailability;

    if (_isDiscovering) {
      _startDiscovery();
    }
  }

  void _restartDiscovery(){
    setState(() {
      _isDiscovering = true;
    });

    _startDiscovery();
  }

  void _startDiscovery(){
    // Setup a list of the bonded devices
    FlutterBluetoothSerial.instance
        .getBondedDevices()
        .then((List<BluetoothDevice> bondedDevices) {
      setState(() {
        AVdevices = bondedDevices
            .map(
              (device) => _DeviceWithAvailability(
            device,
            widget.checkAvailability
                ? _DeviceAvailability.maybe
                : _DeviceAvailability.yes,
          ),
        ).toList();
      });
    });

     _discoveryStreamSubscription =
        FlutterBluetoothSerial.instance.startDiscovery().listen((r) {
          setState(() {
            Iterator i = AVdevices.iterator;
            while (i.moveNext()) {
              var _device = i.current;
              if (_device.device == r.device) {
                _device.availability = _DeviceAvailability.yes;
                _device.rssi = r.rssi;
                BTdevices.add(BTDevice(r.device));
              }
            }
          });
          setState(() {});
        });

    _discoveryStreamSubscription.onDone(() {
      setState(() {
        _isDiscovering = false;
      });
    });
  }

  @override
  void dispose(){
    WidgetsBinding.instance.removeObserver(this);
    _discoveryStreamSubscription?.cancel();
    super.dispose();
  }

  //@override
  /*void didChangeAppLifeCycleState(AppLifecycleState state) {
    if(state.index == 0){
      if(_bluetoothState.isEnabled){
        _listBondedDevices();
      }
    }
  }*/

  /*_getBTState(){
    FlutterBluetoothSerial.instance.state.then((state){
      _bluetoothState = state;
      if(_bluetoothState.isEnabled){
        _listBondedDevices();
      }
      setState(() {});
    });
  }*/

  _stateChangeListener() {
    FlutterBluetoothSerial.instance.onStateChanged().listen((BluetoothState state) {
      _bluetoothState = state;
      if(_bluetoothState.isEnabled == false){
        devices.clear();
      }
      print("State enabled? ${state.isEnabled}");
      setState(() {});
    });
  }

  /*_listBondedDevices(){
    FlutterBluetoothSerial.instance.getBondedDevices().then((List<BluetoothDevice> bondedDevices){
      devices = bondedDevices;
      setState(() {});

    });
  }*/

  @override
  Widget build(BuildContext context) {
    SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
    return Scaffold(
      appBar: AppBar(
        title: Text('E-puck  Football  Tool', style: TextStyle(color: Colors.black,)),
        centerTitle: true,
        backgroundColor: Colors.lightGreen,
      ),
      body: ListView(
        children: BTdevices.map((device) =>  DeviceCard(device)).toList(),
      ),
      floatingActionButton: Container(
        height: 70.0,
        width: 70.0,
        child: FloatingActionButton(
          child: Icon(
            Icons.bluetooth_searching,
            color: Colors.black,
            size: 30.0,
          ),
          backgroundColor: Colors.lightGreen,
          onPressed: (){
            BTdevices.clear();
            AVdevices.clear();
            setState(() {});
            _restartDiscovery();
            for(int i=0; i < AVdevices.length; ++i){
              AVdevices[i].availability =_DeviceAvailability.maybe;
              setState(() {});
            }

            //_listBondedDevices();
            if(_bluetoothState.isEnabled){
              print("bluetooth enabled");

              _restartDiscovery();
              print(AVdevices.length);
              for(int i=0; i < AVdevices.length; ++i){
                AVdevices[i].availability ==_DeviceAvailability.yes? print("${AVdevices[i].device.name}") : print("oh no");
              }
            }
            //create cards of devices
          },
        ),
      ),
    );
  }
}
