import 'dart:async';
import 'globals.dart' as globals;
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

  //-discovered
  List<_DeviceWithAvailability> AVdevices = List<_DeviceWithAvailability>();

  // Availability
  StreamSubscription<BluetoothDiscoveryResult> _discoveryStreamSubscription;
  bool _isDiscovering;

  @override
  void initState(){ //constructor
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    _getBTState();
    _stateChangeListener();

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
                //creates cards for paired and discovered devices
                BTdevices.add(BTDevice(r.device));
              }
            }
          });
          setState(() {});
        });

    _discoveryStreamSubscription.onDone(() {
      setState(() {
        _isDiscovering = false;
        setState(() {});
      });
    });
  }

  @override
  void dispose(){ //destructor
    WidgetsBinding.instance.removeObserver(this);
    _discoveryStreamSubscription?.cancel();
    super.dispose();
  }

  _getBTState(){ //checks if bluetooth is on
    FlutterBluetoothSerial.instance.state.then((state){
      _bluetoothState = state;
      setState(() {});
    });
  }

  _stateChangeListener() { //checks if bluetooth state changes
    FlutterBluetoothSerial.instance.onStateChanged().listen((BluetoothState state) {
      _bluetoothState = state;
      setState(() {
        print("State enabled? ${state.isEnabled}");
      });
    });
  }

  @override
  Widget build(BuildContext context) { //widget for the display
    SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
    return Scaffold(
      appBar: AppBar( //the top bar
        title: Text(_isDiscovering ? 'Looking...' :'E-puck  Football  Tool', style: TextStyle(color: Colors.black,)),
        centerTitle: true,
        backgroundColor: Colors.lightGreen[600],
      ),
      body: ListView( //lists the paired and visible devices
        children: BTdevices.map((device) => DeviceCard(device)).toList(),
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
          backgroundColor: Colors.lightGreen[600],
          onPressed: (){
            BTdevices.clear();
            for(int i=0; i < AVdevices.length; ++i){
              //prints all paired devices
              AVdevices[i].availability =_DeviceAvailability.maybe;
              BTdevices.add(BTDevice(AVdevices[i].device));
            }
            setState(() {});
            //_restartDiscovery();
              //_listBondedDevices();
            if(_bluetoothState.isEnabled){
              print("bluetooth enabled");
              //print(AVdevices.length);
              for(int i=0; i < AVdevices.length; ++i){
                AVdevices[i].availability ==_DeviceAvailability.yes? print("${AVdevices[i].device.name}") : print("oh no");
              }
            }

          },
        ),
      ),
    );
  }
}
