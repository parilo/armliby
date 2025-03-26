# armliby
robotic arm library, now only contains VR teleop example with virtual robot and Open3D visualization

# install
```
git clone git@github.com:parilo/armliby.git
pip install -e armliby 
```

# generate SSL certs
```
cd armliby/examples
openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -days 365 -nodes
```

# run VR teleop example
```
python examples/try_vr_teleop.py
```

In terminal you should have:
```
WebSocket server process started with PID 24332
Flask app started on https://your-ip:5000 with PID 24333
WebSocket server started on wss://your-ip:8765
 * Serving Flask app 'armliby.vrteleop.vr_teleop_server'
 * Debug mode: off
WARNING: This is a development server. Do not use it in a production deployment. Use a production WSGI server instead.
 * Running on https://your-ip:5000
Press CTRL+C to quit
```

1 Navigate to https://your-ip:5000 in VR headset (tested on Oculus Quest 2)
2 Enter VR mode
3 While you hold B on right controller robot in VR follows your movement
4 So does the robot in Open3d window

For test purposes SO-ARM100 is used
https://github.com/TheRobotStudio/SO-ARM100
