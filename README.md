# pyarmlib
robotic arm library

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

# run
```
python examples/try_vr_teleop.py
```
