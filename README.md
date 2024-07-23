# for Goyoung Robot requirements

## REQUIREMENTS
1. install this package
    (this package include serial)
    ```
    $ git clone https://github.com/RoverRobotics-forks/serial-ros2.git
    $ cd serial-ros2 && mkdir build && cd build
    $ cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
    $ make
    $ sudo make install
    '''
2. setup peak-can
    '''
    $ sudo nano /etc/rc.local
    $ /sbin/modprobe peak_usb
    $ /sbin/ip link set can0 up type can bitrate 1000000
    $ /sbin/ifconfig can0 up
    $ sudo chmod +x /etc/rc.local
    $ sudo systemctl daemon-reload
    $ sudo systemctl start rc-local
    $ sudo systemctl enable rc-local
    '''