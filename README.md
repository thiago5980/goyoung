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
    ```
2. setup peak-can
    ```
    $ sudo nano /etc/rc.local
    $ /sbin/modprobe peak_usb
    $ /sbin/ip link set can0 up type can bitrate 1000000
    $ /sbin/ifconfig can0 up
    $ sudo chmod +x /etc/rc.local
    $ sudo systemctl daemon-reload
    $ sudo systemctl start rc-local
    $ sudo systemctl enable rc-local
    ```

## 로봇 실행
1. 디버그 모드 해제 방법 (로봇 전원 인가 시 로봇 구동 X)
    ```
    $ gedit ~/ros2_ws/src/Goyoung/goyoung_main/params/setting.yaml
    $ 코드 안에 있는 debug를 전부 True로 변경
    $ gedit ~/.bashrc
    $ ros2 launch goyoung_main main.launch 부분 주석 처리
    ```
2. 코드 수정 후 실행
    ```
    $ colcon build --symlink-install
    $ ros2 launch goyoung_main main.launch
    $ source ~/ros2_ws/install/setup.bash
    ```
3. 모터 연결에 이상이 있을 경우
    ```
    sudo modprobe peak_usb
    sudo ip link set can0 up type can bitrate 1000000
    sudo ifconfig can0 up
    ```