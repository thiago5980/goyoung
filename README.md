# for Goyoung Robot requirements

## 일산고등학교 웰컴 로봇
![robot_img](https://github.com/user-attachments/assets/95d0d2ea-9817-4d80-a973-b84d2a3a39ed)

## REQUIREMENTS
1. install this package
    (this package include audio play)
    ```
    $ sudo apt-get install libsfml-dev
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

## 로봇 소프트웨어 구조
![graph](https://github.com/user-attachments/assets/7c14f059-065e-4177-9ffd-469d97e559c2)

## 로봇 모션 저장 방법
    로봇 모션 저장을 위한 GUI를 사용하여 로봇 모션을 저장
    버튼은 초기 화면에서 버튼은 총 4가지로 구성되어 있으며 버튼은 아래 설명과 같음
    *로봇 실행 버튼의 경우 최종 로봇 표정 구동 화면으로 모션 저장 시에 필요하지 않음 (만약 로봇 표정 변경 시 다음 GUI 구성 python 파일을 수정하여 표정을 변경할 것)

1. 디버깅 모드 진입 시 최초 화면
    goyoung_main pkg의 setting.yaml파일을 수정하여 debug 모드 진입
![page1](https://github.com/user-attachments/assets/0f5b2f58-51ae-49a6-8b9c-19f87ece614f)

2. 모션 데이터 저장 화면 (최초 화면에서 모션 생성 버튼을 클릭)
![page2](https://github.com/user-attachments/assets/1dc1c485-e454-4a7c-9bdd-29339b229199)
    
    녹화 버튼 : 해당 화면으로 진입했을 경우 흰색 빈 칸에 저장하고자 하는 모션의 파일명을 설정하고 녹화 버튼 실행
    파일명 예시 : stay.yaml

    녹화 버튼 클릭 시, 로봇 모터의 torque가 해제 되면서 직접 교시가 가능하도록 로봇이 구동이 됨

    point 버튼 : 안전을 위해 로봇 대기 동작을 point 버튼을 사용하여 저장하고 원하는 동작 구간의 모터 값을 point 버튼을 눌러가며 모터 데이터를 저장, 안전을 위해 처음 동작과 끝 동작은 대기 동작으로 설정하여 저장

    이후 동작 시퀀스 완료 시, 저장 버튼을 눌러서 최초 화면으로 재진입
    
    동작 저장 완료 시, 다음 사진과 같이 파일이 저장됨을 볼 수 있음(저장 경로는 goyoung_motion pkg의 save_file 폴더에 앞서 지정한 이름으로 저장됨)

![yaml_exe](https://github.com/user-attachments/assets/6276bd27-f480-42e6-b3fb-dea2201d5fc8)

    만약 모터 저장 값(모터 9개-(right_motor 1~4, left_motor 1~4, chest))에 모터<->로봇간 연결 불안정성으로 인해 데이터가 0으로 표시 될 때, 앞선 동작 재 저장을 수행해야 함.

    mode의 경우 LED 모드 변경 용으로 수기로 yaml 파일에 입력을 해야 함. 동작 mode 숫자는 다음과 같음
    
![led](https://github.com/user-attachments/assets/3d4c347b-b9c4-465d-a459-9a3d458a8e8f)


3. 모션 구동 시간을 설정하는 화면 (최초 화면에서 모션 시간 저장 버튼 클릭)
![page4](https://github.com/user-attachments/assets/4908dded-0881-4898-82a8-9a976c6a0565)



4. 모션 테스트 작동 화면 (최초 화면에서 모션 재생 버튼 클릭)
![page3](https://github.com/user-attachments/assets/12dbe85b-f6a5-4757-8f33-e091d7bea154)


## 로봇 실행
1. 디버그 모드 해제 방법 (로봇 전원 인가 시 로봇 구동 X)
    키보드의 esc 버튼 클릭 시 전체화면 모드에서 창 모드로 변경됨 
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