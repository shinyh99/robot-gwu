# Python in ROS

## Python을 이용하 ROS 어떻게 쓰는가
### ROS Package 구조
```
ROS
 ├── ROS_Workspace
        ├── build
            ( 컴파일된 실행가능 파일이 담겨있는 폴더 )
        ├── devel
            ( Workspace의 패키지 경로 환경변수가 담겨있는 폴더 )
        ├── src
            ( 패키지가 담겨있는 폴더_source)
            ├── tutorial
```

### Python으로 코딩하는 경우
1. package 만들기
```
$ cd catkin_ws/src
$ catkin_create_pkg "패키지이름" rospy
```
2. 소스폴더 만들기
```
$ cd "패키지이름"
$ mkdir scripts
Python은 Scripts 폴더에 소스코드를 저장합니다.
```

3. .py 작성하기
```
#! /usr/bin/env python or /usr/bin/env python3
꼭 최상단에 위와 같은 코드를 삽입하여 주세요.

그리고 한글을 넣고싶으시면
# -*- coding: utf-8 -*-
이 주석을 위 코드 다음에 넣어주세요.
```

4. 실행하기
```
$ sudo chmod 666 "실행하고자하는 Python 파일"
$ rosrun 패키지이름 파일이름
```
extra. Python3를 사용하고 싶다면,
```
$ sudo apt-get install python3-catkin-pkg-modules 

$ sudo apt-get install python3-rospkg-modules 
```

## VS Code 깔기
1. 설치 스크립트 실행
```
$ sudo snap install --classic code
```
2. 실행 끝
```
$ code
```

## demo file list
- make_node.py
- make_publisher.py
- make_subscriber.py
- rospy_and_rate.py
- rospy_pub.py
- rospy_sub.py
- tuto_laserscan.py

### 튜토리얼 데모 소개
#### make_node.py
- node 생성 코드
#### make_publisher.py
- publisher 생성 코드
#### make_subscriber.py
- subscriber 생성 코드
#### rospy_and_rate.py
- spin과 rate 생성 코드
#### rospy_pub.py, rospy_sub.py
- pub, sub 실습 예제 코드
#### tuto_laserscan.py
- sensor_msgs.msg LaserScan 데이터 가공 예제

## Package Tree
```
├──RPLidar ROS
├── Tutorial
        ├── Scripts
                ├── make_node.py
                ├── rospy_and_rate.py
                (----설명용 예제----)
                ├── make_publisher.py
                ├── make_subscriber.py
                ├── rospy_pub.py
                ├── rospy_sub.py
                ├── tuto_laserscan.py
                (---실행 가능 예제---)
```