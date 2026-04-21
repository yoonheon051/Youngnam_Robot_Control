# 영남향우회 자동파종기기

## Doosan Robotcis Rokey Boot Camp 6기 협동1 A-2조 프로젝트 복원자료입니다.

이 프로젝트는 Firebase Realtime Database를 활용하여 웹 기반의 인터페이스로 M0609의 파종작업을 원격으로 제어하는 시스템입니다.

## 프로젝트 개요
- 기술 스택: ROS2 (Humble), Python, Firebase Admin SDK, HTML/JS
- 주요 기능: 
  1. 웹 패널을 통한 실시간 로봇 상태 모니터링 및 제어 (시작/일시정지/중지)
  2. Firebase를 매개체로 한 클라우드-로봇 통신
  3. 액션 서버(Action Server) 기반의 정밀한 시퀀스 제어

##  폴더 구조

```
Youngnam_Robot_Control/
├── .gitignore               # Firebase키, build, install 등
├── README.md                # 프로젝트 설명 및 실행 순서 기록
│
├── frontend/                # [웹 제어 패널]
│   └── index.html
│
├── src/                     # [ROS2 소스 코드]
│   ├── main_controller.py   # 전체 흐름 제어
│   └── make_furrow_node.py  # 실제 로봇 동작 수행
│
└── config/                  # [설정 파일]
    ├── data.json            # 로봇의 위치(Pose) 데이터 저장
    └── firebase_key.json    # Firebase Admin SDK 비밀키 (보안 주의)
```

## 실행 방법

### 1. 사전 준비
- ROS2가 설치된 환경에서 `cobot1_interfaces` 액션 패키지가 빌드되어 있어야 합니다.
- Firebase 콘솔에서 생성한 서비스 계정 키(.json)를 `config/` 폴더에 위치시킵니다.

### 2. 로봇 동작 서버 실행

bash
```
# Terminal 1
ros2 run <package_name> make_furrow_node
```

### 3. 메인 컨트롤러 실행(Firebase 리스너)

bash
```
# Terminal 2
python3 src/main_controller.py
```

### 4. 웹 패널 접속
- frontend/control_panel.html 파일을 브라우저로 열어 로봇을 제어합니다.
