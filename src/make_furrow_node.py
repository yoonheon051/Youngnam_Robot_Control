import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import GoalResponse
from rclpy.executors import MultiThreadedExecutor
from cobot1_interfaces.action import TaskSequence

import time
import DR_init
import json
import os

# ============================
# 1. 전역 좌표 및 포즈 변수 선언
# ============================
# 각 하드웨어 구동 메서드간에 위치 데이터를 공유하기 위한 전역(Global) 변수
global tpos         # 삽(Tool)이 보관되어 있는 기본 포즈 (좌표 배열)
global tpos_above   # 삽 보관 위치 직전/직후의 충돌 방지용 상단 포즈
global gpos         # 도랑 파기 작업을 시작할 지면 포즈 (Start Go Position)
global gpos_above   # 도랑 파기 시작 위치 직전의 상단 포즈
global hpos         # 로봇의 홈(Home) 포즈


# ==============================
# 2. 로봇 식별자 상수 및 경로 정의
# ==============================
BASE_DIR = os.path.dirname(os.path.abspath(__file__)) # 현재 스크립트 파일의 절대 경로 추출


def find_pose_file():
    """포즈 데이터(data.json) 위치: 환경변수 POSE_FILE → 스크립트와 같은 폴더 → 저장소의 config/ 폴더 순서로 찾는다.
    (ros2 run 으로 설치본을 실행하면 스크립트 폴더가 install/ 아래라서, 경로를 직접 줄 수 있게 함)"""
    candidates = [os.environ.get("POSE_FILE"),
                  os.path.join(BASE_DIR, "data.json"),
                  os.path.join(os.path.dirname(BASE_DIR), "config", "data.json")]
    for path in candidates:
        if path and os.path.isfile(path):
            return path
    raise FileNotFoundError("data.json 을 찾지 못했습니다. 찾아본 위치: " + ", ".join(p for p in candidates if p)
                            + " (환경변수 POSE_FILE 로 경로 지정 가능)")

# 두산로보틱스 협동로봇 드라이버 연동용 하드웨어 상수 설정
ROBOT_ID = "dsr01"            # ROS 2 네임스페이스 및 로봇 식별자 ID
ROBOT_MODEL = "m0609"         # 로봇 모델명
ROBOT_TOOL = "Tool Weight"    # 엔드이펙터(End-Effecter) 툴 무게 프로파일
ROBOT_TCP = "GripperDA_v1_1"  # 공구 중심점(TCP, Tool Center Point) 정의 명칭

# 로봇 엔드이펙터의 동작 선속도 및 가속도 제한 상수 (안전을 위해 최대치의 퍼센트 또는 mm/s 단위 적용)
VELOCITY = 60
ACC = 60

# 두산로보틱스 시스템 패키지(DR_init)의 내부 변수에 상수를 직접 바인딩하여 드라이버 초기화 준비
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇 하드웨어의 엔드이펙터 툴 및 공구 중심점(TCP) 프로파일 설정을 화면에 출력하는 초기화 메서드

    주의: 출력만 하고 set_tool()/set_tcp()는 호출하지 않는다. 툴 무게·TCP는 티치펜던트에 설정된 값을 그대로 쓴다.
    make_furrow 의 힘 제어는 툴 무게 설정에 영향을 받으므로, 실행 전 티치펜던트에서 ROBOT_TOOL/ROBOT_TCP 가 선택돼 있는지 확인할 것.
    """

    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)
    

def loadpos():
    """JSON 파일로부터 티칭(Teaching)된 위치 좌표 데이터를 읽어와 전역 변수에 파싱 및 가공하는 메서드"""
    global tpos
    global tpos_above
    global gpos
    global gpos_above
    global hpos

    # 데이터의 안전한 Read를 위해 Python의 컨텍스트 매니저(with) 구문 활용
    pose_file = find_pose_file()
    print(f"Pose data: {pose_file}")
    with open(pose_file, "r", encoding="utf-8") as f:
        posdata = json.load(f)

    # JSON 키값에 매핑된 좌표(6자유도 공간 좌표 [X, Y, Z, Rx, Ry, Rz] 또는 Joint 값) 추출
    hpos = posdata["home"]
    tpos = posdata["tool"]
    gpos = posdata["startgo"]

    # --- 충돌 방지용 접근(Approach) 좌표 수학적 오프셋 연산 ---
    tpos_above = tpos.copy()
    tpos_above[0] += 25  # X축 방향으로 25mm 오프셋 이동
    tpos_above[2] = 650  # Z축(높이) 좌표를 안전 고도인 650mm로 고정 고정

    gpos_above = gpos.copy()
    gpos_above[2] = 650  # Z축(높이) 좌표를 안전 고도인 650mm로 고정


# ==========================================
# 3. 로봇 하드웨어 및 단일 동작 제어 API 함수군
# ==========================================

def grab():
    """로봇 공압/디지털 그리퍼 동작: DO(Digital Output) 신호를 제어하여 그리퍼를 닫는(Pneumatic Close) 함수"""
    from DSR_ROBOT2 import set_digital_output, wait, ON, OFF
    set_digital_output(1, ON)   # 1번 디지털 출력 포트 활성화 (그리퍼 파지 압력 인가)
    set_digital_output(2, OFF)  # 2번 디지털 출력 포트 비활성화
    wait(1)                     # 하드웨어가 완전히 닫히고 압력이 형성될 때까지 1초간 블로킹 대기


def release():
    """로봇 공압/디지털 그리퍼 동작: DO 신호를 제어하여 그리퍼를 여는(Pneumatic Open) 함수"""
    from DSR_ROBOT2 import set_digital_output, wait, OFF
    set_digital_output(1, OFF)  # 파지 압력 해제
    set_digital_output(2, OFF)  
    wait(0.25)                  # 실린더 동작 시간인 0.25초간 대기


def basic_pose():
    """로봇 축 제어(Joint Space Move): 관절 각도를 기준으로 로봇을 기본 대기 포즈로 이동시키는 함수"""
    from DSR_ROBOT2 import movej, mwait
    bpose = [0, 0, 45, 0, 135, -90] # 1번축부터 6번축까지의 목표 관절 각도(Degree) 배열
    
    release() # 동작 전 엔드이펙터 초기화
    # movej: 관절 공간 이동 (특이점 피하기에 유리함, 목표 도달 시간 1.5초 제한 지정)
    movej(bpose, time=1.5, vel=VELOCITY, acc=ACC)
    mwait(0.25) # 로봇의 기계적 진동이 감쇄될 때까지 모션 완료 동기식 대기(Motion Wait)


def get_shovel():
    """삽(Tool) 보관소로 이동하여 그리퍼로 잡고 상단 안전 고도로 복귀하는 일련의 모션 시퀀스"""
    mforward_Tpos = [70, 0, 20, 0, 0, 0] # 상대 좌표 이동을 위한 오프셋 값 [dX, dY, dZ, dRx, dRy, dRz]
    
    # 순차적으로 이동할 타겟 포즈들의 시퀀스 배열화
    poslist = [tpos_above, tpos, mforward_Tpos, tpos_above]
    from DSR_ROBOT2 import movel, wait, posx, DR_MV_MOD_REL

    i = 0
    while(i < len(poslist)):
        if(i == 2):
            # movel: 데카르트 직교 공간 내 직선 이동 (Linear Move)
            # mod=DR_MV_MOD_REL : 현재 위치를 원점으로 삼는 상대 좌표(Relative) 이동 방식으로 삽을 밀어 넣음
            movel(posx(poslist[i]), time=1.5, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
            wait(0.25)
        else:
            # 절대 좌표 기준 직선 이동
            movel(posx(poslist[i]), time=1.5, vel=VELOCITY, acc=ACC)
            wait(0.25)
            
            # 특정 인덱스(위치) 도달 시점에 하드웨어 그리퍼 연동
            if(i == 0):
                release()  # 진입 전 그리퍼 개방
            elif(i == 1):
                grab()     # 삽 위치 도달 시 파지 수행
        i += 1


def make_furrow():
    """힘 제어(Force Control/Compliance Control) 메커니즘을 적용하여 지면에 삽을 대고 도랑을 파는 함수"""
    from DSR_ROBOT2 import movel, wait, posx, DR_FC_MOD_REL, DR_BASE, move_periodic, task_compliance_ctrl, release_force, set_desired_force, release_compliance_ctrl

    # 1. 도랑 파기 지점 상공으로 직선 이동 후 지면 목표 좌표(gpos)로 하강
    movel(posx(gpos_above), time=1, vel=VELOCITY, acc=ACC)
    movel(posx(gpos), time=1, vel=VELOCITY, acc=ACC)
    
    # 2. 컴플라이언스(유연성) 제어 매개변수 정의
    # stx: 6축 방향별 강성(Stiffness) 설정 [X, Y, Z, Rx, Ry, Rz] -> 값이 낮을수록 외부 힘에 유연하게 밀림
    setcomp = [40, 3000, 3000, 200, 200, 200] 
    # setfd: 목표 힘/토크 크기 배열 (X축 방향으로 130N의 힘을 지속적으로 인가)
    setfd = [130, 0, 0, 0, 0, 0]
    # setfdir: 힘 제어를 적용할 좌표축 방향 지시 포인터 (1: 활성, 2: 좌표계 기준 비례 적용 등)
    setfdir = [2, 0, 0, 0, 0, 0]

    # 로봇의 작업 공간 컴플라이언스 제어 활성화 (지면의 높낮이 저항에 로봇이 망가지지 않도록 유연성 인가)
    task_compliance_ctrl(stx=setcomp)
    wait(0.25)
    
    # 로봇 토크 센서를 기반으로 목표 힘(Desired Force) 인가 시작
    set_desired_force(fd=setfd, dir=setfdir, mod=DR_FC_MOD_REL)

    # 3. 주기적 모션(Periodic Move)을 통한 흙 파기 수행
    # amp: 각 축별 왕복 운동 진폭 설정 [x, y, z, rx, ry, rz] → 4번째 값이므로 Rx축(롤) 방향으로 15도씩 흔들며 도랑을 파헤침
    amp = [0, 0, 0, 15, 0, 0]
    # 로봇 베이스 좌표계(DR_BASE)를 기준으로 1초 주기로 5회 왕복 진동 모션 수행
    move_periodic(amp, period=1, repeat=5, ref=DR_BASE)
    
    # 4. 힘 제어 프로세스 종료 및 안전 고도 복귀
    release_force()            # 인가하던 힘 제어 종료
    wait(0.25)
    release_compliance_ctrl()  # 컴플라이언스 모드 해제 (일반 고강성 포지션 제어로 복귀)
    
    # 작업이 완료된 지점에서 다시 gpos 좌표(안전선)로 직선 이동하여 복귀
    movel(posx(gpos), time=1.5, vel=VELOCITY, acc=ACC, ref=DR_BASE)


def getback_shovel():
    """사용했던 삽(Tool)을 다시 보관소 위치에 안전하게 릴리즈하고 복귀하는 모션 시퀀스"""
    from DSR_ROBOT2 import movel, mwait, posx, DR_BASE

    # 역순 이동 경로 시퀀스 구성
    poslist = [gpos_above, tpos_above, tpos, tpos_above]
    i = 0
    while(i < len(poslist)):
        movel(posx(poslist[i]), time=1.5, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        mwait(0.25)
        if(i == 2):
            release()  # 삽 보관함 내부에 진입 완료 시 그리퍼 개방(삽 반납)
        i += 1
    basic_pose()       # 최종 대기 포즈로 전환
    

# ===========================================
# 4. ROS 2 액션 서버(Action Server) 클래스 정의
# ===========================================
class makefurrow(Node):
    """
    상위 클라이언트 노드(TaskSequenceClient)로부터 구체적인 단일 동작 신호(signal)를 받아서
    실제 로봇 제어 함수들을 바인딩하여 실행하는 비동기 액션 서버 노드
    """
    def __init__(self):
        # rclpy.node.Node 생성자를 호출하여 상위 네트워크에 'make_furrow_action' 노드 등록
        super().__init__('make_furrow_action')
        
        # 액션 서버 객체 인스턴스화
        # 연결 노드, 인터페이스 타입(TaskSequence), 토픽명('Task_Sequence_mf') 지정
        # goal_callback: 클라이언트가 요청을 보냈을 때 수락 여부를 결정하는 함수
        # execute_callback: 수락된 요청의 실제 하드웨어 로직을 비동기로 처리하는 함수
        self._server = ActionServer(
            self, TaskSequence, 'Task_Sequence_mf',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
        )
        self.get_logger().info('Action Server ready: /Task_Sequence_mf')

    def goal_callback(self, goal_request: TaskSequence.Goal):
        """클라이언트가 송신한 작업 목표(Goal) 문자열을 검증하여 수락(Accept) 또는 거절(Reject)을 판단하는 콜백"""
        self.get_logger().info(f'Goal received: signal="{goal_request.signal}"')
        
        # 자신이 제어할 수 있는 하드웨어 기능 풀(Pool)에 속하는 문자열인지 검증
        if goal_request.signal in ["basic_pose", "get_shovel", "make_furrow", "getback_shovel"]:
            return GoalResponse.ACCEPT # 처리 가능하므로 통신 승인 응답반환
        else:
            return GoalResponse.REJECT # 처리 불가능한 명령이므로 거절


    def execute_callback(self, goal_handle):
        """수락 완료된 하드웨어 동작 명령을 실질적으로 실행하고 피드백 및 최종 결과를 반환하는 실행 엔진 콜백"""
        signal = goal_handle.request.signal
        feedback = TaskSequence.Feedback() # 실시간 트래킹용 피드백 메시지 객체 생성
                  
        # -------------------------------------------------------------------------
        # 클라이언트가 요청한 signal 문자열 조건 분기를 통해 타겟 하드웨어 제어 함수 호출
        # -------------------------------------------------------------------------
        if signal == "basic_pose":
            self.get_logger().info(f'Executing: "{signal}"')
            basic_pose()

        elif signal == "get_shovel":
            self.get_logger().info(f'Executing: "{signal}"')
            get_shovel()
            
        elif signal == "make_furrow":
            self.get_logger().info(f'Executing: "{signal}"')
            make_furrow()

        elif signal == "getback_shovel":
            self.get_logger().info(f'Executing: "{signal}"')
            getback_shovel()
        
        # 동작 실행 중 클라이언트 노드로 현재 하드웨어 처리 상태(Status) 피드백 퍼블리시
        feedback.status = f'processing : "{signal}"...'
        goal_handle.publish_feedback(feedback)
        time.sleep(0.3) # 통신 데이터 유실 방지 및 컨텍스트 스위칭을 위한 최소한의 타임 딜레이

        # 하드웨어 물리 모션이 완전히 종료되었음을 인지하고 Goal 상태를 성공(Succeeded)으로 천이
        goal_handle.succeed()
        
        # 클라이언트에게 최종 리턴해줄 액션 결과(Result) 메시지 패키징
        result = TaskSequence.Result()
        result.accepted = True
        result.message = f'"{signal}"is End.'
        self.get_logger().info(f'Result: {result.message}')
        
        return result
        
    
def main(args=None):
    """메인 실행 프로세스: 두산로보틱스 제어 노드와 ROS 2 액션 서버 노드를 결합하여 구동하는 영역"""
    rclpy.init(args=args)
    
    # 1. 두산로보틱스 전용 API가 참조하여 하드웨어를 제어할 기본 통신 노드(nodeR) 생성
    # 네임스페이스를 'dsr01'로 지정하여 멀티 로봇 환경에서의 토픽 충돌 방지
    node = rclpy.create_node("make_furrow", namespace=ROBOT_ID)
    nodeR = node
    
    # 2. 위에서 정의한 자체 액션 서버 노드(nodeA) 인스턴스 생성
    nodeA = makefurrow()
    
    # 3. 중요: 두 개의 노드(하드웨어 통신 노드, 액션 서버 노드)의 콜백 큐를 
    # 병렬로 스핀(spin)시키기 위해 MultiThreadedExecutor 인스턴스화 및 노드 추가
    executor = MultiThreadedExecutor()
    executor.add_node(nodeR)
    executor.add_node(nodeA)

    # 두산로보틱스 코어 전역 변수 파라미터(`__dsr__node`)에 실제 ROS 2 통신 노드 바인딩 (드라이버 연동 핵심)
    DR_init.__dsr__node = nodeR

    try:
        # 시스템 기동 시 최초 1회 로봇 툴 프로파일 세팅 및 JSON 티칭 데이터 파일 로드
        initialize_robot()
        loadpos()
        
        # 멀티 스레드 실행기를 가동하여 하드웨어 이벤트 및 액션 요청에 대해 무한 대기 및 스핀 시작
        executor.spin()

    except KeyboardInterrupt:
        # Ctrl+C 인터럽트 감수 시 정상 종료 프로세스 진입
        pass
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        # 생성된 ROS 2 노드 자원을 메모리에서 해제하고 통신 레이어 shutdown 처리
        nodeR.destroy_node()
        nodeA.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
