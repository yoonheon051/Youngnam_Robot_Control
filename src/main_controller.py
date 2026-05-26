# ========================================================================
# 1. ROS 2 (Robot Operating System 2) 파이썬 클라이언트 인터페이스 라이브러리
# ========================================================================
import rclpy                                 
# rclpy: ROS 2의 파이썬 핵심 클라이언트 라이브러리(Client Library)
# 노드의 생성, 초기화, 통신 레이어 가동 및 프로세스 종료(Shutdown) 등 ROS 2 시스템의 수명 주기(Lifecycle)를 관리

from rclpy.node import Node                  
# Node: ROS 2 그래프(Graph) 네트워크에서 하나의 독립적인 연산 주체(프로세스)를 생성하는 기본 클래스
# 이 클래스를 상속받음으로써 토픽, 서비스, 액션 등의 통신 객체를 내부에 선언하고 구동

from rclpy.action import ActionClient        
# ActionClient: ROS 2의 3대 통신 방식 중 하나인 'Action' 통신에서 클라이언트 역할을 수행하는 클래스
# 목표(Goal) 송신, 피드백(Feedback) 수신, 최종 결과(Result) 확인 및 작업 취소(Cancel) 요청을 비동기식으로 제어

from rclpy.executors import MultiThreadedExecutor 
# MultiThreadedExecutor: ROS 2 노드 내부로 들어오는 다양한 콜백(Callback) 이벤트들을 스레드 풀(Thread Pool)을 통해 병렬 처리하는 실행기
# 단일 스레드 기반의 기본 실행기(SingleThreadedExecutor)와 달리, 특정 통신(예: 액션 결과 대기)이 블로킹되어도 다른 이벤트(예: 일시정지 명령 수신)를 동시 처리할 수 있음


# ================================================
# 2. 사용자 정의 ROS 2 인터페이스 (Custom Interface)
# ================================================
from cobot1_interfaces.action import TaskSequence 
# TaskSequence: 사용자가 직접 정의한 ROS 2 액션(Action) 메시지 타입 인터페이스
# 액션 통신 시 주고받을 데이터 구조(Goal: 전달할 명령 문자열, Feedback: 실시간 진행 상태, Result: 최종 완료 여부)의 규격을 정의


# ======================================================
# 3. Firebase Admin SDK (원격 데이터베이스 통신 라이브러리)
# ======================================================
import firebase_admin                        
# firebase_admin: Google Firebase 서비스에 관리자 권한으로 접근할 수 있도록 인증 및 연결을 관리하는 메인 패키지

from firebase_admin import credentials, db
# credentials: Firebase 프로젝트 인증용 서비스 계정 키(JSON 파일)를 로드하여 보안 인증 토큰을 생성하는 모듈
# db: Firebase의 NoSQL 기반 원격 저장소인 'Realtime Database'와 연결하여 데이터를 동기식/비동기식으로 읽고 쓰거나, 특정 데이터 경로의 변경 사항을 실시간 모니터링(listen)하는 핵심 모듈


# ================================================
# 4. 파이썬 내장 시스템 라이브러리 (System 내장 모듈)
# ================================================
import threading                             
# threading: 파이썬에서 멀티스레딩(Multi-threading)을 구현하기 위한 내장 모듈
# 외부 네트워크(Firebase)의 이벤트 대기 흐름이 ROS 2 시스템의 메인 실행 루프(Spin)를 멈추지 않도록, 별도의 독립된 백그라운드 작업 흐름(Daemon Thread)을 분리할 때 사용

import time                                  
# time: 시스템의 하드웨어 클록(Clock)에 접근하는 내장 모듈
# 여기서는 로봇의 상태 및 피드백 로그가 발생하는 순간의 타임스탬프(시/분/초)를 문자열 포맷으로 추출하여 데이터베이스에 기록하기 위해 사용

# ===================================================================================================================================================================
# ===================================================================================================================================================================

class TaskSequenceClient(Node):
    """Firebase 실시간 데이터베이스의 명령을 수신하여 로봇의 하드웨어 제어 액션(Action)을 순차적으로 구동하는 ROS 2 클라이언트 노드"""
    
    def __init__(self):
        """ROS 2 rclpy.node.Node 클래스의 생성자를 호출하여 'task_sequence_client' 노드 등록"""
        super().__init__('task_sequence_client')

        # =====================================================================
        # 1. 작업 시퀀스 데이터 구조화 (2중 루프 설계를 위한 리스트 및 딕셔너리 구성)
        # =====================================================================
        # 각 대분류 작업군이 가질 하위 소분류(세부 로봇 동작 명령) 문자열 리스트
        self.make_furrow_c = ['basic_pose', 'get_shovel', 'make_furrow', 'getback_shovel']
        self.digging_c = ['digging']
        self.seeding_c = ['seeding']
        self.cover_dirt_c = ['cover_dirt']
        self.tamping_c = ['stamp_dirt']

        # 대분류 식별자(Key)와 소분류 명령 리스트(Value)를 매핑한 딕셔너리
        self.commands = {
            'mf': self.make_furrow_c, # mf: Make Furrow (도랑 만들기)
            'dg': self.digging_c,     # dg: Digging (땅 파기)
            'sw': self.seeding_c,     # sw: Seeding (파종)
            'cd': self.cover_dirt_c,  # cd: Cover Dirt (흙 덮기)
            'sd': self.tamping_c,     # sd: Stamp Dirt (흙 다지기)
        }
        
        # 전체 태스크가 실행될 순서를 정의한 대분류 인덱스 리스트
        self.cmlist = ['mf', 'dg', 'sw', 'cd', 'sd']

        # ===============================================
        # 2. ROS 2 액션 클라이언트(Action Client) 동적 생성
        # ===============================================
        # 딕셔너리 컴프리헨션을 사용해 대분류별로 독립된 ROS 2 액션 클라이언트 객체를 자동 생성
        # 인터페이스 타입: TaskSequence / 액션 토픽의 이름: Task_Sequence_{key}
        self.act_clients = {
            key: ActionClient(self, TaskSequence, f'Task_Sequence_{key}')
            for key in self.cmlist
        }

        # =================================================
        # 3. 제어 플래그 및 2중 루프 인덱스(Pointer) 변수 정의
        # =================================================
        self.sequence_active = False  # 전체 작업 시퀀스가 가동 중인지 여부 (True일 때만 루프 작동)
        self.paused = False           # 일시정지 명령어 수신 여부 판별 플래그
        
        # 시퀀스 제어의 핵심이 되는 2중 인덱스 카운터
        self.command_con2 = 0         # 대분류 포인터 (self.cmlist의 인덱스)
        self.command_con1 = 0         # 소분류 포인터 (현재 대분류 딕셔너리 내부 리스트의 인덱스)
        
        # 현재 실행 중인 액션의 목표 핸들 객체 (Goal을 취소하거나 제어할 때 접근하는 인스턴스)
        self.current_goal_handle = None 

        # ======================================
        # 4. Firebase Realtime Database 연동 설정
        # ======================================
        # Firebase 애플리케이션의 중복 초기화를 방지하기 위한 예외 처리 조건문
        if not firebase_admin._apps:
            cred = credentials.Certificate("cred/path") # 계정 인증용 서비스 키가 기록된 JSON 파일의 경로
            firebase_admin.initialize_app(cred, {
                'databaseURL': 'databaseURL'            # DB의 주소(URL)
            })

        # 실시간 데이터 교류를 위한 각 노드(데이터 트리 경로)의 레퍼런스(참조) 객체 할당
        self.status_ref = db.reference('robot_status')  # 로봇의 현재 상태(텍스트) 및 진행률(%) 기록 노드
        self.logs_ref = db.reference('logs')            # 타임스탬프 기반 시스템 로그 전송 노드
        self.cmd_ref = db.reference('command')          # 웹/앱 인터페이스로부터 제어 명령을 받는 노드

        # =============================================
        # 5. 가중치(Weight) 기반 진행률 계산용 매핑 데이터
        # =============================================
        # 전체 동작의 총합(115) 대비 현재 완료된 동작의 가중치 합으로 정밀한 % 진행률을 도출하기 위함
        self.command_weight = {
            'basic_pose': 5, 'get_shovel': 5, 'make_furrow': 30, 'getback_shovel': 5,
            'digging': 20, 'seeding': 20, 'cover_dirt': 15, 'stamp_dirt': 15
        }

        # Firebase db.listen() 등록 시, 기존에 존재하던 데이터로 인해 첫 콜백이 발생
        # 이를 무시하고 프로그램 기동 이후의 신규 이벤트만 처리하기 위한 안전장치 플래그
        self._ignore_first_command_event = True

        # 노드 시작 시 Firebase 내부의 잔여 명령(command) 노드를 공백으로 초기화
        try:
            self.cmd_ref.set({})
        except Exception:
            pass

        # Firebase의 listen() 메서드는 블로킹(동기식 대기) 방식으로 동작
        # ROS 2 스핀(spin) 메인 루프를 방해하지 않도록 별도의 백그라운드 스레드(Daemon Thread)로 실행
        threading.Thread(target=self.subscribe_command, daemon=True).start()
    
    # ===================================================================================================================================================================
    
    def subscribe_command(self):
        """Firebase 'command' 노드를 상시 모니터링하는 콜백 리스너 메서드"""
        
        def callback(event):
            # 앱 구동 시 최초 전송되는 데이터는 무시하고 플래그 전환
            if self._ignore_first_command_event:
                self._ignore_first_command_event = False
                return

            cmd_data = event.data
            if not cmd_data or not isinstance(cmd_data, dict):
                return

            # 수신 데이터 딕셔너리에서 cmd(명령 종류)와 task(대상 작업) 파싱
            cmd = cmd_data.get('cmd')
            task = cmd_data.get('task')

            # ------------------------------------
            # [CASE 1] START : 작업 시퀀스 최초 시작
            # ------------------------------------
            if cmd == "start" and not self.sequence_active:
                self.get_logger().info(f"Firebase START received for task: {task}")
                self.logs_ref.push({
                    'time': time.strftime('%H:%M:%S'),
                    'type': 'cmd',
                    'msg': f'Start command received: {task}'
                })
                self.start_sequence()

            # -----------------------------------------------
            # [CASE 2] PAUSE : 시퀀스 일시정지 및 현재 액션 취소
            # -----------------------------------------------
            elif cmd == "pause":
                if self.sequence_active and not self.paused:
                    self.paused = True # 일시정지 플래그 활성화 (다음 액션 송신 차단)
                    
                    # 현재 실행 중인 ROS 2 액션이 있다면 서버에 비동기 취소(Cancel Goal) 요청
                    if self.current_goal_handle:
                        try:
                            cancel_future = self.current_goal_handle.cancel_goal_async()
                            # 취소 완료 후 별도 처리가 필요 없으므로 빈 람다(lambda) 콜백을 바인딩
                            cancel_future.add_done_callback(lambda f: None)
                        except Exception:
                            pass

                    self.get_logger().info("Firebase PAUSE received - sequence paused")
                    self.logs_ref.push({
                        'time': time.strftime('%H:%M:%S'), 'type': 'warn', 'msg': '작업 일시정지'
                    })
                    self.status_ref.update({'status': '일시정지'})

            # --------------------------------------------------
            # [CASE 3] CONTINUE : 일시정지 해제 및 작업 시퀀스 복귀
            # --------------------------------------------------  
            elif cmd == "continue":
                if self.sequence_active and self.paused:
                    self.paused = False # 일시정지 플래그 해제
                    self.get_logger().info("Firebase CONTINUE received - sequence resumed")
                    self.logs_ref.push({
                        'time': time.strftime('%H:%M:%S'), 'type': 'cmd', 'msg': '작업 재개'
                    })
                    self.status_ref.update({'status': '작업 진행 중'})
                    # 멈췄던 인덱스 포인트 지점부터 다시 명령 송신 프로세스 재개
                    self._send_current_command()

            # -----------------------------------------------------------------
            # [CASE 4] STOP : 시퀀스 완전 중단 및 제어 변수 초기화 (긴급 정지 상태)
            # -----------------------------------------------------------------
            elif cmd == "stop":
                self.sequence_active = False
                self.paused = False
                self.command_con1 = 0 # 2중 루프 인덱스 완전 초기화
                self.command_con2 = 0

                # 구동 중인 액션이 있다면 즉시 비동기 취소 요청
                if self.current_goal_handle:
                    try:
                        self.current_goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                    self.current_goal_handle = None

                self.status_ref.update({'status': '대기 중', 'progress': 0})
                self.logs_ref.push({
                    'time': time.strftime('%H:%M:%S'), 'type': 'error', 'msg': '긴급정지 → 작업 초기 상태로 복귀'
                })
                self.get_logger().warn("EMERGENCY STOP → back to idle")

            # 수신된 명령의 중복 실행을 방지하기 위해 Firebase의 해당 'command' 노드를 빈 객체로 리셋
            try:
                self.cmd_ref.set({})
            except Exception:
                pass

        # Firebase 노드 감시 시작 및 이벤트 발생 시 내부 callback 함수 실행 등록
        self.cmd_ref.listen(callback)

    def start_sequence(self):
        """전체 작업 인덱스를 0으로 리셋하고 제어 엔진을 기동하는 메서드"""
        if self.sequence_active:
            return
        self.command_con1 = 0
        self.command_con2 = 0
        self.sequence_active = True
        self.paused = False
        self._send_current_command() # 최초 명령 송신 단계로 진입

    def _send_current_command(self):
        """현재 인덱스 상태(con1, con2)를 연산하여 타겟 액션 서버로 비동기 Goal을 전송하는 핵심 제어 루프"""
        # 활성화 상태가 아니거나 일시정지 상태라면 명령을 송신하지 않고 함수 종료
        if not self.sequence_active or self.paused:
            return

        # [종료 조건 1] 대분류 인덱스가 전체 범위를 초과한 경우 -> 전체 시퀀스 정상 완료 처리
        if self.command_con2 >= len(self.cmlist):
            self.sequence_active = False
            self.status_ref.update({'progress': 100, 'status': '작업 완료'})
            self.logs_ref.push({
                'time': time.strftime('%H:%M:%S'), 'type': 'cmd', 'msg': '작업 완료'
            })
            return

        current_key = self.cmlist[self.command_con2]        # 예: 'mf'
        current_cmd_list = self.commands[current_key]       # 예: ['basic_pose', 'get_shovel', ...]

        # [종료 조건 2] 소분류 인덱스가 현재 하위 리스트 길이를 초과한 경우 -> 다음 대분류 작업 단계로 승격
        if self.command_con1 >= len(current_cmd_list):
            self.command_con2 += 1   # 대분류 인덱스 증가
            self.command_con1 = 0   # 소분류 인덱스 영(0)점 조절
            self._send_current_command() # 재귀 호출(Recursive Call)을 통해 다음 명령 즉시 분석 및 전송
            return

        # 현재 인덱스 포인트에 매핑된 최종 서브 명령 문자열 추출
        current_cmd = current_cmd_list[self.command_con1]  
        client = self.act_clients[current_key]             # 해당 작업을 전당하는 매핑된 액션 클라이언트 객체 선택

        # 비동기 통신 전, 매핑된 ROS 2 액션 서버가 활성화되어 있는지 2.0초간 블로킹 테스트
        if not client.wait_for_server(timeout_sec=2.0):
            self._fail(f'Action server not available: {current_key}')
            return

        # 해당 인터페이스의 Goal 메시지 객체 생성 및 데이터 주입
        goal_msg = TaskSequence.Goal()
        goal_msg.signal = current_cmd 
        
        # ROS 2 액션 서버로 목표(Goal) 비동기 전송
        # - feedback_callback: 서버가 주기적으로 전송하는 중간 상태 피드백 데이터를 처리할 메서드 바인딩
        send_future = client.send_goal_async(goal_msg, feedback_callback=self._feedback_cb)
        # - add_done_callback: 서버로부터 Goal 수락(Accept)/거절(Reject) 응답(Future)이 완료되었을 때 실행될 콜백 바인딩
        send_future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg):
        """액션 서버가 하드웨어 제어 중 실시간 피드백 토픽을 발행(Publish)할 때마다 동적으로 호출되는 콜백"""
        fb = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {fb.status}')
        
        # 하드웨어 세부 피드백 메시지를 Firebase 실시간 로그에 push(누적 저장)
        self.logs_ref.push({
            'time': time.strftime('%H:%M:%S'), 'type': 'feedback', 'msg': fb.status
        })
        # 상태 변화가 감지될 때마다 진행률(%) 재연산 및 Firebase 갱신
        self.update_progress()

    def _goal_response_cb(self, future):
        """액션 서버가 클라이언트가 보낸 Goal 요청에 응답했을 때 호출되는 콜백"""
        goal_handle = future.result()
        
        # 서버측 거절(Reject) 처리 시 실패 메서드 호출 후 시퀀스 다운
        if not goal_handle.accepted:
            self._fail("Goal rejected")
            return
            
        # 수락(Accept) 시, 추후 캔슬(Cancel) 제어를 위해 전역 변수에 현재 활성 Goal Handle 보관
        self.current_goal_handle = goal_handle
        
        # 액션의 최종 완료(Result) 상태를 비동기식으로 수신 대기하고 완료 콜백 바인딩
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        """액션 서버가 최종 결과(Succeeded/Aborted/Canceled)를 반환했을 때 호출되는 콜백"""
        try:
            res = future.result()
        except Exception as e:
            # 사용자가 일시정지를 눌러 발생한 의도된 액션 취소 예외(Exception)라면 에러 처리를 하지 않고 무시
            if self.paused:
                self.current_goal_handle = None
                return
            self._fail(f"Result exception: {e}")
            return

        # 결과 수신 시점에 이미 일시정지 상태로 플래그가 바뀌어 있다면 인덱스를 증가시키지 않고 루프 차단
        if self.paused:
            self.current_goal_handle = None
            return

        result = getattr(res, "result", None)

        # 액션 결과 내 내부 비즈니스 로직 판정이 실패(False)인 경우 예외 실패 처리
        if hasattr(result, "accepted") and not result.accepted:
            self._fail(getattr(result, "message", "Action failed"))
            return

        # [단일 태스크 성공 수렴]
        self.current_goal_handle = None
        self.command_con1 += 1       # 하위 소분류 인덱스를 1 증가시켜 다음 세부 명령 지정
        self._send_current_command() # 메인 제어 루프 메서드를 다시 호출하여 연속 순차 제어 달성

    def update_progress(self):
        """딕셔너리에 매핑된 가중치(Weight) 데이터를 기반으로 전체 진행 상황을 백분율로 연산하는 수학적 메서드"""
        # 1. 이미 완료되어 지나간 대분류(con2 이전 단계들) 내부의 모든 소분류 가중치들을 합산
        done = sum(
            self.command_weight[self.commands[self.cmlist[i]][j]]
            for i in range(self.command_con2)
            for j in range(len(self.commands[self.cmlist[i]]))
        )
        
        # 2. 현재 실행 중인 대분류(con2 단계) 내에서 이미 성공적으로 완료된 소분류(con1 이전 단계들) 가중치 추가 합산
        if self.sequence_active and not self.paused:
            done += sum(
                self.command_weight[self.commands[self.cmlist[self.command_con2]][j]]
                for j in range(self.command_con1)
            )
            
        # 3. 딕셔너리에 내장된 전체 가중치 파라미터의 절대 총합 연산
        total = sum(self.command_weight.values())
        
        # 4. 백분율 계산 및 정수형 데이터 타입캐스팅
        progress = int(done / total * 100) if total > 0 else 0
        
        # 연산 결과를 Firebase DB에 일괄 JSON 형태로 원격 업데이트
        self.status_ref.update({
            'progress': progress,
            'status': '작업 진행 중' if not self.paused else '일시정지'
        })

    def _fail(self, msg):
        """액션 타임아웃, 거절, 내부 예외 상황 발생 시 노드의 제어 플래그를 내리고 에러 로그를 리포팅하는 에러 핸들러"""
        self.sequence_active = False
        self.paused = False
        self.current_goal_handle = None
        # Firebase DB 및 ROS 2 터미널 콘솔창에 고유 에러 상태 기록 및 진행률 제로 리셋
        self.logs_ref.push({'time': time.strftime('%H:%M:%S'), 'type': 'error', 'msg': msg})
        self.status_ref.update({'status': msg, 'progress': 0})
        self.get_logger().error(msg)


def main():
    rclpy.init() # ROS 2 파이썬 통신 하부 레이어 초기화
    
    # 순차 제어 클라이언트 노드 객체화
    task_client_node = TaskSequenceClient()

    # 중요: Firebase 리스너 스레드 자원과 ROS 2 액션 비동기 콜백 함수들이 
    # 교착 상태(Deadlock)에 빠지지 않고 동시다발적으로 병렬 처리될 수 있도록 멀티스레드 실행기 적용
    executor = MultiThreadedExecutor()
    executor.add_node(task_client_node)

    try:
        executor.spin() # 프로그램 강제 종료 시까지 비동기 이벤트 큐(Queue) 무한 스핀 대기
    finally:
        # 종료 스크립트: 할당 노드 객체 소멸 및 ROS 2 컨텍스트 클린 다운
        task_client_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
