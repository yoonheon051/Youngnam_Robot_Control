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

global tpos
global tpos_above
global gpos
global gpos_above
global hpos


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
JSON_PATH = os.path.join(BASE_DIR, "data.json")

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1_1"

# 이동 속도 및 가속도
VELOCITY = 60
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp  # 필요한 기능만 임포트

    # 설정된 상수 출력
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
    global tpos
    global tpos_above
    global gpos
    global gpos_above
    global hpos

    with open(JSON_PATH,"r") as f:
        posdata = json.load(f)

    hpos = posdata["home"]
    tpos = posdata["tool"]
    gpos = posdata["startgo"]

    tpos_above = tpos.copy()
    tpos_above[0] += 25
    tpos_above[2] = 650

    gpos_above = gpos.copy()
    gpos_above[2] = 650

def grab():
    from DSR_ROBOT2 import set_digital_output, wait, ON, OFF
    #그리퍼 닫기
    set_digital_output(1,ON)
    set_digital_output(2,OFF)
    wait(1)

def release():
    from DSR_ROBOT2 import set_digital_output, wait, OFF
    #그리퍼 열기
    set_digital_output(1,OFF)
    set_digital_output(2,OFF)
    wait(0.25)

def basic_pose():
    from DSR_ROBOT2 import movej,mwait
    bpose = [0,0,45,0,135,-90]
    release()
    movej(bpose,time=1.5,vel=VELOCITY,acc=ACC)
    mwait(0.25)
    
def get_shovel():
    global tpos
    global tpos_above
    mforward_Tpos = [70,0,20,0,0,0]
    poslist = [tpos_above,tpos,mforward_Tpos,tpos_above]
    from DSR_ROBOT2 import movel,wait,posx,DR_MV_MOD_REL

    i = 0
    while(i<len(poslist)):
        if(i == 2):
            movel(posx(poslist[i]),time=1.5,vel=VELOCITY,acc=ACC,mod=DR_MV_MOD_REL)
            wait(0.25)
        else:
            movel(posx(poslist[i]),time=1.5,vel=VELOCITY,acc=ACC)
            wait(0.25)
            if(i==0):
                release()
            elif(i==1):
                grab()
        i += 1

def make_furrow():
    from DSR_ROBOT2 import movel,wait,mwait,posx,DR_MV_MOD_REL,DR_FC_MOD_REL,DR_BASE,move_periodic,task_compliance_ctrl,release_force,set_desired_force,release_compliance_ctrl

    movel(posx(gpos_above),time=1,vel=VELOCITY,acc=ACC)
    movel(posx(gpos),time=1,vel=VELOCITY,acc=ACC)
    setcomp = [40,3000,3000,200,200,200]
    setfd = [130, 0, 0, 0, 0, 0]
    setfdir = [2, 0, 0, 0, 0, 0]

    task_compliance_ctrl(stx=setcomp)
    wait(0.25)
    set_desired_force(fd=setfd, dir=setfdir, mod=DR_FC_MOD_REL)

    # 주기 2초 반복:6회,1초
    amp = [0,0,0,15,0,0]
    move_periodic(amp,period=1,repeat=5,ref=DR_BASE)
    release_force()
    wait(0.25)
    release_compliance_ctrl()
    movel(posx(gpos),time = 1.5,vel=VELOCITY,acc=ACC,ref=DR_BASE)

def getback_shovel():
    global tpos
    global tpos_above
    global gpos
    global gpos_above

    from DSR_ROBOT2 import movel,mwait,posx,DR_MV_MOD_REL,DR_BASE

    poslist = [gpos_above,tpos_above,tpos,tpos_above]
    i = 0
    while(i<len(poslist)):
        movel(posx(poslist[i]),time = 1.5,vel=VELOCITY,acc=ACC,ref=DR_BASE)
        mwait(0.25)
        if(i==2):
            release()
        i += 1
    basic_pose()
    
class makefurrow(Node):
    def __init__(self):
        super().__init__('make_furrow_action')
        self._server = ActionServer(
            self, TaskSequence, 'Task_Sequence_mf',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
        )
        self.get_logger().info('Action Server ready: /Task_Sequence_mf')

    def goal_callback(self, goal_request: TaskSequence.Goal):
        self.get_logger().info(f'Goal received: signal="{goal_request.signal}"')
        if goal_request.signal == "basic_pose":
            return GoalResponse.ACCEPT
        elif goal_request.signal == "get_shovel":
            return GoalResponse.ACCEPT
        elif goal_request.signal == "make_furrow":
            return GoalResponse.ACCEPT
        elif goal_request.signal == "getback_shovel":
            return GoalResponse.ACCEPT
        else:
            return GoalResponse.REJECT

    def execute_callback(self, goal_handle):
        signal = goal_handle.request.signal
        feedback = TaskSequence.Feedback()
                  
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
        
        feedback.status = f'processing : "{signal}"...'
        goal_handle.publish_feedback(feedback)
        time.sleep(0.3)

        goal_handle.succeed()
        result = TaskSequence.Result()
        result.accepted = True
        result.message = f'"{signal}"is End.'
        self.get_logger().info(f'Result: {result.message}')
        return result
        
    
def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("make_furrow", namespace=ROBOT_ID)
    nodeR = node
    nodeA = makefurrow()
    executor = MultiThreadedExecutor()
    executor.add_node(nodeR)
    executor.add_node(nodeA)

    # DR_init에 노드 설정
    DR_init.__dsr__node = nodeR

    try:
        # 초기화는 한 번만 수행
        initialize_robot()
        loadpos()
        executor.spin()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        nodeR.destroy_node()
        nodeA.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()