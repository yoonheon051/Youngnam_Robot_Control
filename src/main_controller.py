import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from cobot1_interfaces.action import TaskSequence
import firebase_admin
from firebase_admin import credentials, db
import threading
import time


class TaskSequenceClient(Node):
    def __init__(self):
        super().__init__('task_sequence_client')

        self.make_furrow_c = ['basic_pose', 'get_shovel', 'make_furrow', 'getback_shovel']
        self.digging_c = ['digging']
        self.seeding_c = ['seeding']
        self.cover_dirt_c = ['cover_dirt']
        self.tamping_c = ['stamp_dirt']

        self.commands = {
            'mf': self.make_furrow_c,
            'dg': self.digging_c,
            'sw': self.seeding_c,
            'cd': self.cover_dirt_c,
            'sd': self.tamping_c,
        }
        self.cmlist = ['mf', 'dg', 'sw', 'cd', 'sd']

        self.act_clients = {
            key: ActionClient(self, TaskSequence, f'Task_Sequence_{key}')
            for key in self.cmlist
        }

        self.sequence_active = False
        self.paused = False
        self.command_con2 = 0
        self.command_con1 = 0
        self.current_goal_handle = None

        if not firebase_admin._apps:
            cred = credentials.Certificate("cred/path")
            firebase_admin.initialize_app(cred, {
                'databaseURL': 'databaseURL'
            })

        self.status_ref = db.reference('robot_status')
        self.logs_ref = db.reference('logs')
        self.cmd_ref = db.reference('command')

        self.command_weight = {
            'basic_pose': 5,
            'get_shovel': 5,
            'make_furrow': 30,
            'getback_shovel': 5,
            'digging': 20,
            'seeding': 20,
            'cover_dirt': 15,
            'stamp_dirt': 15
        }

        self._ignore_first_command_event = True

        try:
            self.cmd_ref.set({})
        except Exception:
            pass

        threading.Thread(target=self.subscribe_command, daemon=True).start()

    def subscribe_command(self):
        def callback(event):
            if self._ignore_first_command_event:
                self._ignore_first_command_event = False
                return

            cmd_data = event.data
            if not cmd_data:
                return
            if not isinstance(cmd_data, dict):
                return

            cmd = cmd_data.get('cmd')
            task = cmd_data.get('task')

            if cmd == "start" and not self.sequence_active:
                self.get_logger().info(f"Firebase START received for task: {task}")
                self.logs_ref.push({
                    'time': time.strftime('%H:%M:%S'),
                    'type': 'cmd',
                    'msg': f'Start command received: {task}'
                })
                self.start_sequence()

            elif cmd == "pause":
                if self.sequence_active and not self.paused:
                    self.paused = True
                    if self.current_goal_handle:
                        try:
                            cancel_future = self.current_goal_handle.cancel_goal_async()
                            cancel_future.add_done_callback(lambda f: None)
                        except Exception:
                            pass

                    self.get_logger().info("Firebase PAUSE received - sequence paused")
                    self.logs_ref.push({
                        'time': time.strftime('%H:%M:%S'),
                        'type': 'warn',
                        'msg': '작업 일시정지'
                    })
                    self.status_ref.update({'status': '일시정지'})

            elif cmd == "continue":
                if self.sequence_active and self.paused:
                    self.paused = False
                    self.get_logger().info("Firebase CONTINUE received - sequence resumed")
                    self.logs_ref.push({
                        'time': time.strftime('%H:%M:%S'),
                        'type': 'cmd',
                        'msg': '작업 재개'
                    })
                    self.status_ref.update({'status': '작업 진행 중'})
                    self._send_current_command()

            elif cmd == "stop":
                self.sequence_active = False
                self.paused = False
                self.command_con1 = 0
                self.command_con2 = 0

                if self.current_goal_handle:
                    try:
                        self.current_goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                    self.current_goal_handle = None

                self.status_ref.update({'status': '대기 중', 'progress': 0})
                self.logs_ref.push({
                    'time': time.strftime('%H:%M:%S'),
                    'type': 'error',
                    'msg': '긴급정지 → 작업 초기 상태로 복귀'
                })
                self.get_logger().warn("EMERGENCY STOP → back to idle")

            try:
                self.cmd_ref.set({})
            except Exception:
                pass

        self.cmd_ref.listen(callback)

    def start_sequence(self):
        if self.sequence_active:
            return
        self.command_con1 = 0
        self.command_con2 = 0
        self.sequence_active = True
        self.paused = False
        self._send_current_command()

    def _send_current_command(self):
        if not self.sequence_active or self.paused:
            return

        if self.command_con2 >= len(self.cmlist):
            self.sequence_active = False
            self.status_ref.update({'progress': 100, 'status': '작업 완료'})
            self.logs_ref.push({
                'time': time.strftime('%H:%M:%S'),
                'type': 'cmd',
                'msg': '작업 완료'
            })
            return

        current_key = self.cmlist[self.command_con2]
        current_cmd_list = self.commands[current_key]

        if self.command_con1 >= len(current_cmd_list):
            self.command_con2 += 1
            self.command_con1 = 0
            self._send_current_command()
            return

        current_cmd = current_cmd_list[self.command_con1]
        client = self.act_clients[current_key]

        if not client.wait_for_server(timeout_sec=2.0):
            self._fail(f'Action server not available: {current_key}')
            return

        goal_msg = TaskSequence.Goal()
        goal_msg.signal = current_cmd
        send_future = client.send_goal_async(goal_msg, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {fb.status}')
        self.logs_ref.push({
            'time': time.strftime('%H:%M:%S'),
            'type': 'feedback',
            'msg': fb.status
        })
        self.update_progress()

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._fail("Goal rejected")
            return
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        try:
            res = future.result()
        except Exception as e:
            if self.paused:
                self.current_goal_handle = None
                return
            self._fail(f"Result exception: {e}")
            return

        if self.paused:
            self.current_goal_handle = None
            return

        result = getattr(res, "result", None)

        if hasattr(result, "accepted") and not result.accepted:
            self._fail(getattr(result, "message", "Action failed"))
            return

        self.current_goal_handle = None
        self.command_con1 += 1
        self._send_current_command()

    def update_progress(self):
        done = sum(
            self.command_weight[self.commands[self.cmlist[i]][j]]
            for i in range(self.command_con2)
                for j in range(len(self.commands[self.cmlist[i]]))
        )
        if self.sequence_active and not self.paused:
            done += sum(
                self.command_weight[self.commands[self.cmlist[self.command_con2]][j]]
                for j in range(self.command_con1)
            )
        total = sum(self.command_weight.values())
        progress = int(done / total * 100) if total > 0 else 0
        self.status_ref.update({
            'progress': progress,
            'status': '작업 진행 중' if not self.paused else '일시정지'
        })

    def _fail(self, msg):
        self.sequence_active = False
        self.paused = False
        self.current_goal_handle = None
        self.logs_ref.push({'time': time.strftime('%H:%M:%S'), 'type': 'error', 'msg': msg})
        self.status_ref.update({'status': msg, 'progress': 0})
        self.get_logger().error(msg)


def main():
    rclpy.init()
    task_client_node = TaskSequenceClient()

    executor = MultiThreadedExecutor()
    executor.add_node(task_client_node)

    try:
        executor.spin()
    finally:
        task_client_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
