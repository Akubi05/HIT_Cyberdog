# send_cmd的duration要调为0.md
```
› 查看   
def _send_command(self, mode, gait_id, velocity, step_height=None,rpy=None):
          self.robot._send_cmd(
              mode=mode,
              gait_id=gait_id,
              vel=list(velocity),
              rpy=list(rpy or [0.0, 0.0, 0.0]),
              step_height=list(step_height or [0.0, 0.0]),
              duration=STAGE_THREE_CMD_DURATION_SEC,
          )
其中duration这个参数的c++底层源码实现是如何调用的.当我持续发duration不为0命令会有很大的延迟,但是发duration=0的命令就没有延迟.
```
## codex思考过程

我会先在仓库里定位 _send_command、_send_cmd 以及 C++ 绑定/底层实现，沿着duration 参数一路追到实际调用点，再判断为什么非 0 持续发送会引入延迟。

目前看到 Python 侧 stage_runtime.py 会把秒转成毫秒写进 msg.duration，stage3_process.py 现在常量是 0.0。下一步我会解压/查看仓库里的 locomotion 源码包，找 msg.duration 在 C++ 节点里怎么被消费。

Python 侧链路已经明确：_send_command -> robot._send_cmd -> CommandClient.send-> _make_msg，这里 duration 单位从秒变成毫秒。也就是说你发 0.15，底层 LCM 字段收到的是 150，不是浮点秒。

底层不是把 duration 当作“过期时间戳”或“网络发送等待”，而是把它放进控制命令队列，PrepareCmd() 每个控制周期按 interface_iter_ >= duration 判断当前队列头是否执行完。持续发非 0 命令时，队列就会保留/追加未完成命令，这正是延迟来源。

C++ 侧关键线索已经出现：LCM 的 duration 注释是 “Time of command execution”，进入 CommandInterface 后会被除以 2，变成控制循环迭代计数；非 0 时会进入命令队列/运动列表式执行逻辑。我要再打开具体函数，确认它是否会等待当前命令执行完才切下一条。

底层不是把 duration 当作“过期时间戳”或“网络发送等待”，而是把它放进控制命令队列，PrepareCmd() 每个控制周期按 interface_iter_ >= duration 判断当前队列头是否执行完。持续发非 0 命令时，队列就会保留/追加未完成命令，这正是延迟来源。

还需要再确认 command_.duration 进入控制器后有没有额外插值/滤波逻辑。现在已能解释队列延迟，但我会查一下 FSM/locomotion 是否又把 duration_mode 打开，避免漏掉第二层延迟。

# 回答

查到了，duration 的底层调用链是：

_send_command()
-> stage_runtime.py:1618 robot._send_cmd()
-> stage_runtime.py:729 CommandClient.send()
-> stage_runtime.py:464 _make_msg()
-> stage_runtime.py:500：

msg.duration = int(max(float(duration or 0.0), 0.0) * 1000.0)

所以 Python 这里传的是“秒”，LCM 包里变成“毫秒”。比如 duration=0.15 会发成150。

C++ 底层在 cyberdog_locomotion.zip 里的关键实现是：

void CommandInterface::ProcessLcmCommand(const robot_control_cmd_lcmt*
lcm_cmd) {
    memcpy(&lcm_cmd_, lcm_cmd, sizeof(robot_control_cmd_lcmt));
    lcm_timer_.StartTimer();
}

然后每个控制周期 RobotRunner::SetupStep() 调：

cmd_interface_->PrepareCmd(...);

在 CommandInterface::Lcm2Cmd() 里真正消费：

cmd_cur_.duration = lcm_cmd_.duration / 2;

因为机器人控制周期是 controller_dt = 0.002s，也就是 2ms 一次。所以底层把毫秒 /2 转成“控制循环次数”。例如：

Python duration=0.15s
LCM msg.duration=150ms
C++ cmd_cur_.duration=75
75 * 2ms = 150ms

你遇到的延迟原因在这里：

if (cmd_cur_.duration == 0 && cmd_cur_.mode != MotionMode::kMotion) {
    while (!cmd_list_.empty())
        cmd_list_.pop();
    interface_iter_ = 0;
    ...
} else {
    while (cmd_list_.size() > 0) {
        if (cmd_list_.front().duration == 0) {
            cmd_list_.pop();
            interface_iter_ = 0;
        } else {
            break;
        }
    }
}
cmd_list_.push(cmd_cur_);

结论：duration=0 时，每次新包都会清空 cmd_list_，最新命令立即覆盖旧命令，所以没有明显延迟。duration!=0 时，新命令会进入队列，旧的非 0 duration 命令不会被清掉，必须等队头命令执行完才轮到下一条。

你现在 Python 侧 heartbeat 是 stage_runtime.py:68 默认 0.02s，也就是 20ms 发一次。如果你持续发 duration=0.15，底层等于每 20ms 追加一个“要执行 150ms 的命令”，队列一定越堆越长，速度/转向变化就会越来越滞后。

建议：

对实时遥控、视觉闭环、连续修正速度的命令，应该保持：

STAGE_THREE_CMD_DURATION_SEC = 0.0

也就是你现在 stage_processes/stage3_process.py:53 的设置是对的。duration!=0 更适合一次性动作、motion list、固定动作片段，不适合高频连续速度控制。


核心逻辑在 C++ CommandInterface::Lcm2Cmd()：

  cmd_cur_.duration = lcm_cmd_.duration / 2;

  if (cmd_cur_.duration == 0 && cmd_cur_.mode != MotionMode::kMotion) {
      while (!cmd_list_.empty())
          cmd_list_.pop();
      interface_iter_ = 0;
      motion_list_step_ = 0;
      motion_list_size_ = 0;
  } else {
      while (cmd_list_.size() > 0) {
          if (cmd_list_.front().duration == 0) {
              cmd_list_.pop();
              interface_iter_ = 0;
          } else {
              break;
          }
      }
  }

  cmd_list_.push(cmd_cur_);

  然后 PrepareCmd() 每个 2ms 控制周期检查队头：

  if (interface_iter_ >= command_.duration && command_.duration != 0) {
      cmd_list_.pop();
      ...
      interface_iter_ = 0;
  }

  所以行为就是：

  duration = 0：清空旧队列，只保留最新命令，适合实时速度控制。

  duration != 0：作为有时长的动作片段排队执行，旧命令不完成就不会切到新命令，连
  续高频发送会堆积延迟。

  你的 Stage3 / 视觉闭环这种持续修正速度的场景，应该用 duration=0，用 Python 侧循环频率控制“持续多久发命令”，不要让 C++ 队列用 duration 管持续时间。