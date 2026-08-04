[TOC]

```
demo.py
'''

This demo show the communication interface of MR813 motion control board based on Lcm.
Dependency: 
- robot_control_cmd_lcmt.py
- robot_control_response_lcmt.py
'''
import lcm
import sys
import os
import time
from threading import Thread, Lock

from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt

def main():
    Ctrl = Robot_Ctrl()
    Ctrl.run()
    msg = robot_control_cmd_lcmt()
    try:
        msg.mode = 12 # Recovery stand
        msg.gait_id = 0
        msg.life_count += 1 # Command will take effect when life_count update
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(12, 0)

        msg.mode = 62 # Shake hand, based on position interpolation control
        msg.gait_id = 2
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(62, 2)

        msg.mode = 64 # Twoleg Stand
        msg.gait_id = 0
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(64, 0)

        msg.mode = 21 # Position interpolation control
        msg.gait_id = 0
        msg.rpy_des = [0, 0.3, 0] # Head up
        msg.duration = 500 # Expected execution time, 0.5s 
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep( 0.5 )

        msg.mode = 21 # Position interpolation control
        msg.gait_id = 0
        msg.rpy_des = [0, -0.3, 0] # Head down
        msg.duration = 300 
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep( 0.3 )

        msg.mode = 21 # Position interpolation control
        msg.gait_id = 5
        msg.rpy_des = [0, 0, 0]
        msg.pos_des = [0, 0, 0.22] # Set body height
        msg.duration = 400 
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep( 1 )

        msg.mode = 11 # Locomotion
        msg.gait_id = 26 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        msg.vel_des = [0, 0, 0.5] #转向
        msg.duration = 0 # Zero duration means continuous motion until a new command is used.
                         # Continuous motion can interrupt non-zero duration interpolation motion
        msg.step_height = [0.06, 0.06] # ground clearness of swing leg
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep( 5 )

        msg.mode = 7    # PureDamper
        msg.gait_id = 0
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(7, 0)

    except KeyboardInterrupt:
        pass
    Ctrl.quit()
    sys.exit()


class Robot_Ctrl(object):
    def __init__(self):
        self.rec_thread = Thread(target=self.rec_responce)
        self.send_thread = Thread(target=self.send_publish)
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.cmd_msg = robot_control_cmd_lcmt()
        self.rec_msg = robot_control_response_lcmt()
        self.send_lock = Lock()
        self.delay_cnt = 0
        self.mode_ok = 0
        self.gait_ok = 0
        self.runing = 1

    def run(self):
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.send_thread.start()
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        self.rec_msg = robot_control_response_lcmt().decode(data)
        if(self.rec_msg.order_process_bar >= 95):
            self.mode_ok = self.rec_msg.mode
        else:
            self.mode_ok = 0

    def rec_responce(self):
        while self.runing:
            self.lc_r.handle()
            time.sleep( 0.002 )

    def Wait_finish(self, mode, gait_id):
        count = 0
        while self.runing and count < 2000: #10s
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.005)
                count += 1

    def send_publish(self):
        while self.runing:
            self.send_lock.acquire()
            if self.delay_cnt > 20: # Heartbeat signal 10HZ, It is used to maintain the heartbeat when life count is not updated
                self.lc_s.publish("robot_control_cmd",self.cmd_msg.encode())
                self.delay_cnt = 0
            self.delay_cnt += 1
            self.send_lock.release()
            time.sleep( 0.005 )

    def Send_cmd(self, msg):
        self.send_lock.acquire()
        self.delay_cnt = 50
        self.cmd_msg = msg
        self.send_lock.release()

    def quit(self):
        self.runing = 0
        self.rec_thread.join()
        self.send_thread.join()

# Main function
if __name__ == '__main__':
    main()
```
# 我把main的最后两行代码 Ctrl.quit() sys.exit()去掉会怎么样

If you remove the `Ctrl.quit()` and `sys.exit()` lines from the end of the `main` function, the script will not terminate properly.

Here's a breakdown of what happens:

1.  **`Ctrl.quit()` is not called:** The `quit()` method in the `Robot_Ctrl` class is responsible for cleanup. It sets the `self.runing` flag to `0`, which signals the `rec_responce` and `send_publish` threads to stop their `while` loops. It then uses `.join()` to wait for these threads to finish executing before proceeding. Without this call, the `self.runing` flag remains `1`, and both the receiving and sending threads will continue to run indefinitely in the background.

2.  **`sys.exit()` is not called:** The `main` function will complete its execution, but the entire Python script will not exit because the two daemon threads (`rec_thread` and `send_thread`) are still alive and running. A Python script only exits when all non-daemon threads have finished.

In short, removing those two lines will cause the main part of your script to finish, but the program will hang, kept alive by the endlessly running communication threads. You would have to manually stop the process (e.g., with Ctrl+C).

# 我使用ctrl+c强制终止程序而不使用quit()和exit()会怎样？ 

Excellent question. This gets to the heart of how exception handling and threads interact in Python.

If you press `Ctrl+C` to terminate the program, and you have **not** implemented the `Ctrl.quit()` and `sys.exit()` calls, here is what will happen based on your code:

1.  **`KeyboardInterrupt` is Raised:** Pressing `Ctrl+C` sends a signal that the Python interpreter catches and translates into a `KeyboardInterrupt` exception. This exception is raised in the program's **main thread**.

2.  **The Exception is Caught:** Your main execution logic is wrapped in a `try...except KeyboardInterrupt:` block.

    ```python
    try:
        # ... all your robot commands ...
        time.sleep( 5 )

        # These lines are assumed to be removed for this scenario
        # msg.mode = 7
        # ...

    except KeyboardInterrupt:
        pass # <--- THIS IS THE CRITICAL PART

    # These lines are also assumed to be removed
    # Ctrl.quit()
    # sys.exit()
    ```

    When `KeyboardInterrupt` is raised, the program flow immediately jumps to the `except` block. Because this block contains only `pass`, the exception is "handled" by doing nothing. The program does **not** crash.

3.  **Main Function Finishes:** After the `except` block, there is no more code to run in the `main` function (since we're assuming `Ctrl.quit()` and `sys.exit()` were removed). The `main` function will simply finish and exit.

4.  **Background Threads Keep Running:** This is the key problem. The `main` thread finishing **does not** stop the other threads you started (`rec_thread` and `send_thread`).

      * You never called `Ctrl.quit()`, so the `self.runing` flag was never set to `0`.
      * The `while self.runing:` loop inside both the `rec_responce` and `send_publish` methods will continue to run forever.
      * Because these are not "daemon" threads, they will keep the Python process alive even after the main thread has finished.

### Summary of the Outcome

Using `Ctrl+C` once will cause the `main` function to stop executing commands and exit cleanly, but **the program itself will hang and not terminate**. It will be kept alive indefinitely by the two background threads that are stuck in their infinite loops.

You would then need to send another, more forceful termination signal (like pressing `Ctrl+C` a second time, or using a task manager) to actually kill the process.

### Contrast with a Graceful Shutdown

  * **Intended (Graceful) Shutdown:** The `Ctrl.quit()` method is the "graceful" way to stop. It tells the threads "please finish up" by setting `self.runing = 0`, and then waits for them to do so with `.join()`. This ensures all loops terminate and resources are released properly before the program exits.

  * **Forceful Shutdown (`Ctrl+C` without `quit()`):** You are bypassing the entire cleanup mechanism. The threads are not told to stop; they are left running in the background until the process is killed externally. This can sometimes leave resources (like network sockets) in an indeterminate state, although the operating system is generally good at cleaning them up when a process is killed.

# 程序先执行到try里面，然后按ctrl+c 

Of course. Let's walk through the exact sequence of events for that scenario.

Your scenario is: The program is running the code inside the `try` block, and you press `Ctrl+C`.

Here is the step-by-step breakdown:

---

### 执行流程详解 (Detailed Execution Flow)

**第1步：程序启动 (Program Starts)**

1.  `main()` 函数被调用。
2.  `Ctrl = Robot_Ctrl()` 创建一个机器人控制对象。
3.  `Ctrl.run()` 被调用，这会立即启动两个独立的后台线程：
    * `rec_thread` (接收线程): 开始运行 `rec_responce` 方法，进入 `while self.runing:` 循环，等待接收 LCM 消息。
    * `send_thread` (发送线程): 开始运行 `send_publish` 方法，进入 `while self.runing:` 循环，准备以10Hz的频率发送心跳信号。
4.  此时，你的程序有**三个线程**在同时运行：主线程（执行 `main` 函数中的命令）和两个后台线程。

**第2步：主线程进入 `try` 代码块 (Main Thread Enters the `try` Block)**

1.  主线程开始顺序执行 `try` 块中的代码。
2.  它会发送 `mode = 12` 的命令，然后可能在 `Ctrl.Wait_finish(12, 0)` 中等待。
3.  然后发送 `mode = 62` 的命令，等等。
4.  假设程序正在执行 `time.sleep(5)` 这一行，主线程会在这里暂停5秒。**在此期间，两个后台线程仍在不停地运行。**

**第3步：用户按下 `Ctrl+C` (User Presses `Ctrl+C`)**

1.  **中断信号产生**: 当你按下 `Ctrl+C`，操作系统会向 Python 解释器发送一个中断信号。
2.  **异常被抛出**: Python 解释器在**主线程**中立即抛出一个 `KeyboardInterrupt` 异常。
3.  **执行被中断**: 主线程在 `try` 块中的当前位置（比如 `time.sleep(5)` 的中间）的执行被**立即打断**。它不会再继续执行 `try` 块中的后续代码。

**第4步：跳转到 `except` 代码块 (Control Jumps to the `except` Block)**

1.  Python 寻找这个异常的处理器。它找到了你写的 `except KeyboardInterrupt:`。
2.  程序控制权立刻跳转到 `except` 块内部。
3.  `except` 块中的代码 `pass` 被执行。`pass` 是一个空操作，它意味着 “我已经捕获了这个异常，并且选择什么都不做，请继续正常执行”。
4.  **关键点**：因为异常被成功捕获和处理了，所以程序**不会崩溃**。

**第5步：主线程结束 (The Main Thread Finishes)**

1.  在 `except` 块执行完毕后，程序会继续执行 `try...except` 结构**之后**的代码。
2.  在你的这个假设场景中，`Ctrl.quit()` 和 `sys.exit()` 都被删除了，所以 `try...except` 结构后面已经没有任何代码了。
3.  因此，`main` 函数执行完毕，主线程退出了。

**第6步：最终结果：程序挂起 (Final Result: The Program Hangs)**

1.  主线程虽然退出了，但是它**没有通知**两个后台线程停止。
2.  `Ctrl.quit()` 方法从未被调用，所以 `self.runing` 标志位**仍然是 `1`**。
3.  `rec_thread` 和 `send_thread` 的 `while self.runing:` 循环条件依然为真，所以它们会**永远地运行下去**。
4.  在 Python 中，只要有任何一个非守护（non-daemon）线程在运行，整个程序就不会退出。
5.  **结论：** 你的终端会看起来像程序卡住了一样。它不会退出，也不会再打印任何东西（除非后台线程有打印语句）。你必须再次按下 `Ctrl+C` 或使用系统命令来强制杀死这个挂起的进程。

### 一个比喻 (An Analogy)

想象一下：
* **主线程**是一位项目经理。
* **两个后台线程**是两位工人（一位负责接收邮件，一位负责定期发报告）。
* `try` 块里的代码是经理今天需要完成的一系列任务清单。

经理开始工作（启动了两位工人），然后开始按清单逐项处理任务。当他在喝咖啡休息时（`time.sleep()`），你（用户）按下了 `Ctrl+C`，这相当于他办公室的火警响了 (`KeyboardInterrupt`)。

他看了一眼火警（`except` 块），发现是误报，于是按下了 `pass`（关闭了警报）。然后他就觉得今天的工作被打断了，直接下班回家了（主线程退出）。

**问题是：他下班时忘了去通知那两位还在工作的工人。** 所以，接收邮件和发送报告的工人因为没有收到下班通知 (`self.runing = 0`)，会永远地工作下去，导致整个公司（程序）的灯一直亮着，无法关闭。

# 我再次运行这个程序，还会正常工作吗 

是的，**大概率会正常工作**。

当你再次运行这个程序时，操作系统会创建一个全新的、独立的进程。这个新进程与之前那个被你强制中断后“挂起”的旧进程是完全隔离的。

让我们分析一下可能发生冲突的地方，以及为什么它通常不会成为问题：

### 1. 进程和内存 (Processes and Memory)

* **隔离性**: 新进程有自己独立的内存空间和线程。它完全不知道旧进程的存在。
* **结论**: 这方面完全没有问题。

### 2. 网络端口 (The Network Port) - 最可能出问题的地方

* **潜在冲突**: 你的程序使用了固定的UDP端口 `7670` 和 `7671` 进行LCM通信。通常情况下，如果一个程序（那个挂起的旧进程）已经占用了某个端口，新程序再尝试使用同一个端口时会失败，并报告“地址已在使用中 (Address already in use)”的错误。

* **为什么它依然能工作**: 幸运的是，LCM使用的UDP**多播 (multicast)**通信方式有一个特殊的设计。它允许**多个进程绑定到同一个多播地址和端口上**并同时监听。这在技术上是通过一个叫做 `SO_REUSEADDR` 的套接字选项实现的，大多数LCM库都会默认启用它。
    * **监听 (Receiving)**: 你的新程序可以成功地开始监听端口`7670`，即使旧程序也在监听。它们都会收到发送到该端口的数据。
    * **发送 (Sending)**: 两个程序都可以向端口`7671`发送数据，这不会产生冲突。

**结论**: 因为LCM和操作系统的这个特性，新程序几乎总能成功启动并建立通信，不会因为旧进程占用端口而失败。

---

### 需要注意的副作用 (Potential Side Effects)

尽管新程序可以运行，但之前不规范的退出方式会留下一些隐患：

1.  **资源消耗 (Resource Consumption)**
    那个被挂起的旧进程仍然存在于系统的后台，它没有消失。它仍在占用着CPU时间和内存。如果你反复这样做，系统中会累积很多僵尸进程，最终可能会拖慢整个系统（你的电脑或者机器人主控板）的运行速度。

2.  **通信干扰 (Communication Interference)**
    这是一个更微妙的问题。现在网络中有两个程序在和机器人通信：
    * **旧进程**: 它的 `send_thread` 还在运行，会持续发送它被中断前的最后一个心跳指令（`cmd_msg`）。因为`life_count`不再更新，机器人控制器很可能会忽略这些旧的指令。
    * **新进程**: 它会发送带有新的、递增的 `life_count` 的指令。机器人应该会响应这些新指令。

    在绝大多数情况下，由于 `life_count` 机制的存在，机器人可以正确区分新旧指令。但这是一个不干净的网络环境，存在潜在的、难以预料的风险。

### 最佳实践建议 (Best Practice Recommendation)

**不要依赖强制中断。**

那个被你去掉的 `Ctrl.quit()` 方法至关重要。它提供了一个“**优雅退出 (graceful shutdown)**”的机制：
1.  通知所有后台线程停止工作 (`self.runing = 0`)。
2.  等待它们完成最后的任务并完全退出 (`.join()`)。
3.  干净地释放所有资源。

**总结：再次运行程序基本上是可行的，但之前留下的烂摊子（挂起的旧进程）仍在消耗系统资源，并可能造成轻微的通信噪音。养成良好习惯，总是通过调用 `Ctrl.quit()` 这样的清理函数来正常地终止程序，是编写稳定、可靠的机器人程序的关键。**

# 我在try中按ctrl+c会到finally吗 

```
except KeyboardInterrupt:
print("中断操作")
finally:
msg.mode = 12
Ctrl.Send_cmd(msg)
Ctrl.quit()
executor.shutdown()
cv2.destroyAllWindows()
sys.exit() 
```

是的，**一定会执行 `finally` 块中的代码**。

这是 Python 中 `try...except...finally` 结构一个非常重要的特性。`finally` 块的设计目的就是为了确保无论 `try` 块中发生了什么（无论是正常完成、发生异常被 `except` 捕获，还是发生了未被捕获的异常），它里面的代码都**必须**被执行。这通常用于执行关键的清理操作。

让我们根据你的代码来分解一下具体的执行顺序：

1.  **进入 `try` 块**: 你的程序正常执行 `try` 块里的代码。

2.  **按下 `Ctrl+C`**: 你在程序执行期间按下了 `Ctrl+C`。这会在主线程中立即引发一个 `KeyboardInterrupt` 异常。

3.  **跳转到 `except` 块**:
    * `try` 块的执行被立即中断。
    * Python 查找到能够处理这个异常的 `except KeyboardInterrupt:` 块。
    * 程序控制权跳转到这个 `except` 块，并执行其中的代码：`print("中断操作")`。

4.  **执行 `finally` 块**:
    * 在 `except` 块的代码执行完毕后，无论如何，程序控制权都会**继续进入 `finally` 块**。
    * `finally` 块中的所有代码都会被顺序执行：
        * `msg.mode = 12`
        * `Ctrl.Send_cmd(msg)` (尝试让机器人恢复站立姿态)
        * `Ctrl.quit()` (优雅地关闭后台线程)
        * `executor.shutdown()` (关闭线程池)
        * `cv2.destroyAllWindows()` (关闭所有OpenCV窗口)
        * `sys.exit()` (退出程序)

**总结：**

你写的这段代码是处理程序中断的**非常健壮和正确的范例**。通过将所有清理操作（如停止机器人、关闭线程、释放资源）放在 `finally` 块中，你可以保证即使用户通过 `Ctrl+C` 强行中断程序，这些关键的收尾工作也能够被执行，从而防止留下失控的线程或未关闭的资源，让程序能够干净利落地退出。