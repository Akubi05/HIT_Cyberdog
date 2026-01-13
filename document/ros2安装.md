[TOC]

# ROS2安装
## 日期：2025.3.11
根据官方文档安装
https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html

可能出现的问题：
+ sudo curl -x https://127.0.0.1:7890 -l -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gp
出错
原因：未设置代理，利用clash for windows设置代理，通过命令行设置curl代理即可，比如指向127.0.0.1 端口7890
sudo curl -x http://127.0.0.1:7890 -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
解释：-sSL是什么
-sSL 是 curl 的三个选项组合，分别代表：
    1. -s（--silent）：静默模式，不显示进度条或错误信息（除非有严重错误）。
    2. -S（--show-error）：与 -s 结合使用时，仍然显示错误信息（避免安静模式下错误被忽略）。
    3. -L（--location）：如果服务器返回 HTTP 3xx 重定向，curl 会自动跟随新的 URL。
组合使用的作用
curl -sSL <URL>
    • 静默模式 (-s) 让输出更干净，没有进度条。
    • 如果发生错误 (-S)，仍然会显示错误消息，方便排查问题。
    • 自动跟随重定向 (-L)，确保 curl 能成功下载目标文件，即使 URL 发生跳转。
这在下载 GitHub Raw 文件等资源时特别有用，因为 GitHub 可能会有 URL 重定向。
+ 双击deb安装程序可能会自动退出，
   解决：用命令行
   sudo dpkg -i filename.deb


**以下为安装过程中学到的东西：**
注意：每次打开shell都要激活ros2环境
source /opt/ros/galactic/setup.bash

**这几行代码是干什么的**
Now add the ROS 2 GPG key with apt.
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
Then add the repository to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
这几行代码的作用是**在 Ubuntu 上设置 ROS 2（机器人操作系统 2）的软件包源**，以便通过 `apt` 安装 ROS 2。让我们逐行解析它们的作用。

---

### **1. 更新软件包并安装 `curl`**
```bash
sudo apt update && sudo apt install curl
```
- `sudo apt update`：更新本地的 APT 软件包索引，确保获取最新的软件包信息。  
- `sudo apt install curl`：安装 `curl`，它是一个命令行工具，用于下载文件或与 HTTP 服务器交互（如果 `curl` 已经安装，可以跳过这步）。  

---

### **2. 添加 ROS 2 的 GPG 密钥**
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
- 这一步的目的是**下载并添加 ROS 2 的 GPG 公钥**，以便 Ubuntu 可以验证 ROS 2 的软件包来源是否可信。  
- `curl -sSL`：静默下载 GitHub 上 ROS 2 的公钥文件 `ros.key`。  
- `-o /usr/share/keyrings/ros-archive-keyring.gpg`：将公钥文件保存到 `/usr/share/keyrings/` 目录，命名为 `ros-archive-keyring.gpg`。  
- 这样，APT 在安装 ROS 2 软件包时可以使用这个密钥来验证软件包的完整性和真实性。  

---

### **3. 添加 ROS 2 软件源**
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
这条命令的作用是**向 Ubuntu 的 APT 源列表中添加 ROS 2 软件包仓库**，从而能够通过 `apt install` 直接安装 ROS 2。

拆解命令：
- `dpkg --print-architecture`：获取当前系统的 CPU 架构（如 `amd64`、`arm64`）。  
- `$(. /etc/os-release && echo $UBUNTU_CODENAME)`：
  - `/etc/os-release` 文件包含 Ubuntu 版本信息。
  - 这个命令会输出当前 Ubuntu 版本的代号（如 `focal` 对应 20.04，`jammy` 对应 22.04）。
- `echo "deb [...]"`：
  - 生成一条 APT 源的配置，指定：
    - `deb`：这个源提供二进制包（`.deb`）。
    - `[arch=...]`：指定 CPU 架构（如 `amd64`、`arm64`）。
    - `signed-by=...`：指定 GPG 公钥文件路径，以确保软件包来源可信。
    - `http://packages.ros.org/ros2/ubuntu`：ROS 2 的官方软件包源地址。
    - `$(. /etc/os-release && echo $UBUNTU_CODENAME)`：Ubuntu 代号（如 `focal`、`jammy`）。
    - `main`：表示这个源属于主软件仓库。
- `sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`：
  - `tee` 命令将 `echo` 生成的内容写入 `/etc/apt/sources.list.d/ros2.list`，从而创建一个新的 APT 源文件。
  - `> /dev/null`：防止 `tee` 命令的输出干扰终端。

---

### **总结**
这几行代码的目的是：
1. **确保系统可以下载 ROS 2 软件包**：
   - 更新 APT 并安装 `curl`（如果尚未安装）。
   - 添加 ROS 2 的 GPG 公钥，确保安装的软件包是官方认证的。
   - 添加 ROS 2 的 APT 源，使系统可以找到 ROS 2 相关的软件包。

**下一步**
执行完这些命令后，一般需要运行：
```bash
sudo apt update
```
以刷新软件包索引，然后就可以安装 ROS 2 了：
```bash
sudo apt install ros-humble-desktop  # 以 ROS 2 Humble 为例
```

这样，你的 Ubuntu 机器就可以成功安装 ROS 2 并开始开发机器人应用了！ 🚀


## source /opt/ros/galactic/setup.bash这段代码是什么意思
这段代码的作用是**加载 ROS 2 Galactic 版本的环境变量**，使当前终端能够正确使用 ROS 2 提供的命令和工具。

---

### **逐步解析**
```bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
```
**这部分是注释**，意思是：
- `setup.bash` 适用于 **Bash**（默认 shell）
- `setup.zsh` 适用于 **Zsh**
- `setup.sh` 适用于 **其他 shell**

不同的 shell 需要 `source` 对应的文件，以正确加载 ROS 2 的环境。

---

```bash
source /opt/ros/galactic/setup.bash
```
**具体作用：**
1. **激活 ROS 2 Galactic 版本的环境**  
   `source` 命令用于在当前 shell 中执行 `setup.bash`，而不创建新的子进程。
2. **设置环境变量**（如 `PATH`, `LD_LIBRARY_PATH`, `AMENT_PREFIX_PATH` 等）
   - `PATH`：让终端能找到 ROS 2 的命令，如 `ros2`。
   - `LD_LIBRARY_PATH`：让系统知道 ROS 2 的共享库位置。
   - `AMENT_PREFIX_PATH`：让 ROS 2 的工具（如 `colcon`）找到安装的 ROS 2 包。
3. **确保当前终端能够正确运行 ROS 2**  
   如果不执行 `source`，你可能会遇到 `command not found: ros2` 之类的错误。

---

### **如何让每次打开终端都自动加载 ROS 2？**
如果你希望每次打开终端都自动加载 ROS 2，可以把这行代码加到 `~/.bashrc`（如果你用的是 Bash）：
```bash
echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
如果你用的是 Zsh，把它加到 `~/.zshrc`：
```bash
echo "source /opt/ros/galactic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

这样，每次打开终端时都会自动加载 ROS 2 环境，你就不需要手动 `source` 了。

---

### **适用于其他 ROS 2 版本**
如果你安装的不是 `Galactic`，比如 `Humble`，你需要改成：
```bash
source /opt/ros/humble/setup.bash
```
总之，这行 `source` 代码的作用就是**让 ROS 2 的命令和环境变量在当前终端中生效**。🚀
