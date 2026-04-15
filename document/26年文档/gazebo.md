已经很明确了：**你的 Docker 不是 Docker Desktop 接管的那套 WSL 集成环境，而是你在 Ubuntu 里自己装的一套 Docker Engine**。从 `docker info` 可以直接看出来：

* `Operating System: Ubuntu 20.04.6 LTS`
* `Docker Root Dir: /var/lib/docker`
* `Runtimes: io.containerd.runc.v2 runc`
* **没有 `nvidia` runtime**。这意味着当前这个 Docker daemon 根本不会处理 `--gpus all`，所以才报 `could not select device driver "" with capabilities: [[gpu]]`。NVIDIA 官方文档说明，Docker 要跑 GPU 容器，前提是安装并配置 **NVIDIA Container Toolkit**，并把 Docker 配到可使用 NVIDIA runtime。([NVIDIA Docs][1])

所以你现在不是去改那条 `docker run` 命令，而是要先把 **Docker + NVIDIA runtime** 这层补齐。

---

## 你现在属于哪种情况

你大概率是下面这类：

### 情况 A：WSL 里自己装了 Docker Engine

这是最像你现在的情况。Docker 官方在 WSL 文档里明确建议：如果你打算用 Docker Desktop 的 WSL 2 backend，**先卸掉 WSL 发行版里直接安装的 Docker Engine/CLI**，因为两套并存会冲突。([Docker Documentation][2])

### 情况 B：你本来想用 Docker Desktop，但实际上终端连到的是 WSL 里自装的 daemon

这也很常见。结果就是：
你以为自己在用 “Windows + WSL + Docker Desktop + GPU”，但实际上跑命令的是 “Ubuntu 里裸装 Docker”，而这套又没配 NVIDIA toolkit，所以 `--gpus all` 直接失败。Docker Desktop 的 WSL 集成和 GPU 支持是另一条路线。([Docker Documentation][3])

---

# 最直接的判断结论

你的当前状态可以概括成一句话：

[
\text{当前 Docker daemon 只有 } runc,\ \text{没有 } nvidia\ runtime
]

因此：

[
--gpus\ all \quad \Rightarrow \quad \text{必然失败}
]

---

# 两条修复路线

你选一条，不要混着搞。

---

## 路线 1：继续用你现在这套 WSL 里的 Docker Engine

那就必须安装并配置 **NVIDIA Container Toolkit**。NVIDIA 官方给 Docker 的标准做法是：

1. 安装 `nvidia-container-toolkit`
2. 执行

   ```bash
   sudo nvidia-ctk runtime configure --runtime=docker
   ```
3. 重启 Docker daemon

   ```bash
   sudo systemctl restart docker
   ```

这是 NVIDIA 官方文档写明的 Docker 配置方式；`nvidia-ctk` 会修改 `/etc/docker/daemon.json`，让 Docker 能使用 NVIDIA Container Runtime。([NVIDIA Docs][4])

### 你先检查这几个命令

```bash
nvidia-smi
which nvidia-ctk
cat /etc/docker/daemon.json
docker info | grep -i runtime
```

### 预期结果

* `nvidia-smi` 能跑：说明 **WSL 本身能看到 GPU**
* `which nvidia-ctk` 有路径：说明 toolkit 已装
* `docker info` 里应该最终看到 `nvidia`
* `/etc/docker/daemon.json` 里会有 NVIDIA runtime 相关配置

### 如果没装 toolkit

按 NVIDIA 文档装。官方安装入口就是这个。([NVIDIA Docs][5])

装完后执行：

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker info | grep -i runtime
```

如果配置成功，`docker info` 不该只剩：

```bash
Runtimes: io.containerd.runc.v2 runc
```

而应该能看到 `nvidia`。NVIDIA 文档也把 “runtime not found” 的排查点列成：检查 `nvidia-ctk`、检查 `/etc/docker/daemon.json`、重启 Docker。([NVIDIA Docs][6])

### 然后跑最小测试

```bash
docker run --rm --gpus all nvidia/cuda:12.3.2-base-ubuntu22.04 nvidia-smi
```

如果这条能出显卡信息，说明 GPU 容器层已经通了；再回去跑你的 `cyberdog_sim:v2026`。Docker Desktop 的 GPU 文档也使用 `docker run --rm --gpus=all ...` 作为验证思路。([Docker Documentation][3])

---

## 路线 2：改成官方更推荐的 Windows + Docker Desktop + WSL 2 backend

如果你本来就在 WSL2 + Windows + NVIDIA 显卡环境里，**这条路线通常更省事**。

Docker 官方 WSL 文档写得很明确：

* 安装 Docker Desktop for Windows
* 打开 **Use WSL 2 based engine**
* 在 **Resources > WSL Integration** 里启用你的 Ubuntu 发行版
* 并且在开始前，**卸掉 WSL 里直接安装的 Docker Engine/CLI**，避免冲突。([Docker Documentation][2])

Docker 官方 GPU 文档则说明：Docker Desktop for Windows 支持在 WSL2 上通过 NVIDIA GPU-PV 让容器访问 GPU。([Docker Documentation][3])

### 这条路线的步骤

在 WSL 里先把本地 Docker 停掉并卸掉，然后改用 Docker Desktop 提供的 daemon。常见是：

```bash
sudo systemctl stop docker
sudo apt remove docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

然后在 Windows 上：

1. 安装 Docker Desktop
2. 打开 **Use WSL 2 based engine**
3. 开启你的 Ubuntu 的 **WSL Integration**
4. 确保 Windows 侧 NVIDIA 驱动支持 WSL GPU
5. `wsl --update`
6. 重新开 WSL

这些要求都在 Docker 的 WSL 与 GPU 文档里。([Docker Documentation][3])

### 这条路线的好处

你这条命令里有这些挂载：

```bash
-v /mnt/wslg:/mnt/wslg
-v /usr/lib/wsl:/usr/lib/wsl
--device=/dev/dxg
```

这本来就是很典型的 **WSL 图形/GPU 容器** 写法，所以改成 Docker Desktop 的 WSL backend，和你的目标场景更一致。Docker Desktop 的官方文档就是专门覆盖这类 WSL 2 使用方式的。([Docker Documentation][3])

---

# 我更建议你走哪条

### 如果你只是想尽快把 `cyberdog_sim` 跑起来

我更建议你走 **路线 2：Docker Desktop + WSL 2 backend**。

原因很简单：

* 你的命令明显是 **WSLg/WSL GPU 图形容器** 场景
* Docker 官方对这个路径支持更直接
* 还能避免你手动维护 WSL 里那套 Docker daemon 和 NVIDIA runtime 的组合问题。([Docker Documentation][3])

### 如果你明确想只用纯 Linux/WSL 内自装 Docker

那就走 **路线 1**，安装并配置 NVIDIA Container Toolkit。官方就是这么配的。([NVIDIA Docs][1])

---

# 你现在马上能做的最有效一步

先在 WSL 里执行：

```bash
nvidia-smi
which nvidia-ctk
cat /etc/docker/daemon.json
```

然后再看：

```bash
docker info | grep -i runtime
```

### 结果怎么判断

#### 1. `nvidia-smi` 都跑不通

那不是 Docker 的问题了，是 **WSL/Windows 驱动层** 还没好。WSL GPU 需要 Windows 侧驱动和 WSL 配合。([Docker Documentation][3])

#### 2. `nvidia-smi` 能跑，但 `nvidia-ctk` 没有

那就是 **NVIDIA Container Toolkit 没装**。装它。([NVIDIA Docs][5])

#### 3. `nvidia-ctk` 有，但 `docker info` 还是没有 `nvidia`

那就是 **还没执行 runtime configure**，或者配了但没重启 Docker。NVIDIA 官方要求运行 `sudo nvidia-ctk runtime configure --runtime=docker` 然后重启 daemon。([NVIDIA Docs][4])

---

# 你的原命令本身保留，但要等 GPU 层先修好

你最后真正要跑的命令仍然可以是：

```bash
docker run -it \
  --gpus all \
  --privileged \
  --shm-size=4g \
  --device=/dev/dxg \
  -e DISPLAY=$DISPLAY \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -e PULSE_SERVER=$PULSE_SERVER \
  -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /mnt/wslg:/mnt/wslg \
  -v /usr/lib/wsl:/usr/lib/wsl \
  -v $XDG_RUNTIME_DIR:$XDG_RUNTIME_DIR \
  cyberdog_sim:v2026
```

但前提是先满足：

[
\text{Docker daemon 能识别 NVIDIA runtime}
]

否则 `--gpus all` 一定卡死在启动前。这个结论和你当前 `docker info` 完全一致。([NVIDIA Docs][4])

---

如果你愿意，我下一条可以直接按你当前这台机器的情况，给你一份 **最短修复路径命令清单**，分成“保留当前 Docker”版和“切到 Docker Desktop”版。

[1]: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/1.17.4/install-guide.html?utm_source=chatgpt.com "Installing the NVIDIA Container Toolkit"
[2]: https://docs.docker.com/desktop/features/wsl/?utm_source=chatgpt.com "Docker Desktop WSL 2 backend on Windows"
[3]: https://docs.docker.com/desktop/features/gpu/?utm_source=chatgpt.com "GPU support - Docker Desktop"
[4]: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/1.14.2/install-guide.html?utm_source=chatgpt.com "Installing the NVIDIA Container Toolkit"
[5]: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html?utm_source=chatgpt.com "Installing the NVIDIA Container Toolkit"
[6]: https://docs.nvidia.com/dgx/dgx-spark/nvidia-container-runtime-for-docker.html?utm_source=chatgpt.com "NVIDIA Container Runtime for Docker — DGX Spark User ..."
