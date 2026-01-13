[TOC]

# git , http , https

在 Git 中设置代理可以通过配置 `http.proxy` 和 `https.proxy` 来实现。以下是具体步骤：

### 1. 设置 HTTP 代理
```bash
git config --global http.proxy http://proxy.example.com:8080
git config --global http.proxy http://127.0.0.1:7890
```

### 2. 设置 HTTPS 代理
```bash
git config --global https.proxy https://proxy.example.com:8080
git config --global https.proxy http://127.0.0.1:7890
```

### 3. 设置代理的用户名和密码（如果需要）
如果代理服务器需要认证，可以在 URL 中包含用户名和密码：
```bash
git config --global http.proxy http://username:password@proxy.example.com:8080
git config --global https.proxy https://username:password@proxy.example.com:8080
```

### 4. 取消代理设置
如果需要取消代理设置，可以使用以下命令：
```bash
git config --global --unset http.proxy
git config --global --unset https.proxy
```

### 5. 查看当前代理配置
```bash
git config --global --get http.proxy
git config --global --get https.proxy
```

### 6. 针对特定域名设置代理
如果只想对特定域名使用代理，可以这样配置：
```bash
git config --global http.https://example.com.proxy http://proxy.example.com:8080
```

### 7. 使用环境变量设置代理
你也可以通过环境变量设置代理：
```bash
export http_proxy=http://proxy.example.com:8080
export https_proxy=https://proxy.example.com:8080
```

### 注意事项
- 确保代理服务器的地址和端口正确。
- 如果代理需要认证，确保用户名和密码无误。
- 取消代理设置后，Git 将不再使用代理。
- **通过 Git 命令行参数临时设置代理
  Git 本身不支持直接通过命令行参数设置代理，但可以通过环境变量实现**

通过这些步骤，你可以轻松为 Git 配置代理。

# curl

`curl` 走代理的方法和 `pip` 类似，它支持 **HTTP/HTTPS/SOCKS** 多种代理协议。常见写法如下：

---

## 🔹 临时指定代理

直接在命令后面加 `-x` / `--proxy`：

```bash
curl -x http://127.0.0.1:7890 https://www.google.com
```

如果是 **SOCKS5 代理**：

```bash
curl -x socks5://127.0.0.1:1080 https://www.google.com
```

带用户名密码：

```bash
curl -x http://user:password@127.0.0.1:7890 https://example.com
```

---

## 🔹 指定不同协议的代理

* HTTP 代理：

  ```bash
  curl --proxy http://127.0.0.1:8080 https://example.com
  ```
* HTTPS 代理：

  ```bash
  curl --proxy https://127.0.0.1:8080 https://example.com
  ```
* SOCKS4：

  ```bash
  curl --proxy socks4://127.0.0.1:1080 https://example.com
  ```
* SOCKS5：

  ```bash
  curl --proxy socks5://127.0.0.1:1080 https://example.com
  ```
* SOCKS5 + DNS：

  ```bash
  curl --proxy socks5h://127.0.0.1:1080 https://example.com
  ```

  （区别在于 DNS 解析是否也走代理，推荐 `socks5h`）

---

## 🔹 环境变量方式（全局生效）

在 **Linux / macOS**：

```bash
export http_proxy="http://127.0.0.1:7890"
export https_proxy="http://127.0.0.1:7890"
```

在 **Windows (PowerShell)**：

```powershell
setx http_proxy "http://127.0.0.1:7890"
setx https_proxy "http://127.0.0.1:7890"
```

然后直接用：

```bash
curl https://example.com
```

---

## 🔹 配置文件方式（长期使用）

curl 支持配置文件 `~/.curlrc`（Linux/macOS）或 `_curlrc`（Windows 用户目录下）。
写入：

```bash
proxy = http://127.0.0.1:7890
```

---

⚠️ 提示：

* 如果你用的是科学上网工具（Clash、V2Ray、SS/SSR 等），本地默认端口常见是 `7890`（HTTP/HTTPS）或 `1080`（SOCKS）。
* `socks5h` 比 `socks5` 更好，因为连 DNS 查询也走代理。

---

要不要我给你写一个 **一键切换 curl 代理 / 直连的小脚本**？这样你就能快速在“走代理”和“不走代理”之间切换。

# pip

好的 👍 你问的是 **pip 设置代理**。pip 本身就是基于 `requests` 的，所以可以通过命令行参数、环境变量或配置文件来指定代理。下面给你总结几种常用方法：

---

## 1. 临时设置代理（推荐测试）

如果你有一个 HTTP 或 SOCKS5 代理，比如 `http://127.0.0.1:7890`，可以这样：

```bash
pip install 包名 --proxy http://127.0.0.1:7890
```

或者 socks5：

```bash
pip install 包名 --proxy socks5://127.0.0.1:1080
```

---

## 2. 使用环境变量（推荐长期使用）

Linux / macOS:

```bash
export HTTP_PROXY="http://127.0.0.1:7890"
export HTTPS_PROXY="http://127.0.0.1:7890"
```

Windows CMD:

```cmd
set HTTP_PROXY=http://127.0.0.1:7890
set HTTPS_PROXY=http://127.0.0.1:7890
```

Windows PowerShell:

```powershell
$env:HTTP_PROXY="http://127.0.0.1:7890"
$env:HTTPS_PROXY="http://127.0.0.1:7890"
```

---

## 3. 写入 pip 配置文件（永久生效）

可以在 pip 的配置文件里写上代理。

* **Windows 路径**
  `%APPDATA%\pip\pip.ini`
* **Linux/macOS 路径**
  `~/.pip/pip.conf`

添加内容：

```ini
[global]
proxy = http://127.0.0.1:7890
```

---

## 4. 如果用 conda 环境

有时 conda 和 pip 结合用，可以在激活环境时设置环境变量，也可以在 `.condarc` 中设置代理。

---

⚠️ 注意事项：

* 代理地址要和你实际的代理软件一致（比如 Clash、V2Ray、SSR 等）。
* 如果代理需要用户名和密码，可以写成：

  ```
  http://user:password@proxyserver:port
  ```

---

要不要我给你写一个 **Windows 下 pip + Clash/V2Ray 常用代理配置的完整示例**？这样你直接复制粘贴就能用。
