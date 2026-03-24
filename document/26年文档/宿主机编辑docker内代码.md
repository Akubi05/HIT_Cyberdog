vscode下载docker,dev container

```
luke@akubi:~$ docker --version
Docker version 29.3.0, build 5927d80
luke@akubi:~$ docker ps
permission denied while trying to connect to the docker API at unix:///var/run/docker.sock
luke@akubi:~$ sudo usermod -aG docker $USER
```

说明 Docker 已经装好了，但当前用户 luke 没有权限访问 Docker socke
先把当前用户加入 docker 用户组：
```
sudo usermod -aG docker $USER
```
重启电脑
再次`docker ps`


![alt text](image.png)

点attach vscode
然后就可以修改代码了