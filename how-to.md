# How To

## 1. 容器中使用声卡

Carla官方发布的容器在启动时可能会报声音系统问题。如
```
TODO
```

**解决方法：**
1. 首先确认宿主机声卡可用,这里我们使用card 1， ALC897
```
zyb@zyb-CORSAIR-VENGEANCE-i8100:/data/Carla$ aplay -l
**** List of PLAYBACK Hardware Devices ****
card 0: NVidia [HDA NVidia], device 3: HDMI 0 [HDMI 0]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 0: NVidia [HDA NVidia], device 7: HDMI 1 [HDMI 1]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 0: NVidia [HDA NVidia], device 8: HDMI 2 [HDMI 2]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 0: NVidia [HDA NVidia], device 9: HDMI 3 [HDMI 3]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 1: PCH [HDA Intel PCH], device 0: ALC897 Analog [ALC897 Analog]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 1: PCH [HDA Intel PCH], device 1: ALC897 Digital [ALC897 Digital]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 1: PCH [HDA Intel PCH], device 3: HDMI 0 [HDMI 0]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 1: PCH [HDA Intel PCH], device 7: HDMI 1 [HDMI 1]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 1: PCH [HDA Intel PCH], device 8: HDMI 2 [HDMI 2]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 1: PCH [HDA Intel PCH], device 9: HDMI 3 [HDMI 3]
  Subdevices: 1/1
  Subdevice #0: subdevice #0

```

确认声卡可以播放, 如果工作正常，你会听到"Front Center“两个单词。
```
aplay -D plug:default /usr/share/sounds/alsa/Front_Center.wav

```

2. 加载声卡所需的资源，启动容器，这里添加了`--device /dev/snd:/dev/snd`和`-v /etc/asound.conf:/etc/asound.conf:ro`, 并且，不启动carla server程序
```
sudo docker run --rm -it \
     --privileged --gpus all --net=host  \
     -e DISPLAY=$DISPLAY \
     --device /dev/snd:/dev/snd  \
     --group-add audio \
     -v /etc/asound.conf:/etc/asound.conf:ro \
     --name carla_server \
      carlasim/carla:0.9.15 /bin/bash 
```

`/etc/asound.conf`内容如下：
```
pcm.!default {
    type hw
    card 1
}
ctl.!default {
    type hw
    card 1
}
```

3. 测试容器中是否可以播放声音，用root用户打开一个容器内的交互命令行，安装alas工具套件
```
sudo docker exec -it -u root carla_server bash

apt update
apt install -y alsa-utils

#测试声音播放
aplay -D plug:default /usr/share/sounds/alsa/Front_Center.wav

# 退出
exit

# 用默认的carla用户名重新登陆

sudo docker exec -it carla_server bash
#测试声音播放
aplay -D plug:default /usr/share/sounds/alsa/Front_Center.wav
```

4. 重新打包容器镜像(optional)
```
sudo docker commit carla_server carla:0.9.15_snd
```

5. 重新启动容器
```
sudo docker run --rm -it \
     --privileged --gpus all --net=host  \
     -e DISPLAY=$DISPLAY \
     --device /dev/snd:/dev/snd  \
     --group-add audio \
     -v /etc/asound.conf:/etc/asound.conf:ro \
     --name carla_server \
      carla:0.9.15_snd /bin/bash ./CarlaUE4.sh -windowed -ResX=1024 -ResY=786 -carla-rpc-port=2000 -quality-level=High
```

==Note: Now, we don't get any error message about the sound system, but we still hear nothing from the simulator.==

## 2. 解决`xdg-user-dir: not found`问题

```
sudo docker exec -it -u root carla_server bash

apt update

apt install -y xdg-user-dirs


apt clean && \
rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

exit

sudo docker commit carla_server mycarla:0.9.15_snd_xdg
```

重新启动容器(optional)
```
sudo docker run --rm -it \
     --privileged --gpus all --net=host  \
     -e DISPLAY=$DISPLAY \
     --device /dev/snd:/dev/snd  \
     --group-add audio \
     -v /etc/asound.conf:/etc/asound.conf:ro \
     --name carla_server \
      mycarla:0.9.15_snd_xdg /bin/bash ./CarlaUE4.sh -windowed -ResX=1024 -ResY=786 -carla-rpc-port=2000 -quality-level=High
```
