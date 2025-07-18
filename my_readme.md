# My Readme


启动命令
```
sudo docker run --rm -it \
     --privileged --gpus all --net=host  \
     -e DISPLAY=$DISPLAY \
     --device /dev/snd:/dev/snd  \
     --group-add audio \
     -v /etc/asound.conf:/etc/asound.conf:ro \
     --name carla_server \
      mycarla:0.9.15_snd_xdg /bin/bash ./CarlaUE4.sh -windowed -ResX=1024 -ResY=786 -carla-rpc-port=2000 -quality-level=Epic
```


```
sudo docker run --rm -it \
     --privileged --gpus all --net=host  \
     -e DISPLAY=$DISPLAY \
     --device /dev/snd:/dev/snd  \
     --group-add audio \
     -v /etc/asound.conf:/etc/asound.conf:ro \
     --name carla_server \
      mycarla:0.9.15_maps /bin/bash ./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000 -quality-level=Epic
```


```
VENV_PATH=.venv
python3.10 -m venv $VENV_PATH
$VENV_PATH/bin/pip install -U pip setuptools
$VENV_PATH/bin/pip install poetry
source $VENV_PATH/bin/activate
```