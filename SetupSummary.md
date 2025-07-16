Used [Oversteer](https://github.com/berarma/oversteer) to implement Force Feedback (FFB) with 720 rotation range, 100 global feedback gain and 20 autocenter strength

## Oversteer installation process
1. Install Dependencies:

``sudo apt git install python3 python3-distutils python3-gi python3-gi-cairo python3-pyudev python3-xdg python3-evdev gettext meson appstream-util desktop-file-utils python3-matplotlib python3-scipy``

2. Clone Oversteer into working directory:

````
git clone https://github.com/berarma/oversteer.git
cd oversteer
````

3. Prepare build system:

```
meson setup build
cd build
```

4. Installing (needs administration rights):

```ninja install```

