### 2025HUST-HERO-AIM

### 命名规范
命名空间：小写式
类名：全大写式
类内变量名&函数名：驼峰式
临时变量名：小写下划线式

### 功能开发

- 自瞄

- 反前哨

- 基地辅助吊射（TODO）

### 运行

```
mkdir logs
mkdir build && cd build
make -j8
./AutoAim
```

ps：本项目设置虚拟串口，将device_type设为1后读取本地视频可运行
