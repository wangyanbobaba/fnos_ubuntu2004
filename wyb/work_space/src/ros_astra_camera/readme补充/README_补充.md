
1. 该ros包来源：wheeltec830提供的资料包中
2. 使用：先阅读README
3. 本文件记录使用过程中的问题
    1. 安装依赖时报错：
        ```
        taihe@thpc:~/temp/work_space$ sudo apt install ros-noetic-rgbd-launch
        [sudo] password for taihe: 
        Reading package lists... Done
        Building dependency tree       
        Reading state information... Done
        E: Unable to locate package ros-noetic-rgbd-launch
        ```
        解决：
        添加清华镜像源,进入https://mirror.tuna.tsinghua.edu.cn/help/ros/ 按照提示更新索引即可  
        若/etc/apt/sources.list.d/ros-latest.list已经存在，
        注释掉原来的修改为清华镜像中的即可。
        ![alt text](image.png)
        ![alt text](image-1.png)
    2. 安装依赖报错如下图  
        ![alt text](image-2.png)

        解决：  
        Ubuntu 20.04 / ROS Noetic 的 官方 apt 仓库里根本没有 ros-noetic-libuvc 这个包——它从未被 Release 过。（很多教程把 4 个包写在一起，但第 2 个 libuvc 是多余的。）