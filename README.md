## 本项目用于betaflight的autopilot

#### 调试
- 如果使用编译好的二进制文件调试，操作与调试正常c++代码一样
- 如果使用launch文件调试，务必需要保证全局状态下（开启一个新的终端后输入`echo $ROS_PACKAGE_PATH`）包含本项目的环境变量，如果在bashrc中无法添加，可以新开终端先source一下然后在这个终端中打开vscode再进行调试