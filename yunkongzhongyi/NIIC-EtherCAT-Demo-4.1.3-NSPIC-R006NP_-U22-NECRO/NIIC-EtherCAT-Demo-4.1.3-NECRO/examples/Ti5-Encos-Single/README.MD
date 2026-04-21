## 使用命令行

### 测试命令
|DEMO|指令|解释|
|----|---|----|
|ethercat-cxx-bytedance|-o (8, 9, 10, 6)|控制模式，依次是CSP,CSV,CST,HM,默认8
||-b (17, 23, ...)	|电机分辨率，依据伺服手册，默认17位
||-d (0, 1)	|转动方向，1为正转，0为反转，默认1
||-c (1000000, ...)	|EC通讯周期，单位纳秒，默认1000000(1毫秒)


### 示例
``````
./ethercat-cxx-bytedance
``````
or
``````
./ethercat-cxx-bytedance -c 2000000 -b 17 -d 0 -o 8
``````


## 使用源码

### IO控制
实例化不同的子类，通过此方法可自定义不同IO设备的控制方法
``````
if (task.axis_vid(slave_pos) == 0x80ABC && task.axis_pid(slave_pos) == 0x80ABC)
    io_ins = std::make_unique<IO_XinJinCheng>();
else
    io_ins = std::make_unique<IO_example>();
``````
