# 北邮机器人队
# 程序库使用说明

##### Author: Simon
##### Date: 2021/04/25

## 1. 引用方式
+ 新建一个IAR工程，将本文件夹整体拷贝在根目录下。
+ 打开IAR，选择`Project->Options`(或按下Alt+F7)，点击`C/C++ Compiler`，在`Preprocessor`选项卡的`Additional include directories`窗格中添加以下内容：

```C
$PROJ_DIR$/../BUPT_Libraries/SimpleLib
$PROJ_DIR$/../BUPT_Libraries/SimpleLib/can
$PROJ_DIR$/../BUPT_Libraries/SimpleLib/cmd
$PROJ_DIR$/../BUPT_Libraries/SimpleLib/core
$PROJ_DIR$/../BUPT_Libraries/SimpleLib/flash
$PROJ_DIR$/../BUPT_Libraries/SimpleLib/module
$PROJ_DIR$/../BUPT_Libraries/SimpleLib/nrf
$PROJ_DIR$/../BUPT_Libraries/SimpleLib/utils
$PROJ_DIR$/../BUPT_Libraries/SimpleLib/peripherals
$PROJ_DIR$/../BUPT_Libraries/ChassisLib
$PROJ_DIR$/../BUPT_Libraries/MotorLib
```
+ 在左侧Workspace的User内新建三个Group：`ChassisLib`,`MotorLib`,`SimpleLib`，并将每个库下的所有程序文件都添加到相应的Group内。例如，`SimpleLib`中包含`cmd`文件夹，则将`cmd`下的所有文件也添加到SimpleLib Group内。

