RH56 灵巧手 OS 驱动框架设计文档

1. 概述（Overview）

RH56 仿人灵巧手通过 RS485 接口与外部主控通信，通信协议基于帧格式的寄存器读写机制。为了将灵巧手集成到自研操作系统（以下简称 OS）中，需要将官方协议封装为 OS 内部的“设备驱动（Device Driver）”，并向上层提供统一、清晰、稳定的控制接口。

本框架旨在完成以下目标：

屏蔽底层 UART/RS485 细节

提供稳定的寄存器读写接口

提供面向控制层/算法层使用的高级 API（set_angle、set_force…）

支持灵巧手动作、状态、错误码的完整读写

保证未来可移植至不同硬件平台

本文将介绍驱动的架构、分层方式、核心 API、协议解析、与 OS 的对接方法。

2. 驱动整体架构（Architecture）

驱动分成四层：

+------------------------------------------------------+
| 4. 控制层 / 应用层（算法、策略、任务、机器人集成）  |
|   - hand_demo_task、RL policy、任务轨迹控制          |
+------------------------------------------------------+
| 3. RH56 驱动 API 层（对上层友好）                    |
|   - rh56_set_angles() / rh56_get_angles()            |
|   - rh56_set_speeds() / rh56_set_forces()            |
|   - rh56_clear_error()                               |
+------------------------------------------------------+
| 2. 协议驱动层：帧构造与解析                          |
|   - rh56_read_reg() / rh56_write_reg()               |
|   - checksum 计算，读写超时管理                       |
+------------------------------------------------------+
| 1. OS HAL 硬件抽象层（由 OS 提供）                   |
|   - uart_write() / uart_read() / sleep_ms()          |
+------------------------------------------------------+
| 0. 硬件（RH56 灵巧手 + RS485 + MCU 主控）             |
+------------------------------------------------------+


这种结构使得：

上层完全不需要了解通信协议

协议变化时只需修改 2 层

更换 MCU / OS 时只需修改 1 层

逻辑清晰、可维护、可扩展

3. 通信协议简述（Protocol Summary）

RH56 的通信是典型的“寄存器读写 + 帧结构”体系：

3.1 帧头定义
类型	帧头
上位机 → 灵巧手	0xEB 0x90
灵巧手 → 上位机	0x90 0xEB
3.2 命令字
命令	含义
0x11	读寄存器
0x12	写寄存器
3.3 校验方式

checksum = 所有 data 字节累加和的低 8 位

3.4 寄存器模型

灵巧手通过寄存器暴露其全部功能，包括：

分类	作用
配置类	ID、保存参数、清错
控制类	ANGLE_SET、POS_SET、FORCE_SET、SPEED_SET
状态类	ANGLE_ACT、FORCE_ACT、TEMP、STATUS、ERROR
动作序列	SEQ_IDX、SEQ_RUN（用于预录动作）
4. OS 驱动层设计（Driver Design）

驱动文件包括：

rh56_driver.h：对外 API、数据结构、枚举常量

rh56_driver.c：协议实现、帧解析、寄存器接口

4.1 初始化流程
rh56_config_t cfg = {
    .hand_id    = 1,
    .uart_write = my_uart_write,
    .uart_read  = my_uart_read,
    .sleep_ms   = my_sleep_ms,
};

rh56_init(&cfg);


初始化时驱动会：

保存串口函数指针

执行一次 rh56_clear_error()

后续所有操作必需基于此配置进行

5. 协议层：寄存器读写实现

驱动核心由两个内部函数完成：

5.1 读寄存器：rh56_read_reg()

流程：

构造帧：[EB 90] + ID + len + cmd=0x11 + addr + data_len

写串口

等待响应帧：[90 EB] + ID + frame_len + data

校验 ID、命令字、长度字段

解析数据区返回给调用方

5.2 写寄存器：rh56_write_reg()

流程：

构造写命令帧

写串口

读响应帧（仅表示是否写成功）

校验帧头和命令字

返回成功或错误

6. 高层 API：控制接口（对应用层）

对外提供的 API 全都基于读写寄存器实现：

6.1 控制类
rh56_set_angles(int16_t angles[6]);   // 主用接口
rh56_set_speeds(uint16_t speeds[6]);
rh56_set_forces(uint16_t forces[6]);
rh56_clear_error();


说明：

角度范围 0–1000

0 = 握拳，1000 = 张开（经实机测试确定）

力控单位约为 g，可作为安全机制

速度用于调节动作快慢

6.2 观测类
rh56_get_angles(int16_t angles[6]);
rh56_get_forces(uint16_t forces[6]);
rh56_get_temps(uint8_t temps[6]);
rh56_get_status(uint8_t status[6]);
rh56_get_errors(uint8_t errors[6]);


说明：

STATUS/ERROR 用于故障处理、运动状态判断

TEMP 用于监控电机发热

6.3 动作序列（可选）
rh56_run_action_sequence(index);


用于运行手内部预录动作库。

7. Demo：OS 上的测试任务
void hand_demo_task(void)
{
    int16_t open[6]  = {1000,1000,1000,1000,1000,1000};
    int16_t close[6] = {0,0,0,0,0,0};

    rh56_clear_error();

    rh56_set_speeds((uint16_t[6]){800,800,800,800,800,800});
    rh56_set_forces((uint16_t[6]){500,500,500,500,500,500});

    rh56_set_angles(open);
    my_sleep_ms(1000);

    rh56_set_angles(close);
    my_sleep_ms(1000);
}


应用层代码不再涉及 UART、帧协议、校验等，只使用高级 API。

8. 可移植性与扩展性

本驱动框架具备以下特性：

与硬件平台无关
所有串口读写由 HAL 提供，驱动不关心 MCU 型号

与 OS 无关
sleep、任务调度、串口均由上层提供

可扩展性强
后续要添加 move-by-trajectory、闭环力控，都能基于该框架扩展

调试方便
如果有问题，可以直接抓 UART 帧分析

9. 小结

本设计将灵巧手的复杂协议封装成 OS 驱动模块，上层代码无需处理 RS485 的底层细节，只需调用简单的函数即可实现控制。

下一步工作包括：

串口适配

驱动测试

故障机制完善

与机械臂集成

与 RL/规划算法对接

详见后续的 TODO.md。