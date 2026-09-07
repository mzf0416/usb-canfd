# USB-CANFD

基于 `motorevo` 同一块 STM32G474VET6 板，只保留 **USB FS ↔ FDCAN1** 转发。

| 外设 | 引脚 | 用途 |
|---|---|---|
| USB FS | PA11 DM / PA12 DP | CDC ACM 虚拟串口 |
| FDCAN1 | PD0 RX / PD1 TX | CAN-FD，默认仲裁 1 Mbps / 数据 5 Mbps |
| USART1 | PA9 TX / PA10 RX | 115200 调试日志 |

电机、蓝牙、4G、FDCAN2、Flash 均未启用。

## 构建 / 烧录

在已配置的 Zephyr west 工作区中：

```powershell
west build -b st_g474_board/stm32g474xx -d build-g474 e:\Work\D157Bdemo_HC-05\8.28wang\usb-canfd
west flash -d build-g474
```

插上板载 USB 后，主机应出现 CDC 串口（产品名 `G474 USB CANFD`）。

## 行协议

一行一帧，`\n` 结尾，十六进制 ID，十进制长度。

```
t <id> <flags> <nbytes> <hex...>
r <id> <flags> <nbytes> <hex...>
```

`flags`：`x` 扩展帧，`f` CAN-FD，`b` BRS，`r` 远程帧；没有标志写 `-`。

```
t 123 fb 8 11 22 33 44 55 66 77 88
t 1ABCDEF xfb 16 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F
t 100 - 2 AABB
```

其它命令：`O` 打开转发，`C` 暂停转发，`S 1000000 5000000` 改波特率，`s` 查总线状态。

总线无第二节点时发送会因缺少 ACK 失败，属正常现象。
