﻿1. 定时器
（1）开机后启动200ms定时器，提示“GIS ININ...”，主界面加载时则关闭此定时器。
（2）开机后启动500ms定时器timer_WatchDog，向上位机发送服务器时间，超时未发则被重启。
（3）显示更新100ms定时器TimerDisplay100ms

2.文件
（1）auto_initial.txt存储工作空间的文件名（含路径），任务文件名