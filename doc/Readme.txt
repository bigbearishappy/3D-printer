移植进度：
20160505前
Config_RetrieveSettings();	完成
plan_init();				完成
tp_init();					细节未完成

下一步移植任务：
get_command();
20160505
get_command()进行到MAX_CMD_SIZE-1 					( OK at 20160515)

下一步移植任务：
get_command()进行到 end while(MYserial.read()...) 	( ok at 20160516)

下一步移植任务：
process_commands()进行到case 29:  					( ok at 20160518)

下一步移植任务：
prepare_move()完成移植								( ok at 20160518)								

下一步移植任务：
plan_buffer_line()完成移植							( ok at 20160925)

下一步移植任务：
planner_reverse_pass()完成移植						( ok at 20150926)							

下一步移植任务：
process_commands()完成移植							( ok at 20160929)

下一步移植任务：
manage_heater()完成移植								( ok at 20161005)

下一步移植任务：
manage_inactivity()完成移植							( ok at 20161005)

下一步移植任务：
checkHitEndstops()完成移植							( ok at 20161005)

第一遍移植完成：
细节检查：
1 tp_init()移植不完整
  tp_init()现在还有引脚定义和温度的具体获取没有完成
2 st_init()移植不完整
  st_init()现在还有引脚定义没有完成
3 get_command()中打印信息部分不完整 代码第一遍检查完成
4 process_command()中打印信息部分不完整 代码第一遍检查完成

通过ADC获取电压计算出实时温度						( ok at 20170411)

下一步移植任务：
与上位机联调，在窗口中显示实时温度(目前温度值不准)	( ok at 20170412)

下一步移植任务：
PID_autotune()完成移植								( ok at 20170413)

下一步移植任务：
加热头控制的pwm波生成


PS: //// 表示后面的代码已经在下面被替换，这里留着作为记录
    //   表示这个代码目前还没有被替换，以后的工作需要来替换他