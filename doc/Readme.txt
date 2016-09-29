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