��ֲ���ȣ�
20160505ǰ
Config_RetrieveSettings();	���
plan_init();				���
tp_init();					ϸ��δ���

��һ����ֲ����
get_command();
20160505
get_command()���е�MAX_CMD_SIZE-1 					( OK at 20160515)

��һ����ֲ����
get_command()���е� end while(MYserial.read()...) 	( ok at 20160516)

��һ����ֲ����
process_commands()���е�case 29:  					( ok at 20160518)

��һ����ֲ����
prepare_move()�����ֲ								( ok at 20160518)								

��һ����ֲ����
plan_buffer_line()�����ֲ							( ok at 20160925)

��һ����ֲ����
planner_reverse_pass()�����ֲ						( ok at 20150926)							

��һ����ֲ����
process_commands()�����ֲ							( ok at 20160929)

��һ����ֲ����
manage_heater()�����ֲ								( ok at 20161005)

��һ����ֲ����
manage_inactivity()�����ֲ							( ok at 20161005)

��һ����ֲ����
checkHitEndstops()�����ֲ							( ok at 20161005)

��һ����ֲ��ɣ�
ϸ�ڼ�飺
1 tp_init()��ֲ������
  tp_init()���ڻ������Ŷ�����¶ȵľ����ȡû�����
2 st_init()��ֲ������
  st_init()���ڻ������Ŷ���û�����
3 get_command()�д�ӡ��Ϣ���ֲ�����