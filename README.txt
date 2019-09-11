2019.9.11 edit
#程式啟動流程:
    1.$cd robotrace
    (下面每個指令前先ctrl+shift+T另開分頁,並輸入$. devel/setup.bash)
    2.$roscore
    3.roslaunch turtlebot3_bringup robotrace.launch
    4.roslaunch usb_cam usb_cam-test.launch
    5.roslaunch rosbridge_server rosbridge_websocket.launch
    6.rosrun strategy strategy2019.py
    7.開啟tb3_web,strategy2019輸出ready後,網頁點選start開始程式
    8.策略完成後按stop停止程式,放回原點後按reset將定位值初始化 

=================================================

#待更新及製作項目:
    1.避障系統優化
    2.指定位置搜尋
