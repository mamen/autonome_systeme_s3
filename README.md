# Documentation for Turtlebot3 Burger model

D$ ... desktop terminal prompt
R$ ... raspberry terminal prompt via ssh

## Architecture

Network: 192.168.0.xxx/24
Gateway: 192.168.0.1
Master-IP (desktop): 192.168.0.58
Turtlebot-IP (raspberry): 192.168.0.59

## Demo
this demo relates to http://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/

1. start Ros Core (either via terminal or IDE):
```
D$ roscore
```

2. start Turtlebot Node on TB
```
R$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

3. start "Remote Control", namely teleop on desktop
```
D$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

4. control robot with:
- w (forward)
- a (left)
- d (right)
- x (reverse)
- s (stop)
