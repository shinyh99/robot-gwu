
# Costmap
## ## common
### general
robot_radius: radius required for robot to turn
### inflation layer
The advantage is that the robot would prefer to move in the middle of obstacles.

Gray: safe area(free space), robot movement will not collide in this area

Blue: the risk area where robots may clooide

Cyon: dangerous area, where if the center of robot meets cyon, it collides

Purple: wall


### Tips
so basically, if the laser reading is equal to max_range, it takes no effect
Thus, we need to set max_range reading to inf, and allow obstacle_layer to clear inf by setting inf_is_valid to true
So, we can change the max_range reading to inf using laserfilters