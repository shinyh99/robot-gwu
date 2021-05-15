check raytrace distance

check amcl
rosserivce call update_localization

build waypoint

rosservice call clear_costmaps


# Things to consider or IDEA
 - upon path abort, instead of using auto-recovery, lets use my own recovery, that allows scout to move away from the closest laser point(which covers all surrounding, 360 degree)
    - So it requires 360 surrounding scan data
 - 