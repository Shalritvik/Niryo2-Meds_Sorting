#!/usr/bin/env python
from niryo_robot_python_ros_wrapper import *
import rospy

rospy.init_node('niryo_ned_example_python_ros_wrapper')

niryo_robot = NiryoRosWrapper()

conveyor_id = 9

workspace_name = "conveyorkejriwal"

niryo_robot.calibrate_auto()
niryo_robot.update_tool()

observation_pose = (0.158, -0.024, 0.149, -2.828, 1.328, -2.873)

while(True):
	niryo_robot.move_pose(*observation_pose)
	niryo_robot.control_conveyor(conveyor_id, bool_control_on=True, speed=50, direction=ConveyorDirection.FORWARD)

	# if niryo_robot.digital_read(sensor_pin_id) == PinState.HIGH :
	# 	niryo_robot.control_conveyor(conveyor_id, True, 0, ConveyorDirection.FORWARD)
		
        obj_found=0
	obj_found, object_pose,  shape, color = niryo_robot.get_target_pose_from_cam(workspace_name, height_offset=-0.01, shape = ObjectShape.ANY, color = ObjectColor.ANY)
	
	if obj_found:
		niryo_robot.wait(0.5)
                niryo_robot.control_conveyor(conveyor_id, True, 0, ConveyorDirection.FORWARD)
	
       

                if(color == ObjectColor.RED):
                        offset = -0.015
                        niryo_robot.vision_pick(workspace_name, offset , shape, color)
                        next_place_pose = (-0.028, -0.254, 0.049, -0.209, 1.535, -1.704)
                        niryo_robot.place_from_pose(*next_place_pose)
                
                elif(color == ObjectColor.BLUE):
                        offset = -0.015
                        niryo_robot.vision_pick(workspace_name, offset, shape, color)
                        next_place_pose = (0.033, 0.298,0.064,-0.231, 1.548, 1.278 )
                        niryo_robot.place_from_pose(*next_place_pose)

                elif(color == ObjectColor.GREEN):
                        offset = -0.015
                        niryo_robot.vision_pick(workspace_name, offset, shape, color)
                        next_place_pose = (-0.166, -0.204 ,0.061,2.515, 1.531, 0.064 )
                        niryo_robot.place_from_pose(*next_place_pose)

                #next_place_pose = (*observation_pose)

        else:
		continue     
	
	#niryo_robot.place_from_pose(*next_place_pose)
niryo_robot.move_pose(*observation_pose)
niryo_robot.end()
