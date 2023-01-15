'''
A program for the DJI Robomaster S1: scan and follow

The robot will scan the area around it until it identifys
another S1 using its vision system. Once spotted, it follows
and maintains a constant distance. Losing sight will cause 
a return to scanning.

'''

# init PID controllers
pid_x = rm_ctrl.PIDCtrl()
pid_y = rm_ctrl.PIDCtrl()
pid_m = rm_ctrl.PIDCtrl()

# Set following parameters
car_x = 0.5   # maintain S1 in image centre
car_y = 0.5
car_w = 0.25
car_h = 0.25  # maintain S1 distance using vertical bounding box height

def vision_recognized_car(msg):

    '''
    This function is called when another S1 is spotted
    The postion paramters are updated
    '''

    # Allow access to params
    global car_x
    global car_y
    global car_w
    global car_h

    detection_info = vision_ctrl.get_car_detection_info()
    car_x = detection_info[1]
    car_y = detection_info[2]
    car_w = detection_info[3]
    car_h = detection_info[4]

def start():
    '''
    Main script, which is run when the S1 is initiated
    '''

    # Allow access to params
    global car_x
    global car_y
    global car_w
    global car_h
    global pid_x
    global pid_y
    global pid_m

    # Set PID paramters (tuned manually)
    pid_x.set_ctrl_params(115, 0, 5)
    pid_y.set_ctrl_params(85, 0, 3)
    pid_m.set_ctrl_params(1, 0.1, 3)

    # Initialisation
    vision_ctrl.enable_detection(rm_define.vision_detection_car)
    robot_ctrl.set_mode(rm_define.robot_mode_chassis_follow)

    # Show that s1 is initialised by changing LED lighting to dim red
    led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 100, 0, 0, rm_define.effect_always_on)
    led_ctrl.set_top_led(rm_define.armor_top_all, 100, 0, 0, rm_define.effect_always_on)

    while(True): # Main loop runs until the S1 is terminated

        # When another S1 is recognised:
        while (vision_ctrl.check_condition(rm_define.cond_recognized_car)):

            # Set S1 chassis to follow gimbal and LEDs to red
            robot_ctrl.set_mode(rm_define.robot_mode_chassis_follow)
            led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 0, 0, rm_define.effect_always_on)
            led_ctrl.set_top_led(rm_define.armor_top_all, 255, 0, 0, rm_define.effect_always_on)

            print(car_h) # return car height to terminal

		 # Create error signal for each control variable
            pid_x.set_error(car_x - 0.5)
            pid_y.set_error(0.5 - car_y)
            pid_m.set_error(car_h-0.35)

		 # provide control signal to gimbal and chassis
            gimbal_ctrl.rotate_with_speed(pid_x.get_output(), pid_y.get_output())
            chassis_ctrl.move_with_speed(pid_m.get_output(), 0, 0) # Only move forwards

            time.sleep(0.05) # set refresh rate

        time.sleep(0.5) # Gives a moment for S1 to reidentify if lost for a split second

	   # When another S1 is not recognised:
        while not (vision_ctrl.check_condition(rm_define.cond_recognized_car)):

            # Set S1 gimbal to move independantly and LEDs to blue
            robot_ctrl.set_mode(rm_define.robot_mode_free)
            led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 0, 0, 255, rm_define.effect_always_on)
            led_ctrl.set_top_led(rm_define.armor_top_all, 0, 0, 255, rm_define.effect_always_on)

            # Scan surronds to search for S1
            gimbal_ctrl.set_rotate_speed(20)
            gimbal_ctrl.rotate_with_degree(rm_define.gimbal_left, 180)
            gimbal_ctrl.recenter()
            gimbal_ctrl.rotate_with_degree(rm_define.gimbal_right, 180)
