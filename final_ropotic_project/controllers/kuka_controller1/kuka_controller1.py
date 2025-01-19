import time
from controller import Robot, Motor, Camera, DistanceSensor

class YouBotController(Robot):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())

        # Motors for wheels
        self.front_right_wheel = self.getDevice("wheel1")
        self.front_left_wheel = self.getDevice("wheel2")
        self.back_right_wheel = self.getDevice("wheel3")
        self.back_left_wheel = self.getDevice("wheel4")

        for wheel in [self.front_right_wheel, self.front_left_wheel, self.back_right_wheel, self.back_left_wheel]:
            wheel.setPosition(float('inf'))
            wheel.setVelocity(0)

        # Motors for the arm
        self.arm1 = self.getDevice("arm1")
        self.arm2 = self.getDevice("arm2")
        self.arm3 = self.getDevice("arm3")
        self.arm4 = self.getDevice("arm4")
        self.arm5 = self.getDevice("arm5")
        self.finger_left = self.getDevice("finger::left")
        self.finger_right = self.getDevice("finger::right")

        for arm_motor in [self.arm1, self.arm2, self.arm3, self.arm4, self.arm5]:
            arm_motor.setPosition(float('inf'))
            arm_motor.setVelocity(0)

        for finger in [self.finger_left, self.finger_right]:
            finger.setPosition(0.0)
            finger.setVelocity(0)
        

        # Camera for color detection
        self.camera = self.getDevice("color_camera")
        self.camera.enable(self.timestep)
        

        # Line following sensors
        self.sensors = [self.getDevice(f"s{i}") for i in range(8)]
        self.weights = [-1000, -1000, -1000, -1000, 1000, 1000, 1000, 1000]
        
        for sensor in self.sensors:
            sensor.enable(self.timestep)
        # Distance sensors
        self.left_distance_sensor = self.getDevice("left_distance_sensor")
        self.right_distance_sensor = self.getDevice("right_distance_sensor")
        self.front_distance_sensor = self.getDevice("d1") 
        self.front_distance_sensor2 = self.getDevice("d2") 
        self.left_distance_sensor.enable(self.timestep)
        self.right_distance_sensor.enable(self.timestep) 
        self.front_distance_sensor.enable(self.timestep)
        self.front_distance_sensor2.enable(self.timestep)

        self.color_sequence = []
        self.current_color_index = 0
        self.v=4.0
        self.PID_KP = 0.07
        self.PID_KI = 0.01
        self.PID_KD = 0.003
        self.last_error = 0
        self.integral = 0

    def get_line_position(self):
        values = [sensor.getValue() for sensor in self.sensors]
        threshold = 500
        filtered_values = [1 if value > threshold else 0 for value in values]
        print("Filtered sensor values:", filtered_values)

        if sum(filtered_values) == 0:
            print("Line lost!")
            return 0  

        error = sum([weight * value for weight, value in zip(self.weights, filtered_values)]) / sum(filtered_values)
        return error


    def follow_line(self):
        error = self.get_line_position()
        self.integral += error
        derivative = error - self.last_error

        correction = (self.PID_KP * error) + (self.PID_KI * self.integral) + (self.PID_KD * derivative)

        left_speed = max(min(self.v - correction, 6.28), -6.28)
        right_speed = max(min(self.v + correction, 6.28), -6.28)

        self.front_left_wheel.setVelocity(left_speed)
        self.back_left_wheel.setVelocity(left_speed)
        self.front_right_wheel.setVelocity(right_speed)
        self.back_right_wheel.setVelocity(right_speed)

        self.last_error = error

    def detect_color(self):
        image = self.camera.getImage()
        red = self.camera.imageGetRed(image, self.camera.getWidth(), 0, 0)
        green = self.camera.imageGetGreen(image, self.camera.getWidth(), 0, 0)
        blue = self.camera.imageGetBlue(image, self.camera.getWidth(), 0, 0)

        if red > green and red > blue:
            return "red"
        elif green > red and green > blue:
            return "green"
        elif blue > red and blue > green:
            return "blue"
        elif red > 0 and green > 0 and blue < 50:
            return "yellow"
        else:
            return "unknown"
    def turn_left(self):
        self.front_right_wheel.setVelocity(self.v)
        self.front_left_wheel.setVelocity(-self.v)
        self.back_left_wheel.setVelocity(-self.v)
        self.back_right_wheel.setVelocity(self.v)
        self.step(800) 

    def turn_right(self):
        self.front_right_wheel.setVelocity(-self.v)
        self.front_left_wheel.setVelocity(self.v)
        self.back_left_wheel.setVelocity(self.v)
        self.back_right_wheel.setVelocity(-self.v)
        self.step(800)  
    def rotate_clockwise(self):
   
        self.front_right_wheel.setVelocity(-14.0)
        self.back_right_wheel.setVelocity(-14.0)
        self.front_left_wheel.setVelocity(14.0)
        self.back_left_wheel.setVelocity(14.0)
        self.step(800)  

    def rotate_counterclockwise(self):
    # ضبط سرعات العجلات للدوران عكس عقارب الساعة
        self.front_right_wheel.setVelocity(14.0)
        self.back_right_wheel.setVelocity(14.0)
        self.front_left_wheel.setVelocity(-14.0)
        self.back_left_wheel.setVelocity(-14.0)
        self.step(800)  
    
    def store_color_sequence(self, detected_color):
        if detected_color not in ["unknown", None]:
            if detected_color not in self.color_sequence and len(self.color_sequence) < 4:
                self.color_sequence.append(detected_color)
                print(f"Added color to sequence: {detected_color}")
                print(f"Current sequence: {self.color_sequence}")

    
    def handle_distance_and_color(self):
        threshold = 500
        left_distance = self.left_distance_sensor.getValue()
        right_distance = self.right_distance_sensor.getValue()
        if left_distance > threshold :
            filtered_left_values = 1 
        else :
            filtered_left_values =0 
        if right_distance > threshold :
            filtered_right_values = 1 
        else :
            filtered_right_values =0 
        print(f"r {right_distance} l{left_distance}")
        print(f"r {filtered_right_values} l{filtered_left_values}")
        return filtered_right_values,filtered_left_values

    def stop(self):
      
        self.front_left_wheel.setVelocity(0)
        self.back_left_wheel.setVelocity(0)
        self.front_right_wheel.setVelocity(0)
        self.back_right_wheel.setVelocity(0)

    def detect_object(self):
        threshold = 100000 
        front_distance = self.front_distance_sensor.getValue()
        if front_distance < threshold :
            filtered_values = 1 
        else :
            filtered_values =0 
        return filtered_values
    def detect_object2(self):
        threshold = 100000 
        front_distance = self.front_distance_sensor2.getValue()
        print(f"yooooooooooo{front_distance}")
        if front_distance < threshold :
            filtered_values = 1 
        else :
            filtered_values =0 
        return filtered_values
    def init_arm(self):
    # تهيئة محركات الذراع
        self.arm1.setPosition(0.0)
        self.arm2.setPosition(0.0)
        self.arm3.setPosition(0.0)
        self.arm4.setPosition(0.0)
        self.arm5.setPosition(0.0)
        self.finger_left.setPosition(0.0)
        self.finger_right.setPosition(0.0)
    
        # تعيين سرعة المحركات
        self.arm1.setVelocity(0.5)
        self.arm2.setVelocity(0.5)
        self.arm3.setVelocity(0.5)
        self.arm4.setVelocity(0.5)
        self.arm5.setVelocity(0.5)
        self.finger_left.setVelocity(0.5)
        self.finger_right.setVelocity(0.5)
    
    def lower_arm(self):
        self.arm2.setPosition(-0.8)
        self.arm3.setPosition(-2)
        self.arm4.setPosition(0)
        self.step(1000)
    
    def open_gripper(self):
        self.finger_left.setPosition(0.025)
        self.finger_right.setPosition(0.025)
        self.step(1000)  

    def close_gripper(self):
        self.finger_left.setPosition(0.0)
        self.finger_right.setPosition(0.0)
        self.step(1000)  
    
    def lift_arm(self):
     
        self.arm2.setPosition(0.0)
        self.arm3.setPosition(0.0)
        self.arm4.setPosition(0.0)
        self.step(1000)

    def pick_object(self):
       
        self.open_gripper()
        self.step(1000)
        self.arm2.setPosition(0.1)
        self.arm3.setPosition(-0.6)
        self.arm4.setPosition(-0.9)
        self.step(4000)
        self.close_gripper()
        self.step(1000)
        self.lift_arm()
        self.step(1000)         

    def execute_picking_sequence(self):
        print("Starting picking sequence...")
        self.init_arm()             
        self.step(10)
        self.pick_object()          
        print("Picking sequence completed")
        
    def rotate_180_degrees(self):
        self.front_right_wheel.setVelocity(-14.0)
        self.back_right_wheel.setVelocity(-14.0)
        self.front_left_wheel.setVelocity(14.0)
        self.back_left_wheel.setVelocity(14.0)
        self.step(1600)  

    def release_object(self):
        
        self.arm2.setPosition(-0.8)
        self.arm3.setPosition(-2)
        self.arm4.setPosition(0)
        self.step(4000)
        self.open_gripper()
        self.step(1000)
        self.lift_arm()
        self.step(1000)
    def move_horizontally(self, direction='right'):
        if direction == 'right':
            self.front_right_wheel.setVelocity(-self.v)
            self.back_right_wheel.setVelocity(self.v)
            self.front_left_wheel.setVelocity(self.v)
            self.back_left_wheel.setVelocity(-self.v)
        else:  # left
            self.front_right_wheel.setVelocity(self.v)
            self.back_right_wheel.setVelocity(-self.v)
            self.front_left_wheel.setVelocity(-self.v)
            self.back_left_wheel.setVelocity(self.v)

    def run(self):
        color_index =0
        index=0
        has_cube = False
        returning_to_main = False
        wait_start_time = None
        direction = 'right'
        cube_grasped = False
        while self.step(self.timestep) != -1:
           
            r, l = self.handle_distance_and_color()
            front_distance = self.detect_object()
        
           
            if len(self.color_sequence) < 4:
                self.follow_line()
                detected_color = self.detect_color()
                self.store_color_sequence(detected_color)
                continue
            
            current_color = self.color_sequence[color_index]
            print(f"Processing color: {current_color}, Index: {color_index}")
        
            if not returning_to_main:
                if not has_cube:
                    self.follow_line()
                    front_distance = self.detect_object()
                    if front_distance == 1:
                        self.stop()
    
                        while self.step(self.timestep) != -1:
                            front_distance2 = self.detect_object2()
                            print(front_distance2)
                            if front_distance2 == 1:
                                  
                                if color_index==0:
                                    # self.front_right_wheel.setVelocity(self.v)
                                    # self.back_right_wheel.setVelocity(-self.v)
                                    # self.front_left_wheel.setVelocity(self.v)
                                    # self.back_left_wheel.setVelocity(-self.v)
                                    # self.step(85)
                                    self.stop()
                                else :
                                    self.stop()
                                if index == 0 and not cube_grasped:
                                    self.execute_picking_sequence()
                                    front_distance22 = self.detect_object2()
                                    if front_distance22 == 0:  # تحقق إذا تم التقاط المكعب بنجاح
                                        values = [sensor.getValue() for sensor in self.sensors]
                                        threshold = 500
                                        filtered_values = [1 if value > threshold else 0 for value in values]
            
                                        if sum(filtered_values) > 0 :  # اكتشاف الخط
                                            self.stop()
                                            cube_grasped = True
                                            self.rotate_180_degrees()
                                            has_cube = True
                                            index = 0
                                            break
                                        else: 
                                            self.move_horizontally(direction)
                                        
                                    else:
                                        index += 1
            
                                if index == 3 and not cube_grasped:
                                    self.front_right_wheel.setVelocity(self.v)
                                    self.back_right_wheel.setVelocity(-self.v)
                                    self.front_left_wheel.setVelocity(-self.v)
                                    self.back_left_wheel.setVelocity(self.v)
                                    self.step(50)
                                    self.stop()
                                    self.execute_picking_sequence()
                                    self.step(50)
                                    self.front_right_wheel.setVelocity(-self.v)
                                    self.back_right_wheel.setVelocity(self.v)
                                    self.front_left_wheel.setVelocity(self.v)
                                    self.back_left_wheel.setVelocity(-self.v)
                                    self.step(50)
                                    self.stop()
                                    front_distance22 = self.detect_object2()
                                    if front_distance22 == 0:  # تحقق إذا تم التقاط المكعب بنجاح
                                        values = [sensor.getValue() for sensor in self.sensors]
                                        threshold = 500
                                        filtered_values = [1 if value > threshold else 0 for value in values]
            
                                        if sum(filtered_values) > 0 :  # اكتشاف الخط
                                            self.stop()
                                            cube_grasped = True
                                            self.rotate_180_degrees()
                                            has_cube = True
                                            index = 0
                                            break
                                        else: 
                                            self.move_horizontally(direction)
                                        
                                    else:
                                        index += 1
                                if index == 2 and not cube_grasped:
                                    self.front_right_wheel.setVelocity(-self.v)
                                    self.back_right_wheel.setVelocity(self.v)
                                    self.front_left_wheel.setVelocity(self.v)
                                    self.back_left_wheel.setVelocity(-self.v)
                                    self.step(50)
                                    self.stop()
                                    self.execute_picking_sequence()
                                    self.step(50)
                                    self.front_right_wheel.setVelocity(self.v)
                                    self.back_right_wheel.setVelocity(-self.v)
                                    self.front_left_wheel.setVelocity(-self.v)
                                    self.back_left_wheel.setVelocity(self.v)
                                    self.step(50)
                                    self.stop()
                                    front_distance22 = self.detect_object2()
                                    if front_distance22 == 0:  
                                        values = [sensor.getValue() for sensor in self.sensors]
                                        threshold = 500
                                        filtered_values = [1 if value > threshold else 0 for value in values]
            
                                        if sum(filtered_values) > 0 :  
                                            self.stop()
                                            cube_grasped = True
                                            self.rotate_180_degrees()
                                            has_cube = True
                                            index = 0
                                            break
                                        else: 
                                            self.move_horizontally(direction)
                                        
                                    else:
                                        index += 1
                                if index == 4 and not cube_grasped:
                                    self.front_right_wheel.setVelocity(-self.v)
                                    self.back_right_wheel.setVelocity(self.v)
                                    self.front_left_wheel.setVelocity(-self.v)
                                    self.back_left_wheel.setVelocity(self.v)
                                    self.step(50)
                                    self.stop()
                                    self.execute_picking_sequence()
                                    self.step(50)
                                    self.front_right_wheel.setVelocity(self.v)
                                    self.back_right_wheel.setVelocity(-self.v)
                                    self.front_left_wheel.setVelocity(self.v)
                                    self.back_left_wheel.setVelocity(-self.v)
                                    self.step(50)
                                    self.stop()
                                    front_distance22 = self.detect_object2()
                                    if front_distance22 == 0:  
                                        values = [sensor.getValue() for sensor in self.sensors]
                                        threshold = 500
                                        filtered_values = [1 if value > threshold else 0 for value in values]
            
                                        if sum(filtered_values) > 0 :  
                                            self.stop()
                                            cube_grasped = True
                                            self.rotate_180_degrees()
                                            has_cube = True
                                            index = 0
                                            break
                                        else: 
                                            self.move_horizontally(direction)
                                        
                                    else:
                                        index += 1
                                if index == 1 and not cube_grasped:
                                    self.front_right_wheel.setVelocity(self.v)
                                    self.back_right_wheel.setVelocity(-self.v)
                                    self.front_left_wheel.setVelocity(self.v)
                                    self.back_left_wheel.setVelocity(-self.v)
                                    self.step(100)
                                    self.stop()
                                    self.execute_picking_sequence()
                                    self.step(50)
                                    self.front_right_wheel.setVelocity(-self.v)
                                    self.back_right_wheel.setVelocity(self.v)
                                    self.front_left_wheel.setVelocity(-self.v)
                                    self.back_left_wheel.setVelocity(self.v)
                                    self.step(100)
                                    self.stop()       
                                    front_distance22 = self.detect_object2()
                                    if front_distance22 == 0:  
                                        values = [sensor.getValue() for sensor in self.sensors]
                                        threshold = 500
                                        filtered_values = [1 if value > threshold else 0 for value in values]
            
                                        if sum(filtered_values) > 0 :  
                                            self.stop()
                                            cube_grasped = True
                                            self.rotate_180_degrees()
                                            has_cube = True
                                            index = 0
                                            break
                                        else: 
                                            self.move_horizontally(direction)
                                        
                                    else:
                                        index += 1
                                if index == 5 and not cube_grasped:
                                    self.front_right_wheel.setVelocity(-self.v)
                                    self.back_right_wheel.setVelocity(self.v)
                                    self.front_left_wheel.setVelocity(self.v)
                                    self.back_left_wheel.setVelocity(-self.v)
                                    self.step(50)
                                    self.front_right_wheel.setVelocity(self.v)
                                    self.back_right_wheel.setVelocity(-self.v)
                                    self.front_left_wheel.setVelocity(self.v)
                                    self.back_left_wheel.setVelocity(-self.v)
                                    self.step(100)
                                    self.stop()
                                    self.execute_picking_sequence()
                                    self.step(50)
                                    self.front_right_wheel.setVelocity(-self.v)
                                    self.back_right_wheel.setVelocity(self.v)
                                    self.front_left_wheel.setVelocity(-self.v)
                                    self.back_left_wheel.setVelocity(self.v)
                                    self.step(100)
                                    self.front_right_wheel.setVelocity(self.v)
                                    self.back_right_wheel.setVelocity(-self.v)
                                    self.front_left_wheel.setVelocity(-self.v)
                                    self.back_left_wheel.setVelocity(self.v)
                                    self.step(50)
                                    self.stop()       
                                    front_distance22 = self.detect_object2()
                                    if front_distance22 == 0:  
                                        values = [sensor.getValue() for sensor in self.sensors]
                                        threshold = 500
                                        filtered_values = [1 if value > threshold else 0 for value in values]
            
                                        if sum(filtered_values) > 0 :  
                                            self.stop()
                                            cube_grasped = True
                                            self.rotate_180_degrees()
                                            has_cube = True
                                            index = 0
                                            break
                                        else: 
                                            self.move_horizontally(direction)
                                        
                                    else:
                                        index += 1    
                          
                            self.move_horizontally(direction)
                            if direction == 'right':
                                direction = 'left'
                            else:
                                direction = 'right'

                            self.step(1500)
                         
                                

                               

                else:  
                    values = [sensor.getValue() for sensor in self.sensors]
                    threshold = 500
                    filtered_values = [1 if value > threshold else 0 for value in values]
                    if sum(filtered_values) == 0 :  # اكتشاف الخط
                        self.front_right_wheel.setVelocity(-self.v)
                        self.back_right_wheel.setVelocity(-self.v)
                        self.front_left_wheel.setVelocity(self.v)
                        self.back_left_wheel.setVelocity(self.v)
                        self.step(300)
                        self.stop()
                    else: 
                        self.follow_line()
                    # self.follow_line()
                    r, l = self.handle_distance_and_color()
                    front_distance = self.detect_object()
                    if current_color == "yellow" and l == 1:
                        self.step(100)
                        self.turn_left()
                        while self.step(self.timestep) != -1:
                            values = [sensor.getValue() for sensor in self.sensors]
                            threshold = 500
                            filtered_values = [1 if value > threshold else 0 for value in values]
        
                            if sum(filtered_values) == 0:  
                                self.stop()
                                self.release_object()
                                self.rotate_180_degrees()
                                self.step(100)
                                returning_to_main = True
                                break    
                            self.follow_line()
                             
                    elif current_color == "blue" and r == 1:
                        self.step(100)
                        self.turn_right()
                        while self.step(self.timestep) != -1:
                            values = [sensor.getValue() for sensor in self.sensors]
                            threshold = 500
                            filtered_values = [1 if value > threshold else 0 for value in values]
            
                            if sum(filtered_values) == 0:
                                self.stop()
                                self.release_object()
                                self.rotate_180_degrees()
                                self.step(100)
                                returning_to_main = True                              
                                break
                            self.follow_line()
                            
                    elif current_color == "green" and r == 1:
                        if wait_start_time is None:
                            print("Detected blue and left sensor is 1, starting wait...")
                            wait_start_time = time.time()
                        elif time.time() - wait_start_time >= 10:
                            r, l = self.handle_distance_and_color()
                            if r == 1:
                                self.step(100)
                                self.turn_right()
                                wait_start_time = None
                                while self.step(self.timestep) != -1:
                                    values = [sensor.getValue() for sensor in self.sensors]
                                    threshold = 500
                                    filtered_values = [1 if value > threshold else 0 for value in values]
            
                                    if sum(filtered_values) == 0:
                                        self.stop()
                                        self.release_object()
                                        self.rotate_180_degrees()
                                        self.step(100)
                                        returning_to_main = True  
                                        break
                                    self.follow_line()
                    elif current_color == "red":
                        while self.step(self.timestep) != -1:
                            values = [sensor.getValue() for sensor in self.sensors]
                            threshold = 500
                            filtered_values = [1 if value > threshold else 0 for value in values]
            
                            if sum(filtered_values) == 0:
                                self.stop()
                                self.release_object()
                                self.rotate_180_degrees()
                                self.step(100)
                                returning_to_main = True
                                break
                            self.follow_line()

                    
            else:  # في طريق العودة
                self.follow_line()
                if current_color == "yellow":
                    r, l = self.handle_distance_and_color()    
                    if r == 1: 
                        self.step(300)
                        self.turn_right()
                        self.follow_line()
                        color_index += 1
                        wait_start_time = None
                        has_cube = False
                        returning_to_main = False
                        direction = 'right'
                        cube_grasped = False
                if current_color == "green" :
                    r, l = self.handle_distance_and_color()         
                    if l == 1:
                        self.step(300)
                        self.turn_left()
                        self.follow_line()
                        color_index += 1
                        wait_start_time = None
                        has_cube = False
                        returning_to_main = False
                        direction = 'right' 
                        cube_grasped = False
                if current_color == "blue":
                    r, l = self.handle_distance_and_color()         
                    if l == 1:
                        self.step(300)
                        self.turn_left()
                        self.follow_line()
                        color_index += 1
                        wait_start_time = None
                        has_cube = False
                        returning_to_main = False
                        direction = 'right' 
                        cube_grasped = False
                if current_color == "red":
                    self.follow_line()
                    color_index += 1
                    wait_start_time = None
                    has_cube = False
                    returning_to_main = False
                    direction = 'right' 
                    cube_grasped = False

                if color_index >= len(self.color_sequence):
                    print("Mission completed!")
                    self.stop()
                    break

     
robot = YouBotController()
robot.run()















