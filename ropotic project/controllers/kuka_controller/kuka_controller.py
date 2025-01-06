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
            finger.setPosition(float('inf'))
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
        
        self.left_distance_sensor.enable(self.timestep)
        self.right_distance_sensor.enable(self.timestep) 
        self.front_distance_sensor.enable(self.timestep) 

        self.color_sequence = []
        self.current_color_index = 0
        self.PID_KP = 0.05
        self.PID_KI = 0.01
        self.PID_KD = 0.005
        self.last_error = 0
        self.integral = 0

    def get_line_position(self):
        values = [sensor.getValue() for sensor in self.sensors]
        threshold = 500
        filtered_values = [1 if value > threshold else 0 for value in values]
        print("Filtered sensor values:", filtered_values)

        if sum(filtered_values) == 0:
            print("Line lost!")
            return 0  # إذا فقد الخط تمامًا

        error = sum([weight * value for weight, value in zip(self.weights, filtered_values)]) / sum(filtered_values)
        return error


    def follow_line(self):
        error = self.get_line_position()
        self.integral += error
        derivative = error - self.last_error

        correction = (self.PID_KP * error) + (self.PID_KI * self.integral) + (self.PID_KD * derivative)

        left_speed = max(min(4.0 - correction, 6.28), -6.28)
        right_speed = max(min(4.0 + correction, 6.28), -6.28)

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
        self.front_right_wheel.setVelocity(4.0)
        self.front_left_wheel.setVelocity(-4.0)
        self.back_left_wheel.setVelocity(-4.0)
        self.back_right_wheel.setVelocity(4.0)
        self.step(800)  # Adjust time based on turning needs

    def turn_right(self):
        self.front_right_wheel.setVelocity(-4.0)
        self.front_left_wheel.setVelocity(4.0)
        self.back_left_wheel.setVelocity(4.0)
        self.back_right_wheel.setVelocity(-4.0)
        # self.step(800)  # Adjust time based on turning needs
    def rotate_clockwise(self):
    # ضبط سرعات العجلات للدوران مع عقارب الساعة
        self.front_right_wheel.setVelocity(-14.0)
        self.back_right_wheel.setVelocity(-14.0)
        self.front_left_wheel.setVelocity(14.0)
        self.back_left_wheel.setVelocity(14.0)
        self.step(800)  # الوقت لضمان دوران 90 درجة

    def rotate_counterclockwise(self):
    # ضبط سرعات العجلات للدوران عكس عقارب الساعة
        self.front_right_wheel.setVelocity(14.0)
        self.back_right_wheel.setVelocity(14.0)
        self.front_left_wheel.setVelocity(-14.0)
        self.back_left_wheel.setVelocity(-14.0)
        self.step(800)  # الوقت لضمان دوران 90 درجة

    # def move_arm_to_pick(self):
    #     self.arm1.setPosition(1.0)
    #     self.arm2.setPosition(0.5)
    #     self.arm3.setPosition(-0.5)
    #     self.arm4.setPosition(0.0)
    #     self.finger_left.setPosition(0.0)
    #     self.finger_right.setPosition(0.0)

    # def move_arm_to_place(self):
    #     self.arm1.setPosition(0.0)
    #     self.arm2.setPosition(1.0)
    #     self.arm3.setPosition(0.5)
    #     self.arm4.setPosition(-0.5)
    #     self.finger_left.setPosition(1.0)
    #     self.finger_right.setPosition(1.0)
    
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
        print(f"r {filtered_right_values} l{filtered_left_values}")
        return filtered_right_values,filtered_left_values

    def stop(self):
        # تعيين سرعة جميع العجلات إلى 0
        self.front_left_wheel.setVelocity(0)
        self.back_left_wheel.setVelocity(0)
        self.front_right_wheel.setVelocity(0)
        self.back_right_wheel.setVelocity(0)

    def detect_object(self):
        threshold = 100000  # العتبة للكشف عن الجسم (المكعب)
        front_distance = self.front_distance_sensor.getValue()
        if front_distance < threshold :
            filtered_values = 1 
        else :
            filtered_values =0 
        return filtered_values
    def init_arm(self):
    # تهيئة محركات الذراع
        self.arm1.setPosition(0.0)
        self.arm2.setPosition(-0.5)
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
    def open_gripper(self):
    # فتح القابض
        self.finger_left.setPosition(0.025)
        self.finger_right.setPosition(0.025)
        self.step(1000)  # انتظار لإتمام الحركة

    def close_gripper(self):
    # إغلاق القابض
        self.finger_left.setPosition(0.0)
        self.finger_right.setPosition(0.0)
        self.step(1000)  # انتظار لإتمام الحركة

    def prepare_to_pick(self):
    # تحريك الذراع إلى وضع الالتقاط
        self.arm1.setPosition(-1.0)  # تدوير القاعدة
        self.step(2000)
    
        self.arm2.setPosition(1.0)   # خفض الذراع
        self.arm3.setPosition(-0.8)
        self.arm4.setPosition(-0.9)
        self.step(2000)
    
        self.open_gripper()

    def pick_object(self):
        # تسلسل حركات التقاط المكعب
        self.prepare_to_pick()        # تجهيز الذراع
        self.step(1000)              # انتظار
        self.close_gripper()         # إغلاق القابض
        self.step(1000)              # انتظار
    
        # رفع المكعب
        self.arm2.setPosition(0.0)
        self.arm3.setPosition(0.0)
        self.arm4.setPosition(0.0)
        self.step(2000)

    def execute_picking_sequence(self):
    # تنفيذ تسلسل الالتقاط الكامل
        print("Starting picking sequence...")
        self.init_arm()              # تهيئة الذراع
        self.step(1000)
        self.pick_object()           # التقاط المكعب
        print("Picking sequence completed")

    # تعديل دالة run لتشمل عملية الالتقاط
    def run(self):
        color_index = 1
        wait_start_time = None
        object_detected = False
        picking_completed = False

        while self.step(self.timestep) != -1:
            r, l = self.handle_distance_and_color()
            front_distance = self.detect_object()
        
            if front_distance == 1 and len(self.color_sequence) == 4 and not picking_completed:
                print("Stopping in front of the object...")
                self.stop()
                object_detected = True
                self.execute_picking_sequence()  # تنفيذ عملية الالتقاط
                picking_completed = True
                continue

            if not object_detected:
                self.follow_line()

            detected_color = self.detect_color()
            self.store_color_sequence(detected_color)
            print(f"Color sequence: {self.color_sequence}")

            # باقي الكود كما هو...
            if len(self.color_sequence) == 4:
                self.is_reading_colors = False
        
                if color_index < len(self.color_sequence):
                    current_color = self.color_sequence[color_index]
                    print(f"Processing color: {current_color}, Index: {color_index}")

                    if l == 1 and current_color == "green":
                        self.rotate_counterclockwise()
                        color_index += 1
                    elif current_color == "yellow":
                        if r == 1:
                            self.rotate_clockwise()
                            color_index += 1
                    elif l == 1 and current_color == "blue":
                        if wait_start_time is None:
                            print("Detected blue and left sensor is 1, starting wait...")
                            wait_start_time = time.time()
                        elif time.time() - wait_start_time >= 10:
                            r, l = self.handle_distance_and_color()
                            if l == 1:
                                self.rotate_counterclockwise()
                            color_index += 1
                            wait_start_time = None
                    elif current_color == "red":
                        print("yooooo")
                else:
                    print("Finished processing all colors")
                    break

            

     
# Instantiate and run the controller
robot = YouBotController()
robot.run()

















   