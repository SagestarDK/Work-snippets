#!/usr/bin/env python3
#
# TurtleBot3 Obstacle Detection - komplet program med grundige kommentarer

# ---- Importer nødvendige biblioteker ----

from geometry_msgs.msg import Twist       # Bruges til at sende beskeder om bevægelse til robotten
import rclpy                              # ROS2 - robot operativsystemet, der styrer noder
from rclpy.node import Node               # Node = en "del" af programmet der laver noget bestemt
from rclpy.qos import qos_profile_sensor_data, QoSProfile  # Indstillinger for, hvordan data sendes
from sensor_msgs.msg import LaserScan     # Modtager laserscanningsdata (afstande til objekter)
import smbus2 as smbus                    # Til at tale med farvesensoren via I2C
import RPi.GPIO as GPIO                   # Til at styre Raspberry Pi's elektriske ben (pins)
import time as time_module                # Bruges til at måle tid og lave pauser

# ---- Definerer selve robotklassen ----

class Turtlebot3ObstacleDetection(Node):
    """
    Denne klasse får robotten til at:
    - Køre fremad
    - Stoppe eller dreje for at undgå forhindringer
    - Registrere rød farve og tænde en LED
    """

    def __init__(self):
        # Starter ROS2-node med navnet 'turtlebot3_obstacle_detection'
        super().__init__('turtlebot3_obstacle_detection')
        self.get_logger().info("Robot klar til at starte.")

        # --- Indstillinger for robotbevægelse ---
        self.scan_ranges = []            # Her gemmer vi laserscannerens afstandsdata
        self.has_scan_received = False   # Har vi modtaget data endnu?

        self.max_speed = 0.2              # Robotten kører 0.2 meter pr. sekund som standard
        self.max_angle = 1.28             # Robotten kan maksimalt dreje 1.28 radianer/sekund
        self.divisions = 20               # Del laserscanneren op i 20 stykker
        self.critical_distance = 0.16     # Hvis noget er tættere end 16 cm = kollision
        self.start_turning = 0.4           # Robot begynder at dreje hvis noget er tættere end 40 cm
        self.stop_distance = 0.2           # Robot skal stoppe og pivotere hvis noget er tættere end 20 cm
        self.pivot_distance = 0.3          # Grænse hvor robotten vælger at pivotere
        self.turn_factor = 1               # Bruges til at vælge venstre eller højre drejning
        self.r_d = -1                      # Højre drejning (negativ)
        self.l_d = 1                       # Venstre drejning (positiv)
        self.min_speed = 0.07              # Minimum fremad hastighed, så robotten ikke stopper helt

        # --- Kollisionstæller ---
        self.collision_counter = 0          # Hvor mange kollisioner der er sket
        self.last_collision_time = 0.0      # Tidspunkt for sidste kollision
        self.collision_delay = 2.0           # Vent 2 sekunder mellem kollisionstællinger

        # --- Logning af gennemsnitshastighed ---
        self.speed_sum = 0.0                # Samlet distance målt
        self.speed_count = 0                # Hvor mange gange vi har målt hastighed
        self.last_log_time = 0.0             # Sidste gang vi skrev en log
        self.log_interval = 1.0              # Log hvert sekund

        # --- Farvesensor og LED styring ---
        self.init_colour_sensor()            # Sætter farvesensoren op
        self.color_buffer = []               # Gemmer de seneste farvelæsninger
        self.buffer_limit = 1                # Hvor mange farvelæsninger vi gemmer
        self.last_color_log_time = time_module.time()  # Sidste gang farve blev logget
        self.victim_counter = 0              # Hvor mange røde "ofre" er fundet
        self.victim_detected = False         # Om vi har fundet et offer

        self.LED_PIN = 17                    # LED'en sidder på GPIO ben 17
        GPIO.setmode(GPIO.BCM)               # Brug fysisk ben-nummerering på Raspberry Pi
        GPIO.setup(self.LED_PIN, GPIO.OUT)   # Sæt ben 17 som output (dvs. vi kan sende strøm)

        # --- ROS2 kommunikation setup ---
        qos = QoSProfile(depth=10)            # Hvor mange beskeder der må ligge i kø
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)  # Bruges til at sende bevægelseskommandoer
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)  # Læser laserscanner
        self.timer = self.create_timer(0.1, self.timer_callback)  # Kør timer_callback hvert 0.1 sekund

        # --- Manuel styring (ekstra mulighed) ---
        self.tele_twist = Twist()             # Bruges hvis man vil styre robotten selv
        self.tele_twist.linear.x = 0.2
        self.tele_twist.angular.z = 0.0
        self.cmd_vel_raw_sub = self.create_subscription(Twist, 'cmd_vel_raw', self.cmd_vel_raw_callback, qos_profile_sensor_data)

    def init_colour_sensor(self):
        """Sætter farvesensoren op via I2C på Raspberry Pi."""
        try:
            self.bus = smbus.SMBus(1)  # Starter kommunikation på bus 1 (standard på Raspberry Pi)
            self.bus.write_byte_data(0x44, 0x01, 0x05)  # Sætter sensor i aktiv tilstand
            time_module.sleep(1)  # Vent 1 sekund på at sensor starter
            self.get_logger().info("Farvesensor initialiseret.")
        except Exception as e:
            self.get_logger().error(f"Fejl ved opsætning af farvesensor: {e}")
            self.bus = None

    def update_colour(self):
        """Læs farve fra sensor, opdater LED, og registrér røde ofre."""
        if not self.bus:
            return  # Hvis sensoren ikke findes, stop

        try:
            data = self.bus.read_i2c_block_data(0x44, 0x09, 6)  # Læs 6 bytes farvedata
            green = (data[1] * 0.9) + data[0] / 256.0  # Beregn grøn værdi
            red   = data[3] + data[2] / 256.0          # Beregn rød værdi
            blue  = (data[5] * 1.85) + data[4] / 256.0 # Beregn blå værdi

            # Bestem hvilken farve vi ser mest
            if green < 20 and blue < 20 and red < 20:
                color = 'No color'
            elif green > red and green > blue:
                color = "Green"
            elif blue > red:
                color = "Blue"
            else:
                color = "Red"

            # Gem farven i buffer
            self.color_buffer.append(color)
            if len(self.color_buffer) > self.buffer_limit:
                self.color_buffer.pop(0)  # Fjern gamle målinger

            # Hvis vi ser konstant rød farve → offer fundet
            if all(c == "Red" for c in self.color_buffer) and not self.victim_detected:
                self.victim_detected = True
                self.victim_counter += 1
                GPIO.output(self.LED_PIN, GPIO.HIGH)  # HIGH = Tænd LED
                self.get_logger().info("‼️ Offer fundet!")
            else:
                if not self.victim_detected:
                    GPIO.output(self.LED_PIN, GPIO.LOW)  # LOW = Sluk LED

            # Log farver hver sekund
            now = time_module.time()
            if now - self.last_color_log_time > 1.0:
                self.last_color_log_time = now
                self.get_logger().info(f"Farve: R{int(red)} G{int(green)} B{int(blue)} → {color}, Ofre: {self.victim_counter}")

        except Exception as e:
            self.get_logger().error(f"Fejl ved læsning af farvesensor: {e}")

    def angle_factor(self, front_d):
        """Beregner hvor meget robotten skal dreje baseret på hvor tæt en forhindring er."""
        x = abs(self.start_turning - front_d)
        return x / (self.start_turning - self.stop_distance)  # Jo tættere på stop_distance, jo skarpere drejning

    def speed_factor(self, front_d):
        """Beregner hvor hurtigt robotten må køre baseret på afstand til forhindring."""
        if front_d <= self.stop_distance:
            return self.min_speed / self.max_speed  # Gør hastigheden meget lav hvis tæt på
        if front_d >= self.start_turning:
            return 1  # Kør normalt hvis intet er tæt på
        # Lineær interpolation mellem min hastighed og maks hastighed
        factor = (front_d - self.stop_distance) / (self.start_turning - self.stop_distance)
        min_factor = self.min_speed / self.max_speed
        return factor * (1 - min_factor) + min_factor

    def set_rot_d(self, r_dist, l_dist):
        """Vælg om robotten skal dreje til højre eller venstre afhængigt af hvor der er mest plads."""
        for i in range(self.divisions // 4 - 1):
            min_r = min(r_dist[i], r_dist[i+1])
            min_l = min(l_dist[i], l_dist[i+1])

            if self.stop_distance * 1.5 < max(min_l, min_r):
                self.turn_factor = self.r_d if min_l < min_r else self.l_d
                return  # Vælg side med mest plads
            if self.start_turning / 1.5 < min_r:
                self.turn_factor = self.r_d  # Drej til højre
                return
            if self.start_turning / 1.5 < min_l:
                self.turn_factor = self.l_d  # Drej til venstre
                return

    def scan_callback(self, msg):
        """Modtager laserscanner-data og gemmer det."""
        self.scan_ranges = msg.ranges
        self.has_scan_received = True  # Marker at vi nu har data

    def cmd_vel_raw_callback(self, msg):
        """Manuel styring fra brugeren (f.eks. joystick eller keyboard)."""
        if msg.linear.x < 0:
            msg.linear.x = 0.0  # Tillad ikke at køre baglæns
        self.tele_twist = msg

    def timer_callback(self):
        """Denne funktion kører automatisk hver 0.1 sekund."""
        if self.has_scan_received:
            self.detect_obstacle()  # Hvis vi har fået laserdata, tjek forhindringer
        self.update_colour()       # Opdater farvesensor

    def detect_obstacle(self):
        """Den vigtigste funktion - her finder vi ud af hvordan robotten skal bevæge sig."""

        n = len(self.scan_ranges)
        if n == 0:
            return  # Hvis ingen data endnu, gør ingenting
        now = time_module.time()

        # Del laserscanneren op i venstre og højre side
        range_size = int(n / self.divisions)
        left_distances = []  # Her gemmer vi minimumsafstande på venstre side
        right_distances = []  # Og her på højre side

        for i in range(self.divisions // 4):
            seg_left = self.scan_ranges[range_size * i : range_size * (i + 1)]  # Et segment venstre
            seg_right = self.scan_ranges[(n - 1) - range_size * (i + 1) : (n - 1) - range_size * i]  # Et segment højre

            left_distances.append(min(seg_left) if seg_left else 10.0)   # Hvis segmentet er tomt, brug 10m som standard
            right_distances.append(min(seg_right) if seg_right else 10.0)

        # Find den mindste afstand foran robotten (højre eller venstre)
        front_distance = min(left_distances[0], right_distances[0])

        # Tjek om vi er meget tæt på noget (kollision)
        if front_distance < self.critical_distance:
            if (now - self.last_collision_time) > self.collision_delay:
                self.collision_counter += 1
                self.last_collision_time = now

        # Bestem hvilken retning der er bedst at dreje
        self.set_rot_d(right_distances, left_distances)

        # Lav en ny bevægelsesbesked
        twist = Twist()

        # Vælg hvordan vi skal bevæge os baseret på hvor tæt vi er på en forhindring
        if front_distance <= self.stop_distance:
            # Meget tæt → pivot langsomt og drej hurtigt
            twist.linear.x = self.min_speed
            twist.angular.z = self.max_angle * self.turn_factor
        elif front_distance < self.pivot_distance:
            # Tæt, men ikke helt → kør langsommere og begynd at dreje
            factor = self.speed_factor(front_distance)
            twist.linear.x = self.max_speed * factor
            twist.angular.z = self.max_angle * self.turn_factor * self.angle_factor(front_distance)
        else:
            # Ingen forhindringer → kør ligeud
            twist.linear.x = self.max_speed
            twist.angular.z = 0.0

        # Sørg for at hastigheden aldrig bliver for lav
        if twist.linear.x < self.min_speed:
            twist.linear.x = self.min_speed

        # Send bevægelsesbesked til motorerne
        self.cmd_vel_pub.publish(twist)

        # Opdater gennemsnitlig hastighedsberegning
        self.update_average_speed(twist)

        # Log information hvis det er tid
        self.log_status(now)

    def update_average_speed(self, twist):
        """Opdater data så vi kan finde gennemsnitlig fremad hastighed."""
        self.speed_sum += abs(twist.linear.x)
        self.speed_count += 1

    def log_status(self, now):
        """Logger information om kollisioner og gennemsnitshastighed regelmæssigt."""
        if (now - self.last_log_time) >= self.log_interval:
            avg_speed = self.speed_sum / self.speed_count if self.speed_count else 0.0
            self.get_logger().info(f"Kollisioner: {self.collision_counter}, Gns. hastighed: {avg_speed:.2f} m/s")
            self.speed_sum = 0.0
            self.speed_count = 0
            self.last_log_time = now

# ---- Hovedprogrammet ----

def main(args=None):
    """Starter robotprogrammet."""
    rclpy.init(args=args)                # Starter ROS2 systemet
    node = Turtlebot3ObstacleDetection() # Opretter vores robotklasse
    try:
        rclpy.spin(node)                  # Holder programmet kørende
    finally:
        GPIO.cleanup()                    # Slukker Raspberry Pi's pins sikkert (vigtigt for hardware)
        node.destroy_node()               # Lukker node
        rclpy.shutdown()                  # Lukker ROS2

# Hvis vi kører filen direkte, så start main()
if __name__ == '__main__':
    main()

