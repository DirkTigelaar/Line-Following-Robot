from machine import Pin, I2C
from time import sleep_us, sleep_ms, ticks_us, ticks_diff

# ----- MPU6050 Setup -----
class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')  # Wake up

    def read_raw(self, reg):
        data = self.i2c.readfrom_mem(self.addr, reg, 2)
        value = (data[0] << 8) | data[1]
        if value > 32767:
            value -= 65536
        return value

    def get_accel(self):
        ax = self.read_raw(0x3B) / 16384.0
        ay = self.read_raw(0x3D) / 16384.0
        az = self.read_raw(0x3F) / 16384.0
        return ax, ay, az

    def get_gyro(self):
        gx = self.read_raw(0x43) / 131.0
        gy = self.read_raw(0x45) / 131.0
        gz = self.read_raw(0x47) / 131.0
        return gx, gy, gz

# ----- Ultrasonic Sensor -----
def read_distance(trig, echo):
    trig.off()
    sleep_us(2)
    trig.on()
    sleep_us(10)
    trig.off()
    while echo.value() == 0:
        start = ticks_us()
    while echo.value() == 1:
        end = ticks_us()
    duration = ticks_diff(end, start)
    distance_cm = duration / 58.0
    return distance_cm

# ----- Setup Pins -----

# MPU6050 I2C
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
mpu = MPU6050(i2c)

# Ultrasonic sensor
trig = Pin(19, Pin.OUT)
echo = Pin(23, Pin.IN)

# Button
button = Pin(18, Pin.IN, Pin.PULL_UP)

# Electromagnet
electromagnet = Pin(5, Pin.OUT)
electromagnet.off()

# IR line sensors
ir_pins = [
    Pin(39, Pin.IN),
    Pin(4, Pin.IN),
    Pin(35, Pin.IN),
    Pin(34, Pin.IN),
    Pin(36, Pin.IN)
]

# ----- Encoder Pins and Setup -----
pin_a1 = Pin(25, Pin.IN, Pin.PULL_UP)
pin_b1 = Pin(26, Pin.IN, Pin.PULL_UP)

pin_a2 = Pin(17, Pin.IN, Pin.PULL_UP)
pin_b2 = Pin(16, Pin.IN, Pin.PULL_UP)

position1 = 0
last_state1 = pin_a1.value()

position2 = 0
last_state2 = pin_a2.value()

def update_position1(pin):
    global position1, last_state1
    state = pin_a1.value()
    b_state = pin_b1.value()
    if state != last_state1:
        if b_state != state:
            position1 += 1
        else:
            position1 -= 1
    last_state1 = state

def update_position2(pin):
    global position2, last_state2
    state = pin_a2.value()
    b_state = pin_b2.value()
    if state != last_state2:
        if b_state != state:
            position2 += 1
        else:
            position2 -= 1
    last_state2 = state

pin_a1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_position1)
pin_a2.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_position2)

# ----- Data Reading Loop -----
def read_all():
    while True:
        # MPU6050
        ax, ay, az = mpu.get_accel()
        gx, gy, gz = mpu.get_gyro()

        # Ultrasonic distance
        try:
            dist = read_distance(trig, echo)
        except:
            dist = -1

        # Button state
        button_pressed = button.value() == 1
        
        # Activate electromagnet
        if button.value() == 0:
            electromagnet.off()
        else:
            electromagnet.on()

        # IR line sensor values
        ir_values = [pin.value() for pin in ir_pins]

        # Print all data
        print("\n--- Sensor Readings ---")
        print("Accel: ax={:.2f}, ay={:.2f}, az={:.2f}".format(ax, ay, az))
        print("Gyro : gx={:.2f}, gy={:.2f}, gz={:.2f}".format(gx, gy, gz))
        print("Distance: {:.2f} cm".format(dist) if dist != -1 else "Ultrasonic: Error")
        print("Button Pressed:", button_pressed)
        print("IR Sensors:", ir_values)
        print("Encoder 1 Count:", position1)
        print("Encoder 2 Count:", position2)

        sleep_ms(200)

# Start main loop
read_all()

