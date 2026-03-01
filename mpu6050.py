import time, math
import board, busio
import adafruit_mpu6050

# ─────────── I2C + MPU6050 Setup ───────────
i2c = busio.I2C(scl=board.GP13, sda=board.GP12)
mpu = adafruit_mpu6050.MPU6050(i2c)

# ─────────── Detection Thresholds ───────────
ACCEL_THRESHOLD     = 25.0      # m/s² (≈ 2.5G) for strong impact
GYRO_THRESHOLD_DEG  = 500.0     # °/s for sudden rotation
ZERO_ACCEL_TOL      = 0.20      # m/s² — threshold to consider "no movement"
ZERO_ACCEL_TIME_S   = 0.5       # time before resetting speed
CRASH_HOLD_TIME     = 3.0       # time to wait before rearming

# ─────────── Calibration ───────────
def calibrate_gyro(samples=100, delay=0.01):
    print("📏 Calibrating Gyroscope...")
    gx_sum = gy_sum = gz_sum = 0.0
    for _ in range(samples):
        gx, gy, gz = mpu.gyro
        gx_sum += gx
        gy_sum += gy
        gz_sum += gz
        time.sleep(delay)
    return gx_sum / samples, gy_sum / samples, gz_sum / samples

def calibrate_accel(samples=100, delay=0.01):
    print("📏 Calibrating Accelerometer...")
    ax_sum = ay_sum = az_sum = 0.0
    for _ in range(samples):
        ax, ay, az = mpu.acceleration
        ax_sum += ax
        ay_sum += ay
        az_sum += az
        time.sleep(delay)
    return ax_sum / samples, ay_sum / samples, az_sum / samples

# ─────────── Start Calibration ───────────
gxo, gyo, gzo = calibrate_gyro()
gax, gay, gaz = calibrate_accel()
print("✔ Calibration complete. System ready.\n")

# ─────────── Runtime State Variables ───────────
vx = vy = 0.0
stationary_timer = 0.0
last_time = time.monotonic()

# ─────────── Main Loop ───────────
while True:
    now = time.monotonic()
    dt = now - last_time
    last_time = now
    if dt == 0:
        continue

    # ─ Sensor Readings ─
    ax, ay, az = mpu.acceleration
    gx, gy, gz = mpu.gyro

    # ─ Remove Calibration Offsets ─
    gx -= gxo
    gy -= gyo
    gz -= gzo
    ax -= gax
    ay -= gay

    # ─ Calculate Gyro Speed in °/s ─
    gyro_total = math.sqrt(gx**2 + gy**2 + gz**2)
    gyro_total_deg = math.degrees(gyro_total)

    # ─ Estimate Horizontal Speed from Acceleration ─
    horiz_a_mag = math.sqrt(ax**2 + ay**2)
    vx += ax * dt
    vy += ay * dt
    speed = math.sqrt(vx**2 + vy**2)

    # ─ Reset Speed if Helmet is Stationary ─
    if horiz_a_mag < ZERO_ACCEL_TOL:
        stationary_timer += dt
        if stationary_timer >= ZERO_ACCEL_TIME_S:
            vx = vy = 0.0
            stationary_timer = 0.0
    else:
        stationary_timer = 0.0

    # ─ Detect Crash ─
    accel_total = math.sqrt((ax + gax)**2 + (ay + gay)**2 + (az + gaz)**2)
    if accel_total > ACCEL_THRESHOLD or gyro_total_deg > GYRO_THRESHOLD_DEG:
        print("🚨 Crash Detected!")
        print("Impact Force: %.1f m/s² | Rotation: %.1f °/s" % (accel_total, gyro_total_deg))
        time.sleep(CRASH_HOLD_TIME)  # pause before allowing another trigger

    # ─ Optional: Debug Output ─
    print("Accel=%.2f m/s² | Speed=%.2f m/s | Gyro=%.1f °/s" %
          (horiz_a_mag, speed, gyro_total_deg))

    time.sleep(0.05)
