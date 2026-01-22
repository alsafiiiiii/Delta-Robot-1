# Optimized ESP32 Servo Controller

This firmware is tuned for **High-Performance Delta Robot Control**. It differs significantly from the standard ESP-IDF example.

## Key Optimizations

| Feature | Standard | Optimized (This Firmware) | Benefit |
|---------|----------|---------------------------|---------|
| **PWM Frequency** | 50Hz (20ms) | **200Hz (5ms)** | 4x faster response, 75% less latency. |
| **Smoothing** | 0.9 (Heavy) | **0.15 (Light)** | Minimal lag. Relies on Python for trajectory planning. |
| **Resolution** | 7µs (Quantized) | **1µs (Full)** | Smooth, step-free motion. |
| **Control Rate** | Varies | **200Hz Fixed** | Synchronized with PWM output. |

## Configuration

All critical settings are defined at the top of `main/mcpwm_servo_control_example_main.c`:

```c
#define SERVO_FREQ_HZ 200      // PWM Frequency
#define SMOOTHING_FACTOR 0.15f // EMA Filter
#define DEADBAND_US 4.0f       // Jitter prevention
```

## Protocol

The firmware expects a serial string at **115200 baud**:

```text
A,<servo1_us>,<servo2_us>,<servo3_us>\n
```

Example: `A,1500.5,1600.2,1450.0`

## Flashing

```bash
idf.py build flash monitor
```
