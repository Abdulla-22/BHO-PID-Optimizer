import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# ===== Configuration =====
PORT = "COM13"  # Change to your Arduino port
BAUD = 115200
TARGET_RPM = 50.0
TEST_DURATION = 4.0  # Seconds per test
CPR_TOTAL = 8400     # 64 * 131.25

# Fixed PID Gains (The ones you found good)
KP = 2.0
KI = 0.8
KD = 0.05

# Search Space for Alphas
RPM_ALPHAS = [0.05, 0.1, 0.15, 0.2, 0.3]
D_ALPHAS   = [0.01, 0.05, 0.1, 0.15]

class FilterTuner:
    def __init__(self):
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.1)
            time.sleep(2)
            print(f"Connected to {PORT}")
        except:
            print(f"Failed to connect to {PORT}")
            exit()

    def read_encoder(self):
        self.ser.write(b"GET\n")
        line = self.ser.readline().decode().strip()
        if line.lstrip('-').isdigit():
            return int(line)
        return None

    def run_test(self, rpm_a, d_a):
        print(f"Testing: rpmAlpha={rpm_a}, dAlpha={d_a}...", end=" ", flush=True)
        
        # Reset state
        self.ser.write(b"PWM:1,0\n")
        time.sleep(1)
        
        start_time = time.perf_counter()
        prev_time = start_time
        last_enc = self.read_encoder() or 0
        
        # Control vars
        rpm_filt = 0.0
        prev_rpm_filt = 0.0
        d_filt = 0.0
        integral = 0.0
        
        history_rpm = []
        history_pwm = []
        history_t = []
        
        while (time.perf_counter() - start_time) < TEST_DURATION:
            loop_start = time.perf_counter()
            
            # 1. Read
            cur_enc = self.read_encoder()
            if cur_enc is None: continue
            
            now = time.perf_counter()
            dt = now - prev_time
            if dt < 0.005: continue
            
            # 2. Calc RPM
            delta = cur_enc - last_enc
            raw_rpm = abs((delta * 60.0) / (CPR_TOTAL * dt))
            
            # Apply Filter
            rpm_filt = rpm_a * raw_rpm + (1.0 - rpm_a) * rpm_filt
            
            # 3. PID
            err = TARGET_RPM - rpm_filt
            
            # Derivative
            d_meas = (rpm_filt - prev_rpm_filt) / dt
            d_filt = d_a * d_meas + (1.0 - d_a) * d_filt
            
            # Integral
            integral += err * dt
            integral = max(-50, min(50, integral))
            
            # Correction
            pwm_ff = 35 + (TARGET_RPM / 130.0) * (255 - 35)
            corr = KP * err + KI * integral - KD * d_filt
            pwm = int(np.clip(pwm_ff + corr, 0, 255))
            
            # Write
            self.ser.write(f"PWM:1,{pwm}\n".encode())
            
            # Log
            history_rpm.append(rpm_filt)
            history_pwm.append(pwm)
            history_t.append(now - start_time)
            
            # Cycle
            prev_time = now
            last_enc = cur_enc
            prev_rpm_filt = rpm_filt
            
            # Sync to ~30ms
            elapsed = time.perf_counter() - loop_start
            if elapsed < 0.03:
                time.sleep(0.03 - elapsed)
        
        self.ser.write(b"PWM:1,0\n")
        print("Done.")
        
        return self.calculate_cost(history_t, history_rpm, history_pwm)

    def calculate_cost(self, t, rpm, pwm):
        if len(rpm) < 10: return 9999
        
        # Analyze Steady State (Last 1 second)
        ss_start_idx = int(len(rpm) * 0.75)
        ss_rpm = rpm[ss_start_idx:]
        ss_pwm = pwm[ss_start_idx:]
        
        # 1. Noise (Standard Deviation of RPM) - Goal: 0
        noise_rpm = np.std(ss_rpm)
        
        # 2. PWM Jitter (Standard Deviation of PWM changes) - Goal: 0
        jitter_pwm = np.mean(np.abs(np.diff(ss_pwm)))
        
        # 3. Settling Error (MAE from target)
        error_ss = np.mean(np.abs(np.array(ss_rpm) - TARGET_RPM))
        
        # Total Cost (Weighted)
        # We care most about Noise and Jitter
        total_cost = (noise_rpm * 1.0) + (jitter_pwm * 0.5) + (error_ss * 0.5)
        return total_cost

    def optimize(self):
        best_cost = 9999
        best_params = (None, None)
        
        results = []
        
        for ra in RPM_ALPHAS:
            for da in D_ALPHAS:
                cost = self.run_test(ra, da)
                results.append((ra, da, cost))
                if cost < best_cost:
                    best_cost = cost
                    best_params = (ra, da)
        
        print("\n" + "="*30)
        print("OPTIMIZATION FINISHED")
        print(f"Best rpmAlpha: {best_params[0]}")
        print(f"Best dAlpha:   {best_params[1]}")
        print(f"Best Cost:     {best_cost:.4f}")
        print("="*30)
        
        return results

if __name__ == "__main__":
    tuner = FilterTuner()
    tuner.optimize()
