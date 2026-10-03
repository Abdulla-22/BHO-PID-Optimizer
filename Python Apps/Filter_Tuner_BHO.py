import serial
import time
import numpy as np
import random

# ===== Configuration =====
PORT = "COM13"  # Your Arduino port
BAUD = 115200
TARGET_RPM = 50.0
TEST_DURATION = 4.0  # Duration per star evaluation
CPR_TOTAL = 8400     # 64 * 131.25

# Fixed PID Gains (Best results from previous test)
KP = 2.0
KI = 0.8
KD = 0.05

# Optimization Settings
N_STARS = 10         # Number of candidates per iteration
MAX_ITER = 5         # Number of iterations
VAR_MIN = 0.01       # Min Alpha
VAR_MAX = 0.50       # Max Alpha

class FilterTunerBHO:
    def __init__(self):
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.1)
            time.sleep(2)
            print(f"Connected to {PORT}")
        except Exception as e:
            print(f"Failed to connect: {e}")
            exit()

    def read_encoder(self):
        try:
            self.ser.write(b"GET\n")
            line = self.ser.readline().decode().strip()
            if line.lstrip('-').isdigit():
                return int(line)
        except:
            pass
        return None

    def evaluate_filter(self, alphas):
        rpm_a, d_a = alphas
        print(f"  Evaluating: rpmAlpha={rpm_a:.4f}, dAlpha={d_a:.4f}...", end=" ", flush=True)
        
        # Stop motor and wait
        self.ser.write(b"PWM:1,0\n")
        time.sleep(1.0)
        
        start_t = time.perf_counter()
        prev_t = start_t
        last_enc = self.read_encoder() or 0
        
        rpm_filt = 0.0
        prev_rpm_filt = 0.0
        d_filt = 0.0
        integral = 0.0
        
        rpms = []
        pwms = []
        
        while (time.perf_counter() - start_t) < TEST_DURATION:
            loop_start = time.perf_counter()
            
            cur_enc = self.read_encoder()
            if cur_enc is None: continue
            
            now = time.perf_counter()
            dt = now - prev_t
            if dt < 0.01: continue
            
            # RPM calculation
            delta = cur_enc - last_enc
            raw_rpm = abs((delta * 60.0) / (CPR_TOTAL * dt))
            
            # Filter
            rpm_filt = rpm_a * raw_rpm + (1.0 - rpm_a) * rpm_filt
            
            # PID logic
            err = TARGET_RPM - rpm_filt
            d_meas = (rpm_filt - prev_rpm_filt) / dt
            d_filt = d_a * d_meas + (1.0 - d_a) * d_filt
            
            integral += err * dt
            integral = np.clip(integral, -50, 50)
            
            pwm_ff = 35 + (TARGET_RPM / 130.0) * (255 - 35)
            corr = KP * err + KI * integral - KD * d_filt
            pwm = int(np.clip(pwm_ff + corr, 0, 255))
            
            self.ser.write(f"PWM:1,{pwm}\n".encode())
            
            rpms.append(rpm_filt)
            pwms.append(pwm)
            
            prev_t = now
            last_enc = cur_enc
            prev_rpm_filt = rpm_filt
            
            # Sleep to match ~30ms
            elapsed = time.perf_counter() - loop_start
            if elapsed < 0.03:
                time.sleep(0.03 - elapsed)

        self.ser.write(b"PWM:1,0\n")
        
        # Cost Calculation (Steady State)
        if len(rpms) < 20: return 9999.0
        ss = rpms[int(len(rpms)*0.7):]
        ss_pwm = pwms[int(len(pwms)*0.7):]
        
        # Penalize noise (std) and jitter (pwm changes)
        noise = np.std(ss)
        jitter = np.mean(np.abs(np.diff(ss_pwm)))
        error = np.mean(np.abs(np.array(ss) - TARGET_RPM))
        
        cost = (noise * 1.5) + (jitter * 1.0) + (error * 0.5)
        print(f"Cost: {cost:.4f}")
        return cost

    def run_bho(self):
        print(f"\n--- Starting Black Hole Optimization for Filters ---")
        
        # Initialize Stars
        stars = np.random.uniform(VAR_MIN, VAR_MAX, (N_STARS, 2))
        costs = np.array([self.evaluate_filter(s) for s in stars])
        
        for it in range(MAX_ITER):
            # Best is the Black Hole
            bh_idx = np.argmin(costs)
            bh_star = stars[bh_idx].copy()
            bh_cost = costs[bh_idx]
            
            print(f"\nIteration {it+1}/{MAX_ITER} | Best so far: rAlpha={bh_star[0]:.4f}, dAlpha={bh_star[1]:.4f} | Cost: {bh_cost:.4f}")
            
            # Move Stars towards Black Hole
            for i in range(N_STARS):
                if i == bh_idx: continue
                
                # Movement equation
                stars[i] += np.random.rand(2) * (bh_star - stars[i])
                stars[i] = np.clip(stars[i], VAR_MIN, VAR_MAX)
                
                # Evaluate new position
                costs[i] = self.evaluate_filter(stars[i])
            
            # Event Horizon check
            radius = bh_cost / (np.sum(costs) + 1e-9)
            for i in range(N_STARS):
                if i == bh_idx: continue
                dist = np.linalg.norm(stars[i] - bh_star)
                if dist < radius:
                    # Swallow and regenerate
                    stars[i] = np.random.uniform(VAR_MIN, VAR_MAX, 2)
                    costs[i] = self.evaluate_filter(stars[i])

        # Final Result
        best_idx = np.argmin(costs)
        final_bh = stars[best_idx]
        print("\n" + "!"*40)
        print("BHO OPTIMIZATION COMPLETE")
        print(f"Optimal rpmAlpha: {final_bh[0]:.6f}")
        print(f"Optimal dAlpha:   {final_bh[1]:.6f}")
        print(f"Final Cost:       {costs[best_idx]:.4f}")
        print("!"*40)

if __name__ == "__main__":
    tuner = FilterTunerBHO()
    tuner.run_bho()
