import subprocess
import sys
import os

def launch_apps():
    print("Launching Black Hole Optimizer...")
    # Open BHO.py
    subprocess.Popen([sys.executable, "BHO.py"])
    
    print("Launching Controlling DC Motor...")
    # Open Controlling_DC_Motor.py
    subprocess.Popen([sys.executable, "Controlling_DC_Motor.py"])
    
    print("Both apps are running!")

if __name__ == "__main__":
    launch_apps()
