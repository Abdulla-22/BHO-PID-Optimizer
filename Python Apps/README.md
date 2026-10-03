# Black Hole Optimizer - Installation Guide

## Requirements

Before installing and running the project, make sure you have:

- Python 3.10 or newer
- Windows operating system
- Internet connection for downloading libraries

---

## Project Structure

Place the project files in the following structure:

```text
D:\Python\
│
├── env\
├── Launcher.py
├── BHO.py
├── Controlling_DC_Motor.py
└── images\
```

The `images` folder should contain all visualization GIF and PNG files.

---

## Step 1 - Open PowerShell

Open PowerShell inside the project folder.

Example:

```powershell
cd D:\Python
```

---

## Step 2 - Create Virtual Environment

Run the following command:

```powershell
python -m venv env
```

This will create a virtual environment folder named `env`.

---

## Step 3 - Activate Virtual Environment

Run:

```powershell
.\env\Scripts\Activate.ps1
```

If PowerShell blocks script execution, run:

```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

Then activate the environment again:

```powershell
.\env\Scripts\Activate.ps1
```

After activation, you should see:

```text
(env)
```

at the beginning of the terminal line.

---

## Step 4 - Upgrade pip

Run:

```powershell
python -m pip install --upgrade pip
```

---

## Step 5 - Install Required Libraries

Run:

```powershell
pip install customtkinter numpy scipy matplotlib pandas openpyxl pillow pyserial
```

---

## Step 6 - Verify Installation

Run:

```powershell
python -c "import customtkinter, numpy, scipy, matplotlib, pandas, openpyxl, PIL, serial; print('All libraries installed successfully')"
```

If no errors appear, all libraries are installed correctly.

---

## Step 7 - Run the Application

Run:

```powershell
python Launcher.py
```

---

## Notes

1. Do not rename the `images` folder.

2. Keep all GIF and PNG visualization files inside the `images` folder.

3. The application uses:

   - **CustomTkinter** for GUI
   - **NumPy** and **SciPy** for calculations
   - **Matplotlib** for plotting
   - **Pandas/OpenPyXL** for saving results
   - **Pillow** for image and GIF handling
   - **PySerial** for Arduino communication

4. If the application becomes slow:

   - Reduce `Population size`
   - Reduce `Max Iterations`

5. Recommended settings:

```text
Population = 30
Max Iterations = 50
```

---

## End of Instructions