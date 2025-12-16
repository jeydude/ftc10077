# FTC Robot Code Structure

This repository is organized using a **subsystem-based architecture** to keep code modular, readable, and easy to maintain. Each major robot function is separated into its own class.

---

## 📁 Package Structure
org.firstinspires.ftc.teamcode
│
├── opmodes
│   └── AutonomousOpMode.java
│
├── subsystems
│   ├── DriveBase.java
│   ├── Intake.java
│   ├── Shooter.java
│   └── Kicker.java
│
└── util
    └── RobotConstants.java

---

## 📌 Folder Breakdown

### 🔹 `opmodes/`
Contains FTC OpModes (Autonomous and TeleOp).

- **`AutonomousOpMode.java`**  
  Main autonomous routine.  
  Coordinates subsystems to perform pre-programmed actions such as driving, collecting, and shooting.

---

### 🔹 `subsystems/`
Each class represents a **single robot mechanism**.

- **`DriveBase.java`**  
  Handles mecanum drive logic, motor control, and movement methods.

- **`Intake.java`**  
  Controls the intake motor for collecting or ejecting game elements.

- **`Shooter.java`**  
  Manages shooter motors, RPM control, and voltage compensation.

- **`Kicker.java`**  
  Controls the servo used to feed or kick game elements into the shooter.

---

### 🔹 `util/`
Utility and configuration classes.

- **`RobotConstants.java`**  
  Centralized constants such as:
  - Motor names
  - Encoder values
  - Servo positions
  - Shooter RPM targets
  - Tuning parameters

---

## ✅ Benefits of This Structure

- ✔ Clean separation of logic
- ✔ Easier debugging and tuning
- ✔ Reusable subsystems across OpModes
- ✔ Competition-ready and scalable

---

## 🚀 Notes
- TeleOp and Autonomous OpModes should **only coordinate subsystems**, not directly control motors.
- All hardware access should live inside the subsystem classes.
- Constants should never be hard-coded inside OpModes.

---

Happy coding and good luck at competition! 🏆
