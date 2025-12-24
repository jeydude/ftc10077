# Enable Full 3D in Limelight (FTC)

This guide explains how to enable **Full 3D pose estimation** on a Limelight camera for FTC use (AprilTags, botpose, field-centric driving).

---

## 🔧 Step-by-Step: Enable Full 3D in Limelight

### 1️⃣ Connect to the Limelight Web UI

1. Connect your **Limelight** to your laptop using a **USB cable**.
2. Open a web browser and navigate to: 
http://limelight.local:5801


> If this does not load, use the **Limelight Hardware Manager** application to discover the device.

📘 Reference: Limelight Documentation

---

### 2️⃣ Select Your Pipeline

1. In the Limelight web interface, choose the **vision pipeline** you want to use.
2. For FTC localization, select an **AprilTag** pipeline.

---

### 3️⃣ Open the Advanced Tab

1. Inside the selected pipeline settings, locate and open the **Advanced** tab.

---

### 4️⃣ Enable Full 3D

1. Find the option labeled **“Full 3D Targeting”**, **“Full 3D Tracking”**, or similar.
2. Toggle the option **ON**.

✅ This enables Limelight to compute **full 3D pose data** (X, Y, Z, and rotation) instead of only 2D angles.

📘 Reference: Limelight Documentation

---

### 5️⃣ Calibrate Camera Pose (Required for Accuracy)

1. In the same pipeline settings, locate the **Camera Pose** or **Robot Transform** section.
2. Enter the camera’s **position and rotation relative to the robot frame**:
- Forward/Back offset
- Left/Right offset
- Up/Down height
- Roll, Pitch, and Yaw

⚠️ Accurate values are critical for correct 3D pose estimation.

📘 Reference: Limelight Documentation

---

### 6️⃣ Use the 3D Visualizer

1. Open the **3D Visualizer** tab in the Limelight web UI.
2. Confirm:
- AprilTags appear in correct positions
- Robot pose updates correctly as the robot moves

This is the best way to validate your Full 3D configuration before using it in code.

📘 Reference: Limelight Documentation

---

## ✅ Result

Once configured, Limelight will output:
- `botpose` (robot pose on the field)
- Full **3D position and orientation**
- Accurate data for **field-centric driving and autonomous navigation**

You are now ready to use Limelight Full 3D in FTC code.
