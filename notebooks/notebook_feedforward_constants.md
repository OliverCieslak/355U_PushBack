# Feedforward Constants: kS, kV, and kA

## What are Feedforward Constants?
Feedforward constants are numbers we use in our robot code to help the drivetrain move smoothly and accurately. They let us predict how much voltage (power) we need to give the motors to get the robot to move at a certain speed or accelerate at a certain rate. This makes our robot's movement more precise and easier to control.

## kS: Static Friction Constant
- **What it means:** kS is the amount of voltage needed just to get the robot moving from a stop. This is because the robot has to overcome friction before it can start rolling.
- **How to think about it:** Imagine trying to push a heavy box. You have to push harder to get it started than to keep it moving. kS is like that first push.
- **In code:** We add kS to our motor commands so the robot doesn't "stick" when we want it to move.

## kV: Velocity Constant
- **What it means:** kV tells us how much voltage we need to keep the robot moving at a certain speed. It's like a conversion factor between speed and voltage.
- **How to think about it:** If you want the robot to go faster, you need to give it more power. kV helps us figure out exactly how much more.
- **In code:** We multiply kV by the speed we want, so the robot gets just the right amount of power to reach and hold that speed.

## kA: Acceleration Constant
- **What it means:** kA is the voltage needed to make the robot speed up (accelerate) or slow down (decelerate) at a certain rate.
- **How to think about it:** If you want the robot to go from stopped to full speed quickly, you need extra power to accelerate. kA tells us how much extra voltage to add when changing speed.
- **In code:** We multiply kA by the acceleration we want, so the robot can speed up or slow down smoothly and predictably.

## Why Do We Use These?
By tuning kS, kV, and kA, we make our robot's movements more accurate and consistent. This is especially important for autonomous routines, where the robot needs to follow paths and stop at precise locations. Using these constants, our code can "feed forward" the right amount of power, making our robot more competitive and reliable on the field.
