package org.firstinspires.ftc.teamcode._Libs;

import java.util.ArrayList;

public class RoadRunnerImplementer {
    static public class FollowTrajectory extends AutoLib.MotorGuideStep {
        private float mPower;                               // basic power setting of all 4 motors -- adjusted for steering along path
        private float mMaxPower = 1.0f;                     // maximum power for any motor
        private float mHeading;                             // compass heading to steer for (-180 .. +180 degrees)
        private CustomOpMode mOpMode;                             // needed so we can log output (may be null)
        private HeadingSensor mGyro;                        // sensor to use for heading information (e.g. Gyro or Vuforia)
        private SensorLib.PID mPid;                         // proportional–integral–derivative controller (PID controller)
        private double mPrevTime;                           // time of previous loop() call
        private ArrayList<AutoLib.SetPower> mMotorSteps;            // the motor steps we're guiding - assumed order is right ... left ...

        public FollowTrajectory(CustomOpMode mode, float heading, HeadingSensor gyro, SensorLib.PID pid,
                             ArrayList<AutoLib.SetPower> motorsteps, float power) {
            mOpMode = mode;
            mHeading = heading;
            mGyro = gyro;
            mPid = pid;
            mMotorSteps = motorsteps;
            mPower = power;
        }

        public boolean loop()
        {
            super.loop();

            return true;
        }

        // set motor control steps this step should control (assumes ctor called with null argument)
        public void set(ArrayList<AutoLib.SetPower> motorsteps)
        {
            mMotorSteps = motorsteps;
        }

        // update target direction, heading, and power --
        // used by interactive teleop modes to redirect the step from controller input
        public void set(float heading,float power) {
            mHeading = heading;
            mPower = power;
        }

    }
}
