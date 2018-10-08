package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.CustomOpMode;
import org.firstinspires.ftc.teamcode._Libs.TrapezoidalMotionProfile;

import static org.firstinspires.ftc.teamcode._Libs.AutoLib.normalize;

class TrapezoidalDriveStep extends AutoLib.Step {
    TrapezoidalMotionProfile mProfiler;
    DcMotorEx mMotors[];
    CustomOpMode mOpMode;
    float mWheelCircumference;

    TrapezoidalDriveStep(CustomOpMode mode, DcMotorEx motors[], float circumference, float inches, float inchesPerSecond, float maxAcceleration) {
        mProfiler = new TrapezoidalMotionProfile(inches, inchesPerSecond, maxAcceleration);
        mWheelCircumference = circumference;
        mOpMode = mode;
        mMotors = motors;
    }

    public boolean loop() {
        super.loop();

        mProfiler.update(mOpMode.time);

        for (DcMotorEx em : mMotors) {
            if (em != null) {
                em.setVelocity(mProfiler.getVelocity() / mWheelCircumference * 2 * Math.PI, AngleUnit.RADIANS);
            }
        }

        mOpMode.packet.put("Velocity", mProfiler.getVelocity());
        mOpMode.packet.put("Position", mProfiler.getPosition());
        mOpMode.packet.put("Elapsed Time", mProfiler.getElapsedTime());

        return false;
    }
}

@Autonomous(name="Test", group="Autonomous")
public class TestAuto extends CustomOpMode {
    DcMotorEx mMotors[];

    @Override
    public void init() {
        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0), // start state
                new MotionState(10, 0, 0), // goal state
                5, // max vel
                5 // max accel
        );

        mHardwareFactory = new AutoLib.RealHardwareFactory(this);
        mMotors = new DcMotorEx[2];

        mMotors[0] = mHardwareFactory.getDcMotorEx("right");
        mMotors[1] = mHardwareFactory.getDcMotorEx("left");

        mMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        mMotors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMotors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mMotors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMotors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        mSequence.add(new TrapezoidalDriveStep(this, mMotors, (float) Robot.WHEEL_CIRCUMFERENCE, (float) Robot.DISTANCE, (float) Robot.MAX_VELOCITY, (float) Robot.MAX_ACCELERATION));

        // start out not-done
        bDone = false;
    }

    @Override
    public void loop() {
        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            packet.put("sequence finished", "");

        packet.put("Encoder 0", mMotors[0].getCurrentPosition());
        packet.put("Encoder 1", mMotors[1].getCurrentPosition());
        packet.put("Speed 0", mMotors[0].getPower());
        packet.put("Speed 1", mMotors[1].getPower());

        mDashboard.sendTelemetryPacket(packet);
    }
}

@Config
class Robot {
    public static double WHEEL_CIRCUMFERENCE = 12.625;
    public static double DISTANCE = 120;
    public static double MAX_VELOCITY = 17.675;
    public static double MAX_ACCELERATION = 6;
}

@Config
class PID {
    public static double P = 20, I = 8, D = 12;
}