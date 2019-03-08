package org.firstinspires.ftc.teamcode._Auto.Steps;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;


public class UnlatchStep extends AutoLib.Step {
    DcMotor mEncoderMotor;    // motor to control
    double mPower;          // power level to use
    float mEncoderCount;      // target encoder count
    OpMode opMode;
    boolean mStop;          // stop motor when count is reached
    AutoLib.Timer mTimer;

    public UnlatchStep(OpMode opMode, DcMotor encoderMotor, double power, float count) {
        mEncoderMotor = encoderMotor;
        mPower = power;
        mEncoderCount = count;
        this.opMode = opMode;
    }

    public boolean loop() {
        super.loop();

        boolean done = false;

        if(firstLoopCall()) {
            mTimer = new AutoLib.Timer(1.7);
            mTimer.start();
        }

        mEncoderMotor.setPower(mPower);

        opMode.telemetry.addData("encoder count", mEncoderMotor.getCurrentPosition());
        opMode.telemetry.addData("power", mPower);
        opMode.telemetry.addData("motor 1 power", mEncoderMotor.getPower());
        opMode.telemetry.addData("time", mTimer.elapsed());



        // the rest of the time, just update power and check to see if we're done
       // done = Math.abs(mEncoderMotor.getCurrentPosition()) > Math.abs(mEncoderCount);
        if(mTimer.done()) {
            mEncoderMotor.setPower(0);
            done = true;
        }
        return done;
    }
}