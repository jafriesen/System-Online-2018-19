package org.firstinspires.ftc.teamcode._Auto.Steps;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.AutoOpMode;
import org.firstinspires.ftc.teamcode._Libs.HeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;

public class DoStepsStep extends AutoLib.ConcurrentSequence {
    AutoLib.Step steps[];
    int stepTotal, currentStep;

    public DoStepsStep(AutoLib.Step steps[])
    {
        this.steps = steps;
        this.stepTotal = steps.length-1;
        this.currentStep = 0;
    }

    public void setSteps(AutoLib.Step steps[]) {
        this.steps = steps;
        this.stepTotal = steps.length-1;
        this.currentStep = 0;
    }

    @Override
    public boolean loop() {
        if(steps[currentStep].loop()) {
            currentStep++;
        }
        return currentStep > stepTotal;
    }
}