package org.firstinspires.ftc.teamcode._Auto.Steps;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.AutoOpMode;
import org.firstinspires.ftc.teamcode._Libs.HeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;

public class ChoosePathStep extends AutoLib.Step {
    AutoLib.Step path1[], path2[], path3[];
    DoStepsStep stepStep;
    AutoLib.Data data;

    public ChoosePathStep(DoStepsStep stepStep, AutoLib.Step path1[], AutoLib.Step path2[], AutoLib.Step path3[], AutoLib.Data data)
    {
        this.stepStep = stepStep;
        this.path1 = path1;
        this.path2 = path2;
        this.path3 = path3;
        this.data = data;
    }

    @Override
    public boolean loop() {
        if(data.Float == 1) {
            stepStep.setSteps(path1);
        }
        else if(data.Float == 3) {
            stepStep.setSteps(path3);
        }
        else {
            stepStep.setSteps(path2);
        }
        return true;
    }
}