package org.firstinspires.ftc.teamcode._Samples;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.TrackWidthCalibrationOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._Robot.RobotDrive;

@Autonomous
public class SampleTrackWidthCalibrationOpMode extends TrackWidthCalibrationOpMode {
    @Override
    protected Drive initDrive() {
        return new RobotDrive(hardwareMap);
    }

    @Override
    protected BNO055IMU initIMU() {
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        return imu;
    }
}
