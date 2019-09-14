package org.firstinspires.ftc.teamcode.relic2017;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import evlib.hardware.sensors.Gyro;
import evlib.hardware.sensors.IMUGyro;


/**
 * Created by ftc7393 on 12/9/2017.
 */
@Autonomous(name = "TestOp", group = "Sensor")
@Disabled
public class TestOp extends OpMode {
    Gyro gyro;

    @Override
    public void init() {

        gyro=new IMUGyro( hardwareMap.get(BNO055IMU.class, "imu"));
    }

    @Override
    public void loop() {
//        gyro.update();
        telemetry.addData("Heading",gyro.getHeading());

    }
}
