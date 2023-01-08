package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DistanceTest")
public class DistanceTest extends LinearOpMode {
    private DistanceSensor distance;
    private DcMotor motor;
    private Servo claw;
    private Servo finger;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Get the distance sensor and motor from hardwareMap
        distance = hardwareMap.get(DistanceSensor.class, "Distance");
        claw = hardwareMap.get(Servo.class, "claw");
        finger = hardwareMap.get(Servo.class, "finger");

        claw.setPosition(0.2);
        finger.setPosition(0.5);

        // Loop while the Op Mode is running
        telemetry.addData("Status", "OdoMec2 is ready to run!");
        telemetry.update();



        //Wait for press play
        waitForStart();
        runtime.reset();
        telemetry.addData("Status", "OdoMec2 is ready to run!");
        telemetry.update();


        while (opModeIsActive()) {
            // If the distance in centimeters is less than 10, set the power to 0.3
            telemetry.addData("deviceName",distance.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", distance.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", distance.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", distance.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", distance.getDistance(DistanceUnit.INCH)));

            telemetry.update();

            if (distance.getDistance(DistanceUnit.MM) < 40) {
                claw.setPosition(0.42);
                finger.setPosition(0.6);
            } else {  // Otherwise, stop the motor
                claw.setPosition(0.2);
                finger.setPosition(0.5);
            }
        }
    }
}

