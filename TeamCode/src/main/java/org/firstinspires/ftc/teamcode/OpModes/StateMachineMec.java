package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Cycloptic Turtle")


public class StateMachineMec extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //Mechanism motors
    private DcMotor turret;
    private DcMotor leftSlide;
    private DcMotor rightSlide;

    //Servos
    private Servo odoRetractor;
    private static final double ODO_RETRACT = 1; //.95
    private Servo flipOut;
    private static final double FLIPPED_IN = .50; //.95
    private static final double FLIPPED_OUT = .02;
    private Servo claw;
    private static final double CLAW_CLOSED = 0.60;
    private static final double CLAW_OPEN = 0.05;
    private Servo clawLinkage;
    private static final double CLAW_LINKAGE_FIVE = 0.5;
    private static final double CLAW_LINKAGE_FOUR = 0.54;
    private static final double CLAW_LINKAGE_THREE = 0.62;
    private static final double CLAW_LINKAGE_TWO = 0.7;
    private static final double CLAW_LINKAGE_ONE = 0.78;
    private Servo brake;
    private static final double BRAKE_OFF = 0.16;
    private static final double BRAKE_ON = 0.28;

    private static final double POWER_FULL = 1;

    private static int lastDirection;

    private static boolean flipper_pos;

    //Magnetic Switches
    private RevTouchSensor slideMag;

    //Distance sensor
    private ColorRangeSensor distance;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Mechanism motors
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Servos
        odoRetractor = hardwareMap.get(Servo.class, "odoRetractor");
        flipOut = hardwareMap.get(Servo.class, "flipOut");
        claw = hardwareMap.get(Servo.class, "claw");
        clawLinkage = hardwareMap.get(Servo.class, "clawLinkage");
        brake = hardwareMap.get(Servo.class, "brake");

        //Magnetic switches
        slideMag = hardwareMap.get(RevTouchSensor.class, "slideMag");

        //Distance sensor
        distance = hardwareMap.get(ColorRangeSensor.class, "Distance");

        //Set servo positions
        odoRetractor.setPosition(ODO_RETRACT);
        claw.setPosition(CLAW_CLOSED);
        flipOut.setPosition(FLIPPED_IN);
        flipper_pos = true;
        clawLinkage.setPosition(CLAW_LINKAGE_FIVE);
        brake.setPosition(BRAKE_OFF);

        // Set Turret Motor Power
        turret.setPower(0);

        telemetry.addData("Status", "Cycloptic Turtle is ready to run!");
        telemetry.update();

        //Wait for press play
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            //Mechanisms
            // If the distance is < 40mm close claw
            telemetry.addData("deviceName",distance.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", distance.getDistance(DistanceUnit.MM)));

            telemetry.update();

            // Lift
            double motorVerticalLiftRightPower = -gamepad2.left_stick_y;
            double motorVerticalLiftLeftPower = -gamepad2.left_stick_y;

            if (!slideMag.isPressed()) { // SLIDE_UP
                if (gamepad2.dpad_up) { // Lift to top cone stack
                    clawLinkage.setPosition(CLAW_LINKAGE_FIVE); // 0.53 - 5 cones, ALl the way up
                    lift(75,1);

                } else if (gamepad2.dpad_left) { // Lift to small junction
                    lift(240,1);
                } else if (gamepad2.dpad_down) { // Lift to med junction
                    lift(535,1);
                }  else if (gamepad2.dpad_right) { // Lift to tall junction
                    lift(980,1);
                } else {
                    // SLIDE_UP with left joy stick
                    motorVerticalLiftRightPower = Range.clip(motorVerticalLiftRightPower, -0.6, 1); // was 1
                    motorVerticalLiftLeftPower = Range.clip(motorVerticalLiftLeftPower, -0.6, 1); //-0.90, .93)
                    rightSlide.setPower(motorVerticalLiftRightPower);
                    leftSlide.setPower(motorVerticalLiftLeftPower);
                }
            } else { // SLIDE_DOWN
                motorVerticalLiftRightPower = Range.clip(motorVerticalLiftRightPower, 0, 1);// 1
                motorVerticalLiftLeftPower = Range.clip(motorVerticalLiftLeftPower, 0, 1);// .93
                rightSlide.setPower(motorVerticalLiftRightPower);
                leftSlide.setPower(motorVerticalLiftLeftPower);
            }

            // Turret Brake
            if (gamepad2.left_trigger == 1) {
                brake.setPosition(BRAKE_ON);
            } else {
                brake.setPosition(BRAKE_OFF);
            }

            // Claw Linkage Lift
            if (gamepad1.left_bumper) {
                clawLinkage.setPosition(CLAW_LINKAGE_FIVE); // 0.53 - 5 cones, ALl the way up
            } else if (gamepad1.right_bumper) {
                clawLinkage.setPosition(CLAW_LINKAGE_ONE); // 0.87 - 1 cone, ALl the way down
            }

            if (gamepad2.right_bumper && gamepad2.y) { // 0.63 - 4 cones
                clawLinkage.setPosition(CLAW_LINKAGE_FOUR);

            } else if (gamepad2.right_bumper && gamepad2.b) { // 0.7 - 3 cones
                clawLinkage.setPosition(CLAW_LINKAGE_THREE);
            } else if (gamepad2.right_bumper && gamepad2.a) { // 0.7 - 2 cones
                clawLinkage.setPosition(CLAW_LINKAGE_TWO);
            } else if (gamepad2.right_bumper && gamepad2.x) { // 0.87 1 cone, All the way downLift
                clawLinkage.setPosition(CLAW_LINKAGE_ONE);
            } else if (gamepad2.left_bumper) {
                clawLinkage.setPosition(CLAW_LINKAGE_FIVE); // 0.53 - 5 cones, ALl the way up
            }

            telemetry.addData("Servo",clawLinkage.getPosition());
            telemetry.addData("TurretPos", turret.getCurrentPosition());
            telemetry.update();

            // Odometry retractor mechanism
            if (gamepad1.a) {
                odoRetractor.setPosition(0.7);
            }

            // Flip in Flip out grabber mechanism
            if (gamepad2.x && !gamepad2.right_bumper){
                flipOut.setPosition(FLIPPED_OUT);
                flipper_pos = false;
                sleep(200);
                claw.setPosition(CLAW_OPEN);
            } else if (gamepad2.a && !gamepad2.right_bumper) {
                claw.setPosition(CLAW_CLOSED);
                flipOut.setPosition(FLIPPED_IN);
                flipper_pos = true;
            }

            // Auto/Manual Claw

            if (distance.getDistance(DistanceUnit.MM) < 40 && (!gamepad2.y || gamepad2.right_trigger > 0) ) {
                claw.setPosition(CLAW_CLOSED);

            } else if (!gamepad2.right_bumper && gamepad2.y)  {  // Otherwise, stop the motor
                claw.setPosition(CLAW_OPEN);
                sleep(100);
            } else if (gamepad2.right_trigger > 0) {
                claw.setPosition(CLAW_CLOSED);

            } else if (!flipper_pos) {
                claw.setPosition(CLAW_OPEN);

            } else {
                claw.setPosition(CLAW_CLOSED);
            }

            //turret
            double motorTurretPower = -gamepad2.right_stick_x;

            if (gamepad2.right_stick_x > 0) {                   // + 0.1
                turret.setPower(Range.clip(gamepad2.right_stick_x, 0, 0.35));
                lastDirection = 1;
            } else if (gamepad2.right_stick_x < 0) {
                lastDirection = -1;                               // - 0.1
                turret.setPower(Range.clip(gamepad2.right_stick_x, -0.35, 0));
            } else {
                turret.setPower(0);
            }
            telemetry.update();
            idle();

        }


    }

    public void lift(double speed, int distance) {

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setTargetPosition(distance);
        rightSlide.setTargetPosition(distance);


        leftSlide.setPower(speed);
        rightSlide.setPower(speed);

    }
}