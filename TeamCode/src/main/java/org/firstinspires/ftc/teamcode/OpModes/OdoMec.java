package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.DistanceSensor;

//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Mike Wazowski")


public class OdoMec extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //Motors
    //Drivetrain
    private DcMotor rightFront; //front right 0
    private DcMotor leftFront; //front left 2
    private DcMotor rightRear; //rear right 1
    private DcMotor leftRear; //rear left 3

    //Encoders
    private DcMotor verticalRight; // 1
    private DcMotor verticalLeft; // 3
    private DcMotor horizontal; // 0

    //Mechanism motors
    private DcMotor turret;
    private DcMotor leftSlide;
    private DcMotor rightSlide;

    //Servos
    private Servo odoRetractor;
    private static final double ODO_RETRACT = 1; //.95
    private static final double ODO_DOWN = .25;
    private Servo flipOut;
    private static final double FLIPPED_IN = .29; //.95
    private static final double FLIPPED_OUT = .62;
//    private Servo finger;
//    private static final double FINGER_UP = 0.55;
//    private static final double FINGER_DOWN = 0.85;
    private Servo claw;
    private static final double CLAW_CLOSED = 0.55;
    private static final double CLAW_OPEN = 0.05;
   // private static final double CLAW_CLOSED = 0.4;
   // private static final double CLAW_OPEN = 0.95;
    private Servo clawLinkage;
    private static final double CLAW_LINKAGE_FIVE = 0.5;
    private static final double CLAW_LINKAGE_FOUR = 0.57;
    private static final double CLAW_LINKAGE_THREE = 0.63;
    private static final double CLAW_LINKAGE_TWO = 0.7;
    private static final double CLAW_LINKAGE_ONE = 0.78;
    private Servo brake;
    private static final double BRAKE_OFF = 0.16;
    private static final double BRAKE_ON = 0.28;

    private static final double POWER_FULL = 1;
    private static final double POWER_NINETY = 0.9;
    private static final double POWER_EIGHTY = 0.8;
    private static final double POWER_SEVENTY = 0.7;
    private static final double POWER_SIXTY = 0.6;
    private static final double POWER_FIFTY = 0.5;
    private static final double POWER_FORTY = 0.4;
    private static final double POWER_THIRTY = 0.3;
    private static final double POWER_TWENTY = 0.2;
    private static final double POWER_TEN = 0.1;
    private static final double POWER_FIVE = 0.05;
    private static final double LIFT_DOWN = -0.3;

    private static int lastDirection;

    private static boolean flipper_pos;

    //Magnetic Switches
    private RevTouchSensor slideMag;
//    private RevTouchSensor homeMag;

    //Distance sensor
    private ColorRangeSensor distance;

    BNO055IMU imu;                // Additional Gyro device
    Orientation angles;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables
        //Drive motors
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");

        //Encoders
        verticalRight = hardwareMap.get(DcMotor.class, "rightRear");
        verticalLeft = hardwareMap.get(DcMotor.class, "leftRear");
        horizontal = hardwareMap.get(DcMotor.class, "rightFront");

        //Mechanism motors
        turret = hardwareMap.get(DcMotor.class, "turret");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servos
        odoRetractor = hardwareMap.get(Servo.class, "odoRetractor");
        flipOut = hardwareMap.get(Servo.class, "flipOut");
        //finger = hardwareMap.get(Servo.class, "finger");
        claw = hardwareMap.get(Servo.class, "claw");
        clawLinkage = hardwareMap.get(Servo.class, "clawLinkage");
        brake = hardwareMap.get(Servo.class, "brake");

        //Magnetic switches
        slideMag = hardwareMap.get(RevTouchSensor.class, "slideMag");
        //homeMag = hardwareMap.get(RevTouchSensor.class, "homeMag");

        //Distance sensor
       distance = hardwareMap.get(ColorRangeSensor.class, "Distance");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);

        //Set motor modes
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        //Set servo positions
        odoRetractor.setPosition(ODO_RETRACT);
        claw.setPosition(CLAW_CLOSED);
        flipOut.setPosition(FLIPPED_IN);
        flipper_pos = true;
        clawLinkage.setPosition(CLAW_LINKAGE_FIVE);
        brake.setPosition(BRAKE_OFF);
        //finger.setPosition(FINGER_UP);

// Set Motor Power
        turret.setPower(0);

        telemetry.addData("Status", "Mike Wazowski is ready to run!");
        telemetry.update();

        //Wait for press play
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Drivetrain
            double forward = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(forward)+Math.abs(strafe)+Math.abs(turn), 1);

            double rightFrontPower = (forward - strafe - turn) / denominator;
            double leftFrontPower = (forward + strafe + turn) / denominator;
            double rightRearPower = (forward + strafe - turn) / denominator;
            double leftRearPower = (forward - strafe + turn) / denominator;

            if (gamepad1.right_trigger == 1) {
                rightFrontPower = Range.clip(rightFrontPower, -0.4, 0.4);
                leftFrontPower = Range.clip(leftFrontPower, -0.4, 0.4);
                rightRearPower = Range.clip(rightRearPower, -0.4, 0.4);
                leftRearPower = Range.clip(leftRearPower, -0.4, 0.4);
            } else {
                rightFrontPower = Range.clip(rightFrontPower, -0.8, 0.8);
                leftFrontPower = Range.clip(leftFrontPower, -0.8, 0.8);
                rightRearPower = Range.clip(rightRearPower, -0.8, 0.8);
                leftRearPower = Range.clip(leftRearPower, -0.8, 0.8);
            }


            rightFront.setPower(rightFrontPower);
            leftFront.setPower(leftFrontPower);
            rightRear.setPower(rightRearPower);
            leftRear.setPower(leftRearPower);

            telemetry.addData("Status", "Run " + runtime.toString());
            telemetry.addData("Motors", "forward (%.2f), strafe (%.2f),turn (%.2f)" , forward, strafe, turn);
            telemetry.update();

            //Mechanisms
            // If the distance in centimeters is less than 10, set the power to 0.3
           telemetry.addData("deviceName",distance.getDeviceName() );
           telemetry.addData("range", String.format("%.01f mm", distance.getDistance(DistanceUnit.MM)));

            telemetry.update();

            // Lift
            double motorVerticalLiftRightPower = -gamepad2.left_stick_y;
            double motorVerticalLiftLeftPower = -gamepad2.left_stick_y;


            if (!slideMag.isPressed()) { // SLIDE_UP
                motorVerticalLiftRightPower = Range.clip(motorVerticalLiftRightPower, -0.6, 1); // was 1
                motorVerticalLiftLeftPower = Range.clip(motorVerticalLiftLeftPower, -0.6, 1); //-0.90, .93)
                rightSlide.setPower(motorVerticalLiftRightPower);
                leftSlide.setPower(motorVerticalLiftLeftPower);
            } else { // SLIDE_DOWN
                motorVerticalLiftRightPower = Range.clip(motorVerticalLiftRightPower, 0, 1);// 1
                motorVerticalLiftLeftPower = Range.clip(motorVerticalLiftLeftPower, 0, 1);// .93
                rightSlide.setPower(motorVerticalLiftRightPower);
                leftSlide.setPower(motorVerticalLiftLeftPower);
            }



/*
if (gamepad2.dpad_up) {
    lift(POWER_FULL,940);
}


 */

            // Turret Brake
            if (gamepad2.left_trigger == 1) {
                brake.setPosition(BRAKE_ON);
            } else {
                brake.setPosition(BRAKE_OFF);
            }


            /*
                private static final double CLAW_LINKAGE_FIVE = 0.5;
    private static final double CLAW_LINKAGE_FOUR = 0.57;
    private static final double CLAW_LINKAGE_THREE = 0.63;
    private static final double CLAW_LINKAGE_TWO = 0.7;
    private static final double CLAW_LINKAGE_ONE = 0.78;
             */
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
            } else if (gamepad2.right_bumper && gamepad2.x) { // 0.87 1 cone, All the way down
                clawLinkage.setPosition(CLAW_LINKAGE_ONE);
            } else if (gamepad2.left_bumper) {
                clawLinkage.setPosition(CLAW_LINKAGE_FIVE); // 0.53 - 5 cones, ALl the way up
            }

            telemetry.addData("Servo",clawLinkage.getPosition());
            telemetry.addData("TurretPos", turret.getCurrentPosition());
            telemetry.update();

            if (gamepad1.a) {
                odoRetractor.setPosition(0.7);
            }




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

            // Auto Claw

            if (distance.getDistance(DistanceUnit.MM) < 40 && (!gamepad2.y || gamepad2.right_trigger > 0) ) {
                claw.setPosition(CLAW_CLOSED);

            } else if (!gamepad2.right_bumper && gamepad2.y)  {  // Otherwise, stop the motor
                claw.setPosition(CLAW_OPEN);

            } else if (gamepad2.right_trigger > 0) {
                claw.setPosition(CLAW_CLOSED);

            } else if (!flipper_pos) {
                claw.setPosition(CLAW_OPEN);

            } else {
                claw.setPosition(CLAW_CLOSED);
            }
            //else


/*
            if (gamepad2.right_trigger > 0) {
                claw.setPosition(CLAW_CLOSED);
                //finger.setPosition(FINGER_DOWN);
                telemetry.addData("Status", "Claw " + claw.getPosition());

                telemetry.update();
            } else {
                claw.setPosition(CLAW_OPEN);
                //finger.setPosition(FINGER_UP);
                telemetry.addData("Status", "Claw " + claw.getPosition());

                telemetry.update();
            }
            */




            //horizontal slide
            double motorTurretPower = -gamepad2.right_stick_x;


/*
            if (!homeMag.isPressed()) {
                if (motorTurretPower != 0 && (motorTurretPower < .25 || motorTurretPower > -.25)){
                    motorTurretPower = motorTurretPower + 0.3;
                }
               // motorTurretPower = Range.clip(motorTurretPower, -0.5, 0.5);
                telemetry.addData("motorTurretPowerNeg", motorTurretPower);
                turret.setPower(motorTurretPower);
            } else {
                motorTurretPower = Range.clip(motorTurretPower, 0, 0.7);
                telemetry.addData("motorTurretPowerPos", motorTurretPower);
                turret.setPower(motorTurretPower);
            }
*/

            //telemetry.update();
            /*
            if (gamepad2.dpad_left) {
                turret.setPower(0.4);
            } else if (gamepad2.dpad_right) {
                turret.setPower(-0.4);
            } else {
                turret.setPower(0);
            }

             */
            if (gamepad2.right_stick_x > 0) {                   // + 0.1
                turret.setPower(Range.clip(gamepad2.right_stick_x, 0, 0.35));
                lastDirection = 1;
            } else if (gamepad2.right_stick_x < 0) {
                lastDirection = -1;                               // - 0.1
                turret.setPower(Range.clip(gamepad2.right_stick_x, -0.35, 0));
            } else {
                turret.setPower(0);
            }


            //puts the turret to the "home" based on how close it is to the home position
            /*
            if (gamepad2.b && !gamepad2.right_bumper) {
                while (!homeMag.isPressed()) {
                    //right
                    if (turret.getCurrentPosition() < 0) {
                        if (turret.getCurrentPosition() > -720) {
                            turret.setPower(0.3);
                        } else if (turret.getCurrentPosition() < -720 && turret.getCurrentPosition() > -1440) {
                            turret.setPower(-0.3);
                        }
                        else if (turret.getCurrentPosition() < -1440 && turret.getCurrentPosition() > -2160) {
                            turret.setPower(0.3);
                        }
                        else if (turret.getCurrentPosition() < -2160 && turret.getCurrentPosition() > -2880) {
                            turret.setPower(-0.3);
                        }
                        //left
                    }
                    else if (turret.getCurrentPosition() > 0) {
                        if (turret.getCurrentPosition() < 720) {
                            turret.setPower(-0.3);
                        } else if (turret.getCurrentPosition() > 720 && turret.getCurrentPosition() < 1440) {
                            turret.setPower(0.3);
                        }
                        else if (turret.getCurrentPosition() > 1440 && turret.getCurrentPosition() < 2160) {
                            turret.setPower(-0.3);
                        }
                        else if (turret.getCurrentPosition() > 2160 && turret.getCurrentPosition() < 2880) {
                            turret.setPower(0.3);
                        }

                    }
                }

                turret.setPower(0);
                brake.setPosition(BRAKE_ON);
                sleep(100);
            }

             */
            //IF NOT WORKIKNG, TAKE OUT "&& !gamepad2.start" BELOW
           // if (gamepad2.b && !gamepad2.right_bumper) {
             //   while (!homeMag.isPressed()) {
                    //right
               //     if (turret.getCurrentPosition() < 0) {
                 //       turret.setPower(0.3);
                        //left
                   // }
                   // else if (turret.getCurrentPosition() > 0) {
                     //   turret.setPower(-0.3);

                   // }
                //}

                //turret.setPower(0);
                //brake.setPosition(BRAKE_ON);
            //    sleep(100);
            //}




            //telemetry.addData("LastDirection",lastDirection);




           // telemetry.addData("Encoder value", turret.getCurrentPosition());

            telemetry.update();
            idle();

        }

    }
    /*
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
    public void slideDown(){
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlide.setPower(LIFT_DOWN);
        leftSlide.setPower(LIFT_DOWN);
    }

     */
}
