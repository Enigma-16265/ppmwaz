/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.OpenCv;


import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;
import java.util.Locale;

@Autonomous(name = "ENIGMA Autonomous", group = "00-Autonomous", preselectTeleOp = "mWaz Auto")
public class EnigmaAuto extends LinearOpMode{

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        LEFT,
        RIGHT
    }
    public static START_POSITION startPosition;

    /*
 Mechanisms
  */
    // declare all of the servo and motor objects
    private Servo odoRetractor;
    private Servo flipOut;

    private static final double LEFT_AUTO_DISTANCE = 27.5;
    private static final double LEFT_TURRET_DEGREES = 113;
    private static final double LEFT_TURRET_SPEED = 0.28;
    //private static final int LEFT_LIFT_HEIGHT = 340;
    private static final double RIGHT_TURRET_DEGREES = 93;
    private static final double RIGHT_TURRET_SPEED = 0.18;
    private static final double RIGHT_AUTO_DISTANCE = 30;
    private static final double DROPCONEPAUSE = 1;
    private static final double FLIPPED_IN = .22; //.95
    private static final double FLIPPED_ANGLE = .7; //.95
    private static final double FLIPPED_OUT = .8;
    private Servo claw;
    private static final double CLAW_CLOSED = 0.7;
    private static final double CLAW_OPEN = 0.05;
    private Servo clawLinkage;
    private static final double CLAW_LINKAGE_TOP = 0.15;
    private static final double CLAW_LINKAGE_UP = 0.25;
    private static final double CLAW_LINKAGE_DIP = 0.35;
    private static final double CLAW_LINKAGE_FIVE = 0.37;
    private static final double CLAW_LINKAGE_FOUR = 0.395;
    private static final double CLAW_LINKAGE_THREE = 0.42;
    private static final double CLAW_LINKAGE_TWO = 0.445;
    private static final double CLAW_LINKAGE_ONE = 0.48;    private Servo brake;
    private static final double BRAKE_OFF = 0.16;
    private static final double BRAKE_ON = 0.23;
    // initialize drive hardware
    private static int lastDirection;

    private DcMotor leftSlide;
    private DcMotor rightSlide;
    private static final double POWER_FULL = 1;
    private static final double LIFT_DOWN = -0.3;
    private DcMotor turret;
    private static boolean flipper_pos;
    //Magnetic Switches
    private RevTouchSensor slideMag;
    private RevTouchSensor homeMag;

    //Distance sensor
    private DistanceSensor distance;

    /*
    OpenCV / April Tags
     */
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // tag ids of signal sleeve
    int Left = 1; // Location 1
    int Middle = 2; // Location 2
    int Right = 3; // Location 3
    String ParkingZone = "None";
    int DetectedTag = 2;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {

        // initialize mech hardware
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        selectStartingPosition();
        // Motors
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
        homeMag = hardwareMap.get(RevTouchSensor.class, "homeMag");

        //Distance sensor
        distance = hardwareMap.get(DistanceSensor.class, "Distance");

        //Set servo positions
        odoRetractor.setPosition(0.1);
        clawLinkage.setPosition(CLAW_LINKAGE_ONE);
        claw.setPosition(CLAW_CLOSED);
        flipOut.setPosition(FLIPPED_IN);
        flipper_pos = true;
        brake.setPosition(BRAKE_OFF);

        // Vision OpenCV / Apriltags
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //OpenCV Pipeline
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        Pose2d startPose = new Pose2d(0, 0, 0);
        buildAuto();
        drive.getLocalizer().setPoseEstimate(startPose);

        //runtime.reset();
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        while (!isStopRequested() && !opModeIsActive()) {
            //Run OpenCV and keep watching for the identifier on the Signal Cone.
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == Left || tag.id == Right || tag.id == Middle)
                    {
                        if (tag.id == Left) {
                            ParkingZone = "Location 1";
                        } else if (tag.id == Middle) {
                            ParkingZone = "Location 2";
                        } else if (tag.id == Right) {
                            ParkingZone = "Location 3";
                        }
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }

            //telemetry.clearAll();
            telemetry.addData("Start ENIGMA Auto Mode for Team:","16265");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Selected Starting Position:", startPosition);
            telemetry.addData("Vision identified Parking Location:", ParkingZone);
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            /*
             * The START command just came in: now work off the latest snapshot acquired
             * during the init loop.
             */

            /* Update the telemetry */
            if (tagOfInterest != null) {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            } else {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

            /* Actually do something useful */
            if (tagOfInterest == null) {
                DetectedTag = 2;
            } else if (tagOfInterest.id == Middle) {
                DetectedTag = 2;
            } else if (tagOfInterest.id == Right) {
                DetectedTag = 3;
            } else if (tagOfInterest.id == Left) {
                DetectedTag = 1;
            }
            buildParking();
            //run Autonomous trajectory
            runAutoAndParking();
        }

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format(Locale.ENGLISH,"\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format(Locale.ENGLISH,"Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format(Locale.ENGLISH,"Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format(Locale.ENGLISH,"Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format(Locale.ENGLISH,"Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format(Locale.ENGLISH,"Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format(Locale.ENGLISH,"Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }


    public int inchesToTicks( double inches ) {
        return (int)(inches * 1500 / 33.5);
    }

    //Initialize any other TrajectorySequences as desired
    TrajectorySequence trajectoryAuto, trajectoryParking ;
    //Initialize any other Pose2d's as desired Pose2d initPose; // Starting Pose

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAuto() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        switch (startPosition) {
            case LEFT:
                trajectoryAuto = drive.trajectorySequenceBuilder(startPose)
                        .forward(40) // .forward(??) inches to medium junction
                        .UNSTABLE_addTemporalMarkerOffset(-.7, () -> brake.setPosition(BRAKE_ON))//  set brake on
                        .UNSTABLE_addTemporalMarkerOffset(-.8, () -> lift(POWER_FULL, 410))//  lift up (motor power)
                        .UNSTABLE_addTemporalMarkerOffset(-.9, () -> flipOut.setPosition(FLIPPED_OUT))//  set mechanism to be flipped out
                        .waitSeconds(.3) // pause (??) a microseconds
                        .addTemporalMarker(() -> dropCone(0)) // drop the preloaded cone, enter a count for each one
                        .waitSeconds(0.1) // pause (??) microseconds

                        .forward(11) // drive forward to line up with the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-.4, () -> brake.setPosition(BRAKE_OFF))//  set brake off
                        .UNSTABLE_addTemporalMarkerOffset(-.7, () -> turnTurret(0.45,(int) ticksToDegrees(95, Right)))// turn the turret to the rear of the robot facing the cone stack
                        .waitSeconds(0.1) // pause (??) microseconds
                        .addTemporalMarker(() -> slideDown()) // bring the lift down
                        .turn(Math.toRadians(-90)) // turn the robot so the rear is facing the cone stack
                        .waitSeconds(.1) // pause (??) microseconds
                        .back(21) // drive backwards to the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.3, () -> clawLinkage.setPosition(CLAW_LINKAGE_FIVE))//  lift up (motor power)
                        .waitSeconds(.25) // pause (??) microseconds
                        .addTemporalMarker(() -> pickCone(1)) // grab cone 1 off the stack
                        .waitSeconds(.2) // pause (??) microseconds

                        .forward(LEFT_AUTO_DISTANCE) // drive backwards to the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> turnTurret(0.23,(int) ticksToDegrees(LEFT_TURRET_DEGREES, Left)))// turn the turret to the rear of the robot facing the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.35, () -> lift(POWER_FULL, 470))//  lift up (motor power)
                        .waitSeconds(.25) // pause (??) a microseconds
                        .addTemporalMarker(() -> dropCone(1)) // drop cone 1
                        .addTemporalMarker(() -> clawLinkage.setPosition(CLAW_LINKAGE_TOP))
                        .waitSeconds(0.1) // pause (??) microseconds

                        .back(LEFT_AUTO_DISTANCE)
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> turnTurret(LEFT_TURRET_SPEED,(int) ticksToDegrees(LEFT_TURRET_DEGREES, Right)))
                        .UNSTABLE_addTemporalMarkerOffset(-.3, () -> clawLinkage.setPosition(CLAW_LINKAGE_FOUR))//  lift up (motor power)
                        .UNSTABLE_addTemporalMarkerOffset(-.8, () -> slideDown())// turn the turret to the rear of the robot facing the cone stack
                        .waitSeconds(.25) // pause (??) microseconds
                        .addTemporalMarker(() -> pickCone(2)) // grab cone 1 off the stack
                        .waitSeconds(.2) // pause (??) microseconds

                        .forward(LEFT_AUTO_DISTANCE) // drive backwards to the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> turnTurret(0.23,(int) ticksToDegrees(LEFT_TURRET_DEGREES, Left)))// turn the turret to the rear of the robot facing the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.35, () -> lift(POWER_FULL, 490))//  lift up (motor power)
                        .waitSeconds(.1) // pause (??) a microseconds
                        .addTemporalMarker(() -> dropCone(2)) // drop cone 1
                        .waitSeconds(0.1) // pause (??) microseconds


                        .back(LEFT_AUTO_DISTANCE)
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> turnTurret(LEFT_TURRET_SPEED,(int) ticksToDegrees(LEFT_TURRET_DEGREES, Right)))
                        .UNSTABLE_addTemporalMarkerOffset(-.3, () -> clawLinkage.setPosition(CLAW_LINKAGE_THREE))//  lift up (motor power)
                        .UNSTABLE_addTemporalMarkerOffset(-.8, () -> slideDown())// turn the turret to the rear of the robot facing the cone stack
                        .waitSeconds(.25) // pause (??) microseconds
                        .addTemporalMarker(() -> pickCone(3)) // grab cone 1 off the stack
                        .waitSeconds(.2) // pause (??) microseconds

                        .forward(LEFT_AUTO_DISTANCE) // drive backwards to the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> turnTurret(0.23,(int) ticksToDegrees(LEFT_TURRET_DEGREES, Left)))// turn the turret to the rear of the robot facing the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.35, () -> lift(POWER_FULL, 490))//  lift up (motor power)
                        .waitSeconds(.1) // pause (??) a microseconds
                        .addTemporalMarker(() -> dropCone(3)) // drop cone 1
                        .waitSeconds(0.1) // pause (??) microseconds

                        .back(LEFT_AUTO_DISTANCE)
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> turnTurret(LEFT_TURRET_SPEED,(int) ticksToDegrees(LEFT_TURRET_DEGREES, Right)))
                        .UNSTABLE_addTemporalMarkerOffset(-.3, () -> clawLinkage.setPosition(CLAW_LINKAGE_TWO))//  lift up (motor power)
                        .UNSTABLE_addTemporalMarkerOffset(-.8, () -> slideDown())// turn the turret to the rear of the robot facing the cone stack
                        .waitSeconds(.25) // pause (??) microseconds
                        .addTemporalMarker(() -> pickCone(4)) // grab cone 1 off the stack
                        .waitSeconds(.2) // pause (??) microseconds

                        .forward(LEFT_AUTO_DISTANCE) // drive backwards to the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> turnTurret(0.23,(int) ticksToDegrees(LEFT_TURRET_DEGREES, Left)))// turn the turret to the rear of the robot facing the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.35, () -> lift(POWER_FULL, 490))//  lift up (motor power)
                        .waitSeconds(.1) // pause (??) a microseconds
                        .addTemporalMarker(() -> dropCone(4)) // drop cone 1
                        .waitSeconds(0.1) // pause (??) microseconds

                        .back(LEFT_AUTO_DISTANCE)
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> turnTurret(LEFT_TURRET_SPEED,(int) ticksToDegrees(LEFT_TURRET_DEGREES, Right)))
                        .UNSTABLE_addTemporalMarkerOffset(-.3, () -> clawLinkage.setPosition(CLAW_LINKAGE_ONE))//  lift up (motor power)
                        .UNSTABLE_addTemporalMarkerOffset(-.8, () -> slideDown())// turn the turret to the rear of the robot facing the cone stack
                        .waitSeconds(.25) // pause (??) microseconds
                        .addTemporalMarker(() -> pickCone(5)) // grab cone 1 off the stack
                        .waitSeconds(.2) // pause (??) microseconds

                        .forward(LEFT_AUTO_DISTANCE) // drive backwards to the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> turnTurret(0.23,(int) ticksToDegrees(LEFT_TURRET_DEGREES, Left)))// turn the turret to the rear of the robot facing the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.35, () -> lift(POWER_FULL, 490))//  lift up (motor power)
                        .waitSeconds(.1) // pause (??) a microseconds
                        .addTemporalMarker(() -> dropCone(5)) // drop cone 1
                        .waitSeconds(0.1) // pause (??) microseconds
                        .build(); // build trajectory
                break;
            case RIGHT:
                trajectoryAuto = drive.trajectorySequenceBuilder(startPose)
                        .forward(40) // .forward(??) inches to medium junction
                        .UNSTABLE_addTemporalMarkerOffset(-.7, () -> brake.setPosition(BRAKE_OFF))//  set brake on
                        .UNSTABLE_addTemporalMarkerOffset(-.8, () -> lift(POWER_FULL, 450))//  lift up (motor power)
                        .UNSTABLE_addTemporalMarkerOffset(-1.8, () -> turnTurret(0.20,(int) ticksToDegrees(180, Left)))// turn the turret to the rear of the robot facing the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-.9, () -> flipOut.setPosition(FLIPPED_OUT))//  set mechanism to be flipped out
                        .waitSeconds(.5) // pause (??) a microseconds
                        .addTemporalMarker(() -> dropCone(0)) // drop the preloaded cone, enter a count for each one
                        .waitSeconds(0.1) // pause (??) microseconds

                        .forward(11.5) // drive forward to line up with the cone stack
                        //.UNSTABLE_addTemporalMarkerOffset(-.4, () -> brake.setPosition(BRAKE_OFF))//  set brake off
                        .UNSTABLE_addTemporalMarkerOffset(-.7, () -> turnTurret(0.15,(int) ticksToDegrees(93, Left)))// turn the turret to the rear of the robot facing the cone stack
                        .waitSeconds(0.1) // pause (??) microseconds
                        .addTemporalMarker(() -> slideDown()) // bring the lift down
                        .turn(Math.toRadians(90)) // turn the robot so the rear is facing the cone stack
                        .waitSeconds(.1) // pause (??) microseconds
                        .back(19) // drive backwards to the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.3, () -> clawLinkage.setPosition(CLAW_LINKAGE_FIVE))//  lift up (motor power)
                        .waitSeconds(.25) // pause (??) microseconds
                        .addTemporalMarker(() -> pickCone(1)) // grab cone 1 off the stack
                        .waitSeconds(.25) // pause (??) microseconds

                        .forward(RIGHT_AUTO_DISTANCE) // drive backwards to the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> turnTurret(0.18,(int) ticksToDegrees(RIGHT_TURRET_DEGREES, Right)))// turn the turret to the rear of the robot facing the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.35, () -> lift(POWER_FULL, 470))//  lift up (motor power)
                        .waitSeconds(.25) // pause (??) a microseconds
                        .addTemporalMarker(() -> dropCone(1)) // drop cone 1
                        .addTemporalMarker(() -> clawLinkage.setPosition(CLAW_LINKAGE_TOP))
                        .waitSeconds(0.1) // pause (??) microseconds

                        .back(RIGHT_AUTO_DISTANCE)
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> turnTurret(RIGHT_TURRET_SPEED,(int) ticksToDegrees(RIGHT_TURRET_DEGREES, Left)))
                        .UNSTABLE_addTemporalMarkerOffset(-.3, () -> clawLinkage.setPosition(CLAW_LINKAGE_FOUR))//  lift up (motor power)
                        .UNSTABLE_addTemporalMarkerOffset(-.8, () -> slideDown())// turn the turret to the rear of the robot facing the cone stack
                        .waitSeconds(.25) // pause (??) microseconds
                        .addTemporalMarker(() -> pickCone(2)) // grab cone 1 off the stack
                        .waitSeconds(.25) // pause (??) microseconds

                        .forward(RIGHT_AUTO_DISTANCE) // drive backwards to the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> turnTurret(0.18,(int) ticksToDegrees(RIGHT_TURRET_DEGREES, Right)))// turn the turret to the rear of the robot facing the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.35, () -> lift(POWER_FULL, 490))//  lift up (motor power)
                        .waitSeconds(.1) // pause (??) a microseconds
                        .addTemporalMarker(() -> dropCone(2)) // drop cone 1
                        .waitSeconds(0.1) // pause (??) microseconds


                        .back(RIGHT_AUTO_DISTANCE)
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> turnTurret(RIGHT_TURRET_SPEED,(int) ticksToDegrees(RIGHT_TURRET_DEGREES, Left)))
                        .UNSTABLE_addTemporalMarkerOffset(-.3, () -> clawLinkage.setPosition(CLAW_LINKAGE_THREE))//  lift up (motor power)
                        .UNSTABLE_addTemporalMarkerOffset(-.8, () -> slideDown())// turn the turret to the rear of the robot facing the cone stack
                        .waitSeconds(.25) // pause (??) microseconds
                        .addTemporalMarker(() -> pickCone(3)) // grab cone 1 off the stack
                        .waitSeconds(.25) // pause (??) microseconds

                        .forward(RIGHT_AUTO_DISTANCE) // drive backwards to the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> turnTurret(0.18,(int) ticksToDegrees(RIGHT_TURRET_DEGREES, Right)))// turn the turret to the rear of the robot facing the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.35, () -> lift(POWER_FULL, 490))//  lift up (motor power)
                        .waitSeconds(.1) // pause (??) a microseconds
                        .addTemporalMarker(() -> dropCone(3)) // drop cone 1
                        .waitSeconds(0.1) // pause (??) microseconds

                        .back(RIGHT_AUTO_DISTANCE)
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> turnTurret(RIGHT_TURRET_SPEED,(int) ticksToDegrees(RIGHT_TURRET_DEGREES, Left)))
                        .UNSTABLE_addTemporalMarkerOffset(-.3, () -> clawLinkage.setPosition(CLAW_LINKAGE_TWO))//  lift up (motor power)
                        .UNSTABLE_addTemporalMarkerOffset(-.8, () -> slideDown())// turn the turret to the rear of the robot facing the cone stack
                        .waitSeconds(.25) // pause (??) microseconds
                        .addTemporalMarker(() -> pickCone(4)) // grab cone 1 off the stack
                        .waitSeconds(.25) // pause (??) microseconds

                        .forward(RIGHT_AUTO_DISTANCE) // drive backwards to the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> turnTurret(0.18,(int) ticksToDegrees(RIGHT_TURRET_DEGREES, Right)))// turn the turret to the rear of the robot facing the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.35, () -> lift(POWER_FULL, 490))//  lift up (motor power)
                        .waitSeconds(.1) // pause (??) a microseconds
                        .addTemporalMarker(() -> dropCone(4)) // drop cone 1
                        .waitSeconds(0.1) // pause (??) microseconds

                        .back(RIGHT_AUTO_DISTANCE)
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> turnTurret(RIGHT_TURRET_SPEED,(int) ticksToDegrees(RIGHT_TURRET_DEGREES, Left)))
                        .UNSTABLE_addTemporalMarkerOffset(-.3, () -> clawLinkage.setPosition(CLAW_LINKAGE_ONE))//  lift up (motor power)
                        .UNSTABLE_addTemporalMarkerOffset(-.8, () -> slideDown())// turn the turret to the rear of the robot facing the cone stack
                        .waitSeconds(.25) // pause (??) microseconds
                        .addTemporalMarker(() -> pickCone(5)) // grab cone 1 off the stack
                        .waitSeconds(.25) // pause (??) microseconds

                        .forward(RIGHT_AUTO_DISTANCE) // drive backwards to the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> turnTurret(0.18,(int) ticksToDegrees(RIGHT_TURRET_DEGREES, Right)))// turn the turret to the rear of the robot facing the cone stack
                        .UNSTABLE_addTemporalMarkerOffset(-1.35, () -> lift(POWER_FULL, 490))//  lift up (motor power)
                        .waitSeconds(.1) // pause (??) a microseconds
                        .addTemporalMarker(() -> dropCone(5)) // drop cone 1
                        .waitSeconds(0.1) // pause (??) microseconds
                        .build(); // build trajectory
                break;

        }

        //Drop Preloaded Cone, Pick 5 cones and park
    }
    // Motor Encoder Ticks to Degrees
    public double ticksToDegrees(double degrees, int direction){
        double turnNone = 0;
        double equate = (degrees*2);
        double eqout = (equate*2.0194);
        if (direction == 1) { // turn left
            return -eqout;
        } else if (direction == 3) { // turn right
            return eqout;
        }
        return turnNone;
    }
    //drive turret
    public void turnTurret(double speed, int distance) {

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setTargetPosition(distance);

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turret.setPower(speed);

    }
    //drive lift
    public void lift(double speed, int distance) {
clawLinkage.setPosition(CLAW_LINKAGE_UP);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition(distance);
        rightSlide.setTargetPosition(distance);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(speed);
        rightSlide.setPower(speed);

    }


    //Build parking trajectory based on target detected by vision
    public void buildParking(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        switch (startPosition) {
            case LEFT:
                switch(DetectedTag){
                    case 1:
                        trajectoryParking = drive.trajectorySequenceBuilder(trajectoryAuto.end())
                                .addTemporalMarker(() -> brake.setPosition(BRAKE_ON)) // lock turret
                                .addTemporalMarker(() -> claw.setPosition(CLAW_CLOSED)) // claw closed
                                .addTemporalMarker(() -> clawLinkage.setPosition(CLAW_LINKAGE_ONE))
                                .addTemporalMarker(() -> flipOut.setPosition(FLIPPED_IN)) // flip out claw linkage slide
                                .addTemporalMarker(() -> slideDown()) // bring the lift down
                                .back(30)
                                .waitSeconds(0.1)
                                .build();
                        break; // Location 1
                    case 2:
                        trajectoryParking = drive.trajectorySequenceBuilder(trajectoryAuto.end())
                                .addTemporalMarker(() -> brake.setPosition(BRAKE_ON)) // lock turret
                                .addTemporalMarker(() -> claw.setPosition(CLAW_CLOSED)) // claw closed
                                .addTemporalMarker(() -> clawLinkage.setPosition(CLAW_LINKAGE_ONE))
                                .addTemporalMarker(() -> flipOut.setPosition(FLIPPED_IN)) // flip out claw linkage slide
                                .addTemporalMarker(() -> slideDown()) // bring the lift down
                                .back(8)
                                .waitSeconds(0.1)
                                .build();
                        break; // Location 2
                    case 3:
                        trajectoryParking = drive.trajectorySequenceBuilder(trajectoryAuto.end())
                                .addTemporalMarker(() -> brake.setPosition(BRAKE_ON)) // lock turret
                                .addTemporalMarker(() -> claw.setPosition(CLAW_CLOSED)) // claw closed
                                .addTemporalMarker(() -> clawLinkage.setPosition(CLAW_LINKAGE_ONE))
                                .addTemporalMarker(() -> flipOut.setPosition(FLIPPED_IN)) // flip out claw linkage slide
                                .addTemporalMarker(() -> slideDown()) // bring the lift down
                                .forward(15)
                                .waitSeconds(0.1)
                                .build();
                        break; // Location 3
                }
                break;
            case RIGHT:
                switch(DetectedTag){
                    case 3:
                        trajectoryParking = drive.trajectorySequenceBuilder(trajectoryAuto.end())
                                .addTemporalMarker(() -> brake.setPosition(BRAKE_ON)) // lock turret
                                .addTemporalMarker(() -> claw.setPosition(CLAW_CLOSED)) // claw closed
                                .addTemporalMarker(() -> clawLinkage.setPosition(CLAW_LINKAGE_ONE))
                                .addTemporalMarker(() -> flipOut.setPosition(FLIPPED_IN)) // flip out claw linkage slide
                                .addTemporalMarker(() -> slideDown()) // bring the lift down
                                .back(30)
                                .waitSeconds(0.1)
                                .build();
                        break; // Location 1
                    case 2:
                        trajectoryParking = drive.trajectorySequenceBuilder(trajectoryAuto.end())
                                .addTemporalMarker(() -> brake.setPosition(BRAKE_ON)) // lock turret
                                .addTemporalMarker(() -> claw.setPosition(CLAW_CLOSED)) // claw closed
                                .addTemporalMarker(() -> clawLinkage.setPosition(CLAW_LINKAGE_ONE))
                                .addTemporalMarker(() -> flipOut.setPosition(FLIPPED_IN)) // flip out claw linkage slide
                                .addTemporalMarker(() -> slideDown()) // bring the lift down
                                .back(8)
                                .waitSeconds(0.1)
                                .build();
                        break; // Location 2
                    case 1:
                        trajectoryParking = drive.trajectorySequenceBuilder(trajectoryAuto.end())
                                .addTemporalMarker(() -> brake.setPosition(BRAKE_ON)) // lock turret
                                .addTemporalMarker(() -> claw.setPosition(CLAW_CLOSED)) // claw closed
                                .addTemporalMarker(() -> clawLinkage.setPosition(CLAW_LINKAGE_ONE))
                                .addTemporalMarker(() -> flipOut.setPosition(FLIPPED_IN)) // flip out claw linkage slide
                                .addTemporalMarker(() -> slideDown()) // bring the lift down
                                .forward(15)
                                .waitSeconds(0.1)
                                .build();
                        break; // Location 3
                }
                break;
        }

        telemetry.addData("Picked Cone: Stack", DetectedTag);
    }
    //Run Auto trajectory and parking trajectory
    public void runAutoAndParking(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.setAutoClear(false);
        telemetry.addData("Running ENIGMA Auto Mode for Team:","16265");
        telemetry.addData("---------------------------------------","");
        telemetry.update();
        //Run the trajectory built for Auto and Parking
        drive.followTrajectorySequence(trajectoryAuto);
        drive.followTrajectorySequence(trajectoryParking);


    }

    //Write a method which is able to pick the cone from the stack depending on your subsystems
    public void pickCone(int coneCount) {
        /*TODO: Add code to pick Cone 1 from stack*/
        claw.setPosition(CLAW_CLOSED);
        telemetry.addData("Picked Cone: Stack", coneCount);
        telemetry.update();
    }

    //Write a method which is able to drop the cone depending on your subsystems
    public void dropCone(int coneCount){
        /*TODO: Add code to drop cone on junction*/
       clawLinkage.setPosition(CLAW_LINKAGE_DIP);
        sleep(100);
        claw.setPosition(CLAW_OPEN);
       sleep(100);
       clawLinkage.setPosition(CLAW_LINKAGE_TOP);
        if (coneCount == 0) {
            telemetry.addData("Dropped Cone", "Pre-loaded");
        } else {
            telemetry.addData("Dropped Cone: Stack", coneCount);
        }
        telemetry.update();
    }
    // send the lift up by giving it a power parameter stored in CONSTANTS based on % as follows:
    // POWER_FULL, POWER_NINETY, POWER_EIGHTY, POWER_SEVENTY, POWER_SIXTY, POWER_FIFTY, POWER_FORTY, POWER_THIRTY, POWER_TWENTY, POWER_TEN
    public void liftUp(double motorPower){
        rightSlide.setPower(motorPower);
        leftSlide.setPower(motorPower);
    }

    public void slideDown(){
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlide.setPower(0);
        leftSlide.setPower(0);

        rightSlide.setPower(LIFT_DOWN);
        leftSlide.setPower(LIFT_DOWN);
    }
    public void preSlideDown(){
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlide.setPower(0);
        leftSlide.setPower(0);
    }
    public void parkingComplete(){

        telemetry.addData("Parked in Location", tagOfInterest.id);
        telemetry.update();
    }
    //Method to select starting position using X, Y buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing ENIGMA Auto Mode for Team:","16265");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB Keys on gamepad 1:","");
            telemetry.addData("    Left   ", "(X)");
            telemetry.addData("    Right ", "(Y)");
            if(gamepad1.x){
                startPosition = START_POSITION.LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }
}