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

@Autonomous(name = "ENIGMA Autonomous", group = "00-Autonomous", preselectTeleOp = "Mike Wazowski")
public class AutoRRtest extends LinearOpMode{

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    /*
 Mechanisms
  */
    // declare all of the servo and motor objects
    private Servo odoRetractor;
    private Servo flipOut;
    private static final double FLIPPED_IN = .29; //.95
    private static final double FLIPPED_OUT = .62;
    private Servo finger;
    private static final double FINGER_UP = 0.55;
    private static final double FINGER_DOWN = 0.85;
    private Servo claw;
    private static final double CLAW_CLOSED = 0.85;
    private static final double CLAW_OPEN = 0.4;
    private Servo clawLinkage;
    private static final double CLAW_LINKAGE_FIVE = 0.53;
    private static final double CLAW_LINKAGE_FOUR = 0.63;
    private static final double CLAW_LINKAGE_THREE = 0.7;
    private static final double CLAW_LINKAGE_TWO = 0.78;
    private static final double CLAW_LINKAGE_ONE = 0.87;
    private Servo brake;
    private static final double BRAKE_OFF = 0.16;
    private static final double BRAKE_ON = 0.28;
    // initialize drive hardware
    private static int lastDirection;

    private DcMotor leftSlide;
    private DcMotor rightSlide;
    private static final double SLIDE_UP_TOP = 1;
    private static final double SLIDE_UP_TALL_JUNCTION = .83;
    private static final double SLIDE_UP_MED_JUNCTION = .5;
    private static final double SLIDE_UP_SM_JUNCTION = .4;
    private static final double SLIDE_UP_CONE = .25;
    private static final double SLIDE_DOWN = -0.5;
    private DcMotor turret;

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
        finger = hardwareMap.get(Servo.class, "finger");
        claw = hardwareMap.get(Servo.class, "claw");
        clawLinkage = hardwareMap.get(Servo.class, "clawLinkage");
        brake = hardwareMap.get(Servo.class, "brake");

        //Magnetic switches
        slideMag = hardwareMap.get(RevTouchSensor.class, "slideMag");
        homeMag = hardwareMap.get(RevTouchSensor.class, "homeMag");

        //Distance sensor
        distance = hardwareMap.get(DistanceSensor.class, "Distance");

        //Set servo positions
        odoRetractor.setPosition(0.3);
        flipOut.setPosition(FLIPPED_IN);
        clawLinkage.setPosition(CLAW_LINKAGE_FIVE);
        claw.setPosition(CLAW_CLOSED);
        brake.setPosition(BRAKE_OFF);
        finger.setPosition(FINGER_DOWN);

        initPose = new Pose2d();
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

        buildAuto();
        drive.getLocalizer().setPoseEstimate(initPose);

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
            telemetry.addData("Start ENIGMA Autonomous Mode for Team:","16265");
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

    //Initialize any other Pose2d's as desired
    Pose2d initPose; // Starting Pose

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAuto() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        switch (startPosition) {
            case BLUE_LEFT:
                trajectoryAuto = drive.trajectorySequenceBuilder(new Pose2d())
                        .addTemporalMarker(() -> brake.setPosition(BRAKE_ON)) // lock turret
                        .addTemporalMarker(() -> flipOut.setPosition(FLIPPED_OUT)) // flip out claw linkage slide
                        .waitSeconds(0.2)
                        //.forward(21)
                        .forward(23.5) // move .forward(??) inches and raise the lift all the way up
                        .waitSeconds(0.1) // pause (??) microsec
                        .strafeLeft(36).UNSTABLE_addTemporalMarkerOffset(-0.5, () -> slideUpMed())// .strafeLeft(??) inches
                        .waitSeconds(0.2) // pause (??) a microsec to allow the lift to go all the way up
                        .addTemporalMarker(() -> dropCone(0)) // drop the cone
                        .waitSeconds(0.1)
                        .strafeLeft(13).UNSTABLE_addTemporalMarkerOffset(-0.7, () -> slideDown())
                        //.waitSeconds(0.1)
                        //.addTemporalMarker(() -> slideDown())
                        //.strafeLeft(13).UNSTABLE_addTemporalMarkerOffset(0.1, () -> slideDown()) //.strafeLeft(??) inches to be in line with cone stack and lower the lift all the way down
                        //.strafeLeft(14)
                        //.addTemporalMarker(() -> slideDown())
                        .waitSeconds(0.3) // pause (??) a microsec to allow the lift to go all the way down
                        .addTemporalMarker(() -> brake.setPosition(BRAKE_OFF)) // unlock turret
                        .addTemporalMarker(() -> turnTurret(0.25,-727)) // turn turret
                        //.addTemporalMarker(() -> brake.setPosition(BRAKE_ON)) // unlock turret
                        .waitSeconds(0.2)
                        .addTemporalMarker(() -> brake.setPosition(BRAKE_ON)) // unlock turret
                        .back(45)
                        .waitSeconds(0.2)
                        .addTemporalMarker(() -> pickCone(1))
                        .waitSeconds(0.3)
                        .addTemporalMarker(() -> slideUp(SLIDE_UP_CONE))
                        .waitSeconds(0.6)
                        .addTemporalMarker(() -> brake.setPosition(BRAKE_OFF)) // unlock turret
                        .waitSeconds(0.2)
                        .addTemporalMarker(() -> turnTurret(0.25,780)) // turn turret
                        .waitSeconds(0.2)
                        .addTemporalMarker(() -> brake.setPosition(BRAKE_ON)) // unlock turret
                        .waitSeconds(0.2)
                        //.forward(20)
                        .forward(54)
                        //.forward(54).UNSTABLE_addTemporalMarkerOffset(0.1, () -> lift(0.5,650))
                        .waitSeconds(0.4) // pause (??) a microsec to allow the lift to go all the way up
                        .addTemporalMarker(() -> dropCone(0)) // drop the cone
                        .waitSeconds(0.2)

                        .addTemporalMarker(() -> brake.setPosition(BRAKE_OFF)) // unlock turret
                        .addTemporalMarker(() -> turnTurret(0.25,-780)) // turn turret
                        //.addTemporalMarker(() -> lift(0.1,0)) // turn turret
                        .build();


                break;
            case BLUE_RIGHT:

                break;
            case RED_LEFT:

                break;
            case RED_RIGHT:

                break;
        }

        //Drop Preloaded Cone, Pick 5 cones and park
    }

    //drive turret
    public void turnTurret(double speed, int distance) {

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turret.setTargetPosition(distance);

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turret.setPower(speed);


        while (turret.isBusy() && opModeIsActive()) {
            telemetry.addData("Horizontal Slide", turret.getCurrentPosition());
            telemetry.update();
        }

        turret.setPower(0);

        //turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sleep(100);
    }
    //drive turret
    public void lift(double speed, int distance) {

        //leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setTargetPosition(distance);
        rightSlide.setTargetPosition(distance);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(speed);
        rightSlide.setPower(speed);


        while (!(Math.abs(leftSlide.getCurrentPosition()-distance) <= 10)) {
            telemetry.addData("Horizontal Slide", leftSlide.getCurrentPosition());
            telemetry.update();
        }

        leftSlide.setPower(0);
        rightSlide.setPower(0);

        //turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sleep(100);
    }


    //Build parking trajectory based on target detected by vision
    public void buildParking(){
        switch (startPosition) {
            case BLUE_LEFT:
                switch(DetectedTag){
                    case 1:

                        break; // Location 1
                    case 2:

                        break; // Location 2
                    case 3:

                        break; // Location 3
                }
                break;
            case BLUE_RIGHT:
                switch(DetectedTag){
                    case 1:

                        break; // Location 1
                    case 2:

                        break; // Location 2
                    case 3:

                        break; // Location 3
                }
                break;
            case RED_LEFT:
                switch(DetectedTag){
                    case 1:

                        break; // Location 1
                    case 2:

                        break; // Location 2
                    case 3:

                        break; // Location 3
                }
                break;
            case RED_RIGHT:
                switch(DetectedTag){
                    case 1:

                        break; // Location 1
                    case 2:

                        break; // Location 2
                    case 3:

                        break; // Location 3
                }
                break;
        }
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Picked Cone: Stack", DetectedTag);
    }
    //Run Auto trajectory and parking trajectory
    public void runAutoAndParking(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.setAutoClear(false);
        telemetry.addData("Running ENIGMA Autonomous Mode for Team:","16265");
        telemetry.addData("---------------------------------------","");
        telemetry.update();
        //Run the trajectory built for Auto and Parking
        drive.followTrajectorySequence(trajectoryAuto);
        //drive.followTrajectorySequence(trajectoryParking);


    }

    //Write a method which is able to pick the cone from the stack depending on your subsystems
    public void pickCone(int coneCount) {
        /*TODO: Add code to pick Cone 1 from stack*/
        claw.setPosition(CLAW_CLOSED);
        finger.setPosition(FINGER_DOWN);
        telemetry.addData("Picked Cone: Stack", coneCount);
        telemetry.update();
    }

    //Write a method which is able to drop the cone depending on your subsystems
    public void dropCone(int coneCount){
        /*TODO: Add code to drop cone on junction*/
        claw.setPosition(CLAW_OPEN);
        finger.setPosition(FINGER_UP);

        if (coneCount == 0) {
            telemetry.addData("Dropped Cone", "Pre-loaded");
        } else {
            telemetry.addData("Dropped Cone: Stack", coneCount);
        }
        telemetry.update();
    }
    public void grabCone(){
        claw.setPosition(CLAW_CLOSED);
        finger.setPosition(FINGER_DOWN);
    }
    public void slideUp(double here){
        rightSlide.setPower(here);
        leftSlide.setPower(here);
    }
    public void slideUpTall(){
        rightSlide.setPower(SLIDE_UP_TALL_JUNCTION);
        leftSlide.setPower(SLIDE_UP_TALL_JUNCTION);
    }
    public void slideUpMed(){
        rightSlide.setPower(SLIDE_UP_MED_JUNCTION);
        leftSlide.setPower(SLIDE_UP_MED_JUNCTION);
    }
    public void slideUpSm(){
        rightSlide.setPower(SLIDE_UP_SM_JUNCTION);
        leftSlide.setPower(SLIDE_UP_SM_JUNCTION);
    }
    public void slideDown(){
        rightSlide.setPower(SLIDE_DOWN);
        leftSlide.setPower(SLIDE_DOWN);
    }
    public void parkingComplete(){

        telemetry.addData("Parked in Location", tagOfInterest.id);
        telemetry.update();
    }
    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing ENIGMA Autonomous Mode for Team:","16265");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB Keys on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X)");
            telemetry.addData("    Blue Right ", "(Y)");
            telemetry.addData("    Red Left    ", "(B)");
            telemetry.addData("    Red Right  ", "(A)");
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }
    /* Old Coordinate Trajectory Info */
//Uncomment following line to slow down turn if needed.
    // .setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
    // .lineToLinearHeading(dropConePose0)
    // .addDisplacementMarker(() -> {
    //     dropCone(0); //Drop preloaded Cone
    //})
    //Uncomment following line to stop reduction in speed. And move to the position after which you want to stop reducing speed.
    //.resetVelConstraint()
    //   .lineToLinearHeading(midWayPose)
    ///   .lineToLinearHeading(pickConePose)
    //   .addDisplacementMarker(() -> {
    //       pickCone(1); //Pick top cone from stack
    //   })
    //   .lineToLinearHeading(midWayPose)
    //   .lineToLinearHeading(dropConePose1)
    //   .addDisplacementMarker(() -> {
    //       dropCone(1); //Drop cone on junction
    //   })
    //   .lineToLinearHeading(midWayPose)
    //   .lineToLinearHeading(pickConePose)
    //   .addDisplacementMarker(() -> {
    //       pickCone(2); //Pick second cone from stack
    //   })
    //   .lineToLinearHeading(midWayPose)
    //   .lineToLinearHeading(dropConePose2)
    //   .addDisplacementMarker(() -> {
    //       dropCone(2); //Drop cone on junction
    //   })
    //   .lineToLinearHeading(midWayPose)
}