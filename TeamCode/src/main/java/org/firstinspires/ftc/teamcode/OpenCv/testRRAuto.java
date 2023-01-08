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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

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

@Autonomous(name = "ENIGMA Autonomous", group = "00-Autonomous", preselectTeleOp = "ENIGMA TeleOp")
public class testRRAuto extends LinearOpMode{

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
    private CRServo revCRservo;


    private final static double servoRight = 1;
    private final static double servoLeft = -1;
    private final static double servoStop = 0;

    // initialize drive hardware


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
    int Left = 5; // Location 1
    int Middle = 10; // Location 2
    int Right = 15; // Location 3
    String ParkingZone = "None";
    int DetectedTag = 2;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {

        // initialize mech hardware
        //revCRservo = hardwareMap.get(CRServo.class, "revCRservo");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        selectStartingPosition();

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
    Pose2d midWayPose;
    Pose2d pickConePose;
    Pose2d dropConePose0, dropConePose1, dropConePose2;
    Pose2d parkPose;

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAuto() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-54, 36, Math.toRadians(0)); //Starting pose
                midWayPose = new Pose2d(-12, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(-12, 55, Math.toRadians(90)); //Choose the pose to move to the stack of cones
                dropConePose0 = new Pose2d(-12, 12, Math.toRadians(225)); //Choose the pose to move to the stack of cones
                dropConePose1 = new Pose2d(-11, 12, Math.toRadians(225)); //Choose the pose to move to the stack of cones
                dropConePose2 = new Pose2d(-10, 12, Math.toRadians(225)); //Choose the pose to move to the stack of cones
                break;
            case BLUE_RIGHT:
                initPose = new Pose2d(-54, -36, Math.toRadians(0));//Starting pose
                midWayPose = new Pose2d(-12, -36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(-12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                dropConePose0 = new Pose2d(-12, -12, Math.toRadians(135)); //Choose the pose to move to the stack of cones
                dropConePose1 = new Pose2d(-11, -12, Math.toRadians(135)); //Choose the pose to move to the stack of cones
                dropConePose2 = new Pose2d(-10, -12, Math.toRadians(135)); //Choose the pose to move to the stack of cones
                break;
            case RED_LEFT:
                initPose = new Pose2d(54, -36, Math.toRadians(180));//Starting pose
                midWayPose = new Pose2d(12, -36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                dropConePose0 = new Pose2d(12, -12, Math.toRadians(45)); //Choose the pose to move to the stack of cones
                dropConePose1 = new Pose2d(11, -12, Math.toRadians(45)); //Choose the pose to move to the stack of cones
                dropConePose2 = new Pose2d(10, -15, Math.toRadians(45)); //Choose the pose to move to the stack of cones
                break;
            case RED_RIGHT:
                initPose = new Pose2d(54, 36, Math.toRadians(180)); //Starting pose
                midWayPose = new Pose2d(12, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(12, 55, Math.toRadians(90)); //Choose the pose to move to the stack of cones
                dropConePose0 = new Pose2d(12, 12, Math.toRadians(315)); //Choose the pose to move to the stack of cones
                dropConePose1 = new Pose2d(11, 12, Math.toRadians(315)); //Choose the pose to move to the stack of cones
                dropConePose2 = new Pose2d(10, 12, Math.toRadians(315)); //Choose the pose to move to the stack of cones
                break;
        }

        //Drop Preloaded Cone, Pick 5 cones and park
        trajectoryAuto = drive.trajectorySequenceBuilder(initPose)
                .lineToLinearHeading(midWayPose)
                //Uncomment following line to slow down turn if needed.
                .setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(dropConePose0)
                .addDisplacementMarker(() -> {
                    dropCone(0); //Drop preloaded Cone
                })
                //Uncomment following line to stop reduction in speed. And move to the position after which you want to stop reducing speed.
                //.resetVelConstraint()
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(pickConePose)
                .addDisplacementMarker(() -> {
                    pickCone(1); //Pick top cone from stack
                })
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(dropConePose1)
                .addDisplacementMarker(() -> {
                    dropCone(1); //Drop cone on junction
                })
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(pickConePose)
                .addDisplacementMarker(() -> {
                    pickCone(2); //Pick second cone from stack
                })
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(dropConePose2)
                .addDisplacementMarker(() -> {
                    dropCone(2); //Drop cone on junction
                })
                .lineToLinearHeading(midWayPose)
                .build();
    }
    //Build parking trajectory based on target detected by vision
    public void buildParking(){
        switch (startPosition) {
            case BLUE_LEFT:
                switch(DetectedTag){
                    case 1: parkPose = new Pose2d(-12, 60, Math.toRadians(180)); break; // Location 1
                    case 2: parkPose = new Pose2d(-12, 36, Math.toRadians(180)); break; // Location 2
                    case 3: parkPose = new Pose2d(-12, 11, Math.toRadians(180)); break; // Location 3
                }
                break;
            case BLUE_RIGHT:
                switch(DetectedTag){
                    case 1: parkPose = new Pose2d(-12, -11, Math.toRadians(180)); break; // Location 1
                    case 2: parkPose = new Pose2d(-12, -36, Math.toRadians(180)); break; // Location 2
                    case 3: parkPose = new Pose2d(-12, -60, Math.toRadians(180)); break; // Location 3
                }
                break;
            case RED_LEFT:
                switch(DetectedTag){
                    case 1: parkPose = new Pose2d(12, -60, Math.toRadians(0)); break; // Location 1
                    case 2: parkPose = new Pose2d(12, -36, Math.toRadians(0)); break; // Location 2
                    case 3: parkPose = new Pose2d(12, -11, Math.toRadians(0)); break; // Location 3
                }
                break;
            case RED_RIGHT:
                switch(DetectedTag){
                    case 1: parkPose = new Pose2d(12, 11, Math.toRadians(0)); break; // Location 1
                    case 2: parkPose = new Pose2d(12, 36, Math.toRadians(0)); break; // Location 2
                    case 3: parkPose = new Pose2d(12, 60, Math.toRadians(0)); break; // Location 3
                }
                break;
        }
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Picked Cone: Stack", DetectedTag);
        // trajectoryParking = drive.trajectorySequenceBuilder(midWayPose)
       //         .lineToLinearHeading(parkPose)
       //         .build();
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
        drive.followTrajectorySequence(trajectoryParking);
    }

    //Write a method which is able to pick the cone from the stack depending on your subsystems
    public void pickCone(int coneCount) {
        /*TODO: Add code to pick Cone 1 from stack*/
        //revCRservo.setPower(servoLeft);
        telemetry.addData("Picked Cone: Stack", coneCount);
        telemetry.update();
    }

    //Write a method which is able to drop the cone depending on your subsystems
    public void dropCone(int coneCount){
        /*TODO: Add code to drop cone on junction*/
        //revCRservo.setPower(servoRight);

        if (coneCount == 0) {
            telemetry.addData("Dropped Cone", "Pre-loaded");
        } else {
            telemetry.addData("Dropped Cone: Stack", coneCount);
        }
        telemetry.update();
    }

    public void parkingComplete(){
        //revCRservo.setPower(servoStop);
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

}