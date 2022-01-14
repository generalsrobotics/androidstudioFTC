package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This OpMode illustrates using a webcam to locate and drive towards ANY Vuforia target.
 * The code assumes a basic two-wheel Robot Configuration with motors named left_drive and right_drive.
 * The motor directions must be set so a positive drive goes forward and a positive turn rotates to the right.
 *
 * Under manual control, the left stick will move forward/back, and the right stick will turn left/right.
 * This is called POV Joystick mode, different than Tank Drive (where each joystick controls a wheel).
 * Manually drive the robot until it displays Target data on the Driver Station.
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN and TURN_GAIN constants.
 *
 * For more Vuforia details, or to adapt this OpMode for a phone camera, view the
 *  ConceptVuforiaFieldNavigation and ConceptVuforiaFieldNavigationWebcam samples.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name="VuforiaBlueAutonomous", group = "Linear Opmode")

public class BlueVuforiaAutonomous extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 8.0; //  this is how close the camera should get to the target (inches)
                                         //  The GAIN constants set the relationship between the measured position error,
                                         //  and how much power is applied to the drive motors.  Drive = Error * Gain
                                         //  Make these values smaller for smoother control.
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MM_PER_INCH = 25.40 ;   //  Metric conversion

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ASg9akz/////AAABmUyMSYT980NPnVlwE+IUZvgcmQVycKh4y6qF5zwgIINrt/luYwZsjEBlHQR43ATLOgSb4zmCadznybzcf1EI0fTBLHFk0VArY96x/Cw6kulvlQF5BCmFHXqmWWulbDy7ASUWKVDK64OAbFSEJC0qqMZEYEQ/UHashdor5748WoqVfpnVs+8XeYMqZIDnnJpHHGbl1M4hGlzK0xVH96T1O0/hqsBAMd6XZjg+Whz04FziDdKSc6NVf65WXQop4m0rwbN+EnymPddCqnAj0xVVKefY8KuI7aJNDX/OUbDGUOWTb5hxl2KUbTYiUiGZQpqtLG2d8NpZ003naRIQRl0ev6+4yTFud9YyvAv2yNRgT82h";


    VuforiaLocalizer vuforia    = null;
    OpenGLMatrix targetPose     = null;
    String targetName           = "";


    private DcMotor frontright = null;
    private DcMotor backright = null;
    private DcMotor frontleft = null;
    private DcMotor backleft = null;
    private DcMotor IntakeMotor =null;
    // todo: write your code here
    private int leftPos;
    private int rightPos;
    private DcMotor duckMotor =null;
    private DcMotor armMotor = null;
    private DcMotor extenderMotor = null;

    private double pulses= 537;
    private double diamter = 4.724;

    private int armPulses = 5281;
    private int bigSprocket = 42;
    private int smallSprocket = 8;
    @Override public void runOpMode() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * To get an on-phone camera preview, use the code below.
         * If no camera preview is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the trackable objects from the Assets file, and give them meaningful names
        VuforiaTrackables targetsFreightFrenzy = this.vuforia.loadTrackablesFromAsset("testq");
        // VuforiaTrackables cap = this.vuforia.loadTrackablesFromAsset("testq");


        //targetsFreightFrenzy.add(targetsFreightFrenzy.get(0));
        targetsFreightFrenzy.get(0).setName("Blue Storage");
        //targetsFreightFrenzy.get(1).setName("Blue Alliance Wall");
        //targetsFreightFrenzy.get(2).setName("Red Storage");
        //targetsFreightFrenzy.get(3).setName("Red Alliance Wall");
        // targetsFreightFrenzy.get(4).setName("cup");


        // Start tracking targets in the background
        targetsFreightFrenzy.activate();
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backright = hardwareMap.get(DcMotor.class, "backright");

        duckMotor = hardwareMap.get(DcMotor.class, "m3");

        armMotor = hardwareMap.get(DcMotor.class, "m0");

        extenderMotor = hardwareMap.get(DcMotor.class, "m1");
        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");

        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extenderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setDirection(DcMotor.Direction.FORWARD);
        backleft.setDirection(DcMotor.Direction.FORWARD);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.REVERSE);
        duckMotor.setDirection(DcMotor.Direction.REVERSE);

        leftPos = 0;
        rightPos = 0;





        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        //rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // leftDrive.setDirection(DcMotor.Direction.FORWARD);
        // rightDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        boolean targetFound = false;    // Set to true when a target is detected by Vuforia
        double targetRange = 0;        // Distance from camera to target in Inches
        double targetBearing = 0;        // Robot Heading, relative to target.  Positive degrees means target is to the right.
        double drive = 0;        // Desired forward power (-1 to +1)
        double turn = 0;        // Desired turning power (-1 to +1)

        while (opModeIsActive()) {


            // Look for first visible target, and save its pose.
            targetFound = false;
            for (VuforiaTrackable trackable : targetsFreightFrenzy) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    targetPose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getVuforiaCameraFromTarget();

                    // if we have a target, process the "pose" to determine the position of the target relative to the robot.
                    if (targetPose != null) {
                        targetFound = true;
                        targetName = trackable.getName();
                        VectorF trans = targetPose.getTranslation();

                        // Extract the X & Y components of the offset of the target relative to the robot
                        double targetX = trans.get(0) / MM_PER_INCH; // Image X axis
                        double targetY = trans.get(2) / MM_PER_INCH; // Image Z axis

                        // target range is based on distance from robot position to origin (right triangle).
                        targetRange = Math.hypot(targetX, targetY);

                        // target bearing is based on angle formed between the X axis and the target range line
                        targetBearing = Math.toDegrees(Math.asin(targetX / targetRange));

                        break;  // jump out of target tracking loop if we find a target.
                    }
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData(">", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Target", " %s", targetName);
                telemetry.addData("Range", "%5.1f inches", targetRange);
                telemetry.addData("Bearing", "%3.0f degrees", targetBearing);
            }

            int d = (int) ((80-3) *((pulses/Math.PI)/diamter));

            //level 1
            if (targetFound && targetBearing >= 10 && targetBearing <= 25 ) {
                telemetry.addData("Found Flag","Going to level 1");
                // Determine heading and range error so we can use them to control the robot automatically.
                double rangeError = (targetRange - DESIRED_DISTANCE);
                double headingError = targetBearing;


                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = rangeError * SPEED_GAIN;
                turn = headingError * TURN_GAIN;

                goForward(12);
                turnRight(90);
                goForward(25);
                duckMotor.setPower(-0.25);
                sleep(3000);
                goForward(-44);
                turnLeft(95);
                goForward(9);
                level1();

                turnLeft(90);
                moveArm(100);
                drive(d,d,.50);
                //drive(d,d,.50);
                //goForward(-20);
               // turnRight(55);
               // drive(d,d,.50);


                break;

                //level 2
            } else if (targetFound && targetBearing >= -8 && targetBearing <= 7){
                telemetry.addData("Found Flag","Going to level 2");
                goForward(12);
                turnRight(90);
                goForward(25);
                duckMotor.setPower(-0.25);
                sleep(3000);
                goForward(-44);
                turnLeft(95);
                goForward(9);
                level2();

                turnLeft(90);
                moveArm(100);
                drive(d,d,.50);
                break;
                //level 3
           }else if(targetFound && targetBearing >= -30 && targetBearing <= -15) {
                telemetry.addData("Found Flag","Going to level 3");
                goForward(12);
                turnRight(90);
                goForward(25);
                duckMotor.setPower(-0.25);
                sleep(3000);
                goForward(-44);
                turnLeft(95);
                goForward(9);
                level3();

                turnLeft(90);
                moveArm(100);
                drive(d,d,.50);
                //drive(d,d,.50);
                //goForward(-20);
                //turnRight(55);
                //drive(d,d,.50);
                break;
            }





            telemetry.update();
            sleep(10);
        }
    }
            private void pause(double milliseconds){
                double t;
                t = getRuntime();
                while (getRuntime() - t > milliseconds) ;

            }
            private void goForward(double distance){

                //This will tell to the driver hub whats the robot is doing.
                telemetry.addData("goForward", distance);
                telemetry.update();
                int d = (int) ((distance - 3) * ((pulses / Math.PI) / diamter));
                drive(d, d, 0.3);
            }


            private void turnLeft( int degree)
            {
                //This will tell to the driver hub whats the robot is doing.
                telemetry.addData("turningLeft", degree);
                telemetry.update();
                sleep(500);

                //  int distance;
                int distance = degree * (degree / 16);

                drive(-(int) distance, (int) distance, 0.25);
                sleep(500);

            }

            private void turnRight ( int degree)
            {
                //This will tell to the driver hub whats the robot is doing.
                telemetry.addData("turningRight", degree);
                telemetry.update();
                sleep(500);

                //  int distance;
                int distance = degree * (degree / 16);

                drive((int) distance, -(int) distance, 0.25);
                sleep(500);

            }
            private void drive( int leftTarget, int rightTarget, double speed){
                leftPos += leftTarget;
                rightPos += rightTarget;


                //setting distance target
                backright.setTargetPosition(rightPos);
                backleft.setTargetPosition(leftPos);
                frontright.setTargetPosition(rightPos);
                frontleft.setTargetPosition(leftPos);

                // moving motors to distance target
                backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleft.getCurrentPosition();


                backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //This will tell to the driver hub whats the robot is doing.
                telemetry.addData("backRight", backright.getCurrentPosition());
                telemetry.addData("Rtarget", rightTarget);
                telemetry.addData("Rposition", rightPos);

                telemetry.addData("backLeft", backleft.getCurrentPosition());
                telemetry.addData("Ltarget", leftTarget);
                telemetry.addData("Rposition", leftPos);
                telemetry.addData("armMotor", armMotor.getCurrentPosition());

                telemetry.update();

                frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //setting the speed of the motors
                backleft.setPower(speed);
                backright.setPower(speed);
                frontleft.setPower(speed);
                frontright.setPower(speed);

                while (opModeIsActive() && frontleft.isBusy() && frontright.isBusy()) {
                    idle();
                }

            }
            private void level1 () {
                moveArm(33);
                extendArm(3);
                moveIntake(1);


            }

            private void level2 () {
                moveArm(69);
                extendArm(3);
                moveIntake(1);


            }

            private void level3 () {
                moveArm(90);
                extendArm(5);
                //moveArm(-0.5);
                moveIntake(1);

            }
            private void extendArm ( double distance){
                int d = (int) (67.2 * (distance * 25.4));

                int currentPos = extenderMotor.getCurrentPosition();
                //This will tell to the driver hub whats the robot is doing.
                telemetry.addData("extender position", currentPos);
                telemetry.addData("d", d);
                telemetry.update();

                extenderMotor.setTargetPosition(currentPos + d);
                extenderMotor.setPower(-1); // THis is negative to reverse the direction
                extenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //setting speed


                while (opModeIsActive() && extenderMotor.isBusy()) {
                    idle();
                }


            }


    /* The method moveArm help us know how many degrees we need to move (rotate)
    the arm by doing some math */


            private void moveArm ( int degrees){


                int ratio = (bigSprocket / smallSprocket);

                int total = ((ratio * armPulses) / 360);

                int pulses = (ratio * total);
                degrees = 44 * degrees;


                //int d = (int) ((distance-3) *((armPulses/Math.PI)/diamter));
                //drive(d,d,0.50);
                //int d = (int) (distance );

                //setting target position
                armMotor.setTargetPosition(degrees);
                //goign to position

                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //setting speed
                armMotor.setPower(1);

                while (opModeIsActive() && armMotor.isBusy()) {
                    idle();
                }


            }

            private void moveIntake(int seconds)
            {

                IntakeMotor.setPower(0.7);
                sleep((1000 * seconds));
                IntakeMotor.setPower(0);

            }

        }



