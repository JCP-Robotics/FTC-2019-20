/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous
//@Disabled
public class FTC_First_Competition_Autonomous_2019_2020_Blue extends LinearOpMode {

    //TensorFlow copyPaste
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    //Size thresholds
    private static final double MIDDLE_LOW_RANGE = 0;
    private static final double MIDDLE_HIGH_RANGE = 700;
    private static final double RIGHT_LOW_RANGE = 400;
    private static final double RIGHT_HIGH_RANGE = 1100;

    private static final double DOUBLE_BLOCK_LENGTH_THRESHOLD = 900;

    // Declare OpMode members.
    private ElapsedTime timer = new ElapsedTime();
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private DcMotor motor4 = null;
    private DcMotor motor5 = null;
    private Servo servo0 = null;



    //TensorFlow copyPaste
    private static final String VUFORIA_KEY = "AaEs8tL/////AAABmSQCoNB1r0qxhMgCrweZk2BqmBvmUfiL05sYG8vJILvFrw82V0ONqyny44tkIdi0wNc+kdiKy0b+nHx79Yzs9SZqLeyaioIst5SRiW27XbmaGdl3ogbQggEvOjh5qLp1+CmW5AJL13WDyglKGLnkReiQ37Kls+rcmW1q7VPFUsa2pS9QvFoLbsGirG+kx7USGjzBAlv7hJhcijUmpcPDXDSVFaBUqqvl25GwTHOpJA+v9StXd5pVOEXi0HEBwcuSz0ACjm65Oyq1c9vrayeShSW5W3JeS2zjKm56PsnLCmQjTJWJixrDvfGM7gppUw2aN1ZNqS4Xz/oP4afV7Blt0/Vv786+EpBUdW41lDTAss+P";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        motor5 = hardwareMap.get(DcMotor.class, "motor5");
        servo0 = hardwareMap.get(Servo.class, "servo0");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        // Wait for the game to start (driver presses PLAY)
        servo0.setPosition(0);

        boolean center = false;
        boolean right = false;
        boolean left = false;
        boolean goldDetected = false;
        boolean autonomousCompleted = false;

        waitForStart();

        if (opModeIsActive()) {
            if (tfod != null) {
                tfod.activate();
            }
            while (opModeIsActive()) {
                if(!autonomousCompleted) {
                    verticalMove(-0.5, 0.9);
                    wait(1.0);
                    horizontalMove(0.5,0.375);
                }
                while (!(center || right || left)&& opModeIsActive()){
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size() >= 1) {
                                int goldX;
                                int goldX2;
                                int goldLength = -1;
                                double goldMidpoint = -1;
                                for (Recognition recognition : updatedRecognitions) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        if (Math.abs(recognition.getTop() - recognition.getBottom()) > 400) {
                                            goldX = (int) recognition.getTop();
                                            goldX2 = (int) recognition.getBottom();
                                            goldLength = Math.abs(goldX2 - goldX);
                                            goldDetected = true;
                                            goldMidpoint = (double) (goldX + goldX2) / 2;
                                            telemetry.addData("midpoint", goldMidpoint);

                                        }

                                        if(goldDetected) {
                                            if (goldLength > DOUBLE_BLOCK_LENGTH_THRESHOLD) {
                                                left = true;
                                            } else if (goldMidpoint >= 450) {
                                                right = true;
                                            } else if (goldMidpoint < 450) {
                                                center = true;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        if (left) {
                            telemetry.addData("Left: ", true);
                            //servo0.setPosition(0.99986);
                            //verticalMove(0.5,0.81);
                            //horizontalMove(0.5,2.0);
                        } else if (center) {
                            telemetry.addData("Center: ", true);
                            //servo0.setPosition(0.99986);
                            //verticalMove(0.5,0.81);
                            //horizontalMove(0.5,2.0);
                        } else if (right) {
                            telemetry.addData("Right: ", true);
                            //servo0.setPosition(0.99986);
                            //verticalMove(0.5,0.81);
                            //horizontalMove(0.5,2.0);
                        }
                        telemetry.update();
                    }
            }
                autonomousCompleted = true;
            }
    }

        if (tfod != null) {
            tfod.shutdown();
        }
    }


    private void wait(double t) {
        timer.reset();

        while (timer.seconds()<t && opModeIsActive()){


        }

    }


    private void verticalMove(double power, double t) {
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(-1*power);
        motor4.setPower(-1*power);
        timer.reset();
        while (timer.seconds() < t && opModeIsActive()){

        }
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
    private void horizontalMove(double power, double t) {

        motor1.setPower(power);
        motor2.setPower(-1*power);
        motor3.setPower(-1*power);
        motor4.setPower(power);
        timer.reset();
        while (timer.seconds() < t && opModeIsActive()){

        }
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
    private void rotate(double power, double t) {

        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
        motor4.setPower(power);
        timer.reset();
        while (timer.seconds() < t && opModeIsActive()){

        }
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}
