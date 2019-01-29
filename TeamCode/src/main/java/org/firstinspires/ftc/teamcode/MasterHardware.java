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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import android.os.Handler;
import android.os.SystemClock;
import android.util.Log;
import android.view.ViewParent;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class MasterHardware
{
    /* Public OpMode members. */
    public DcMotor        flDriveMtr        = null;
    public DcMotor        frDriveMtr        = null;
    public DcMotor        rlDriveMtr        = null;
    public DcMotor        rrDriveMtr        = null;
    public DcMotor        liftMtr           = null;
    public DcMotor        rotMtr1           = null;
    public DcMotor        rotMtr2           = null;
    public DcMotor        spoolMtr           = null;

    public DigitalChannel digitalBottomLow  = null;
    public DigitalChannel digitalBottomHigh = null;
    public DigitalChannel digitalTopLow     = null;
    public DigitalChannel digitalTopHigh    = null;
    public Servo          kickServo         = null;
    public Servo          boxServo          = null;
    public Servo           intakeServo       = null;
    public ModernRoboticsI2cGyro gyro = null;
    public ModernRoboticsI2cRangeSensor frRange = null;
    public ModernRoboticsI2cRangeSensor flRange = null;
    public ModernRoboticsI2cRangeSensor lfRange = null;
    public ModernRoboticsI2cRangeSensor lrRange = null;

    public static final int maxEncoder = -16250;
    public static final int dropEncoder = -13000;
    public static final int minEncoder = 0;
    public static final double KICK_OUT = 0.4;
    public static final double BOX_START = 0.0;
    public static final double BOX_COLLECT = 1.0;
    public static final double BOX_MIDDLE = 0.5;
    public static final double SWEEP_IN = 1.0;
    public static final double SWEEP_OUT = 0.0;
    public static final double SWEEP_STOP = 0.0;
    public static final double KICK_IN  = 0.15;
    public static final double LIFT_POWER=1.0;
    public static final double STOP_POWER=0.0;
    public static final double RESET_POWER=0.6;//May need to change for direction
    
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public MasterHardware(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        flDriveMtr = hwMap.get(DcMotor.class, "Front_Left");
        rrDriveMtr = hwMap.get(DcMotor.class, "Front_Right");
        rlDriveMtr = hwMap.get(DcMotor.class, "Rear_Left");
        frDriveMtr = hwMap.get(DcMotor.class, "Rear_Right");
        liftMtr    = hwMap.get(DcMotor.class, "lift");
        rotMtr1    = hwMap.get(DcMotor.class, "rot1");
        rotMtr2    = hwMap.get(DcMotor.class, "rot2");
        spoolMtr    = hwMap.get(DcMotor.class, "spool");
        
        //Define and Initialize Servos
        kickServo  = hwMap.get(Servo.class,"budda");
        boxServo  = hwMap.get(Servo.class,"box");
        //intakeServo  = hwMap.get(Servo.class,"sweep");
        
        // get a reference to our digitalTouch object.
        digitalBottomLow = hwMap.get(DigitalChannel.class, "bl");
        digitalBottomHigh = hwMap.get(DigitalChannel.class, "bh");
        digitalTopLow = hwMap.get(DigitalChannel.class, "tl");
        digitalTopHigh = hwMap.get(DigitalChannel.class, "th");

        // set the digital channel to input.
        digitalBottomLow.setMode(DigitalChannel.Mode.INPUT);
        digitalBottomHigh.setMode(DigitalChannel.Mode.INPUT);
        digitalTopLow.setMode(DigitalChannel.Mode.INPUT);
        digitalTopHigh.setMode(DigitalChannel.Mode.INPUT);
        
        
        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro"); 
        gyro.resetZAxisIntegrator();
        frRange = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range1");
        flRange = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range2");
        lfRange = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range3");
        lrRange = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range4");


        flDriveMtr.setDirection(DcMotor.Direction.FORWARD); 
        frDriveMtr.setDirection(DcMotor.Direction.REVERSE); 
        rlDriveMtr.setDirection(DcMotor.Direction.FORWARD); 
        rrDriveMtr.setDirection(DcMotor.Direction.REVERSE); 
        liftMtr.setDirection(DcMotor.Direction.REVERSE);
        rotMtr1.setDirection(DcMotor.Direction.FORWARD);
        rotMtr2.setDirection(DcMotor.Direction.REVERSE);
        spoolMtr.setDirection(DcMotor.Direction.REVERSE);

        
        driveStop();
        driveReset();
        liftStop();
        //liftReset();
        
        kickServo.setPosition(KICK_IN);
    }
    
    public void driveReset(){

        flDriveMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flDriveMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frDriveMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frDriveMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rlDriveMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlDriveMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rrDriveMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrDriveMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public void liftReset(){
        liftMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(digitalBottomLow.getState() && digitalBottomLow.getState() ){
            liftMtr.setPower(RESET_POWER);
        }//Run until either bottom limit switch is pressed
        liftStop();
        liftMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMtr.setTargetPosition(-500);
        while (liftMtr.isBusy()){
            liftMtr.setPower(RESET_POWER);
        };
        liftStop();
    }
    
    public void driveStop(){
        flDriveMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frDriveMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlDriveMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrDriveMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        
        flDriveMtr.setPower(STOP_POWER);
        frDriveMtr.setPower(STOP_POWER);
        rlDriveMtr.setPower(STOP_POWER);
        rrDriveMtr.setPower(STOP_POWER);        
    }
    
    public void liftStop(){
        liftMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMtr.setPower(STOP_POWER);
    }
    
    public void liftSet(int target){
        liftMtr.setTargetPosition(target);        
    }
    
    public void liftRun(double speed){
        liftMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMtr.setPower(Math.abs(speed));
    }
    
    public void driveRun(double speed){
        flDriveMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frDriveMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rlDriveMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rrDriveMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flDriveMtr.setPower(speed);
        frDriveMtr.setPower(speed);
        rlDriveMtr.setPower(speed);
        rrDriveMtr.setPower(speed);        
    }
 }

