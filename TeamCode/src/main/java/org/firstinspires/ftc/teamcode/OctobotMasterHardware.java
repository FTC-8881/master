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
public class OctobotMasterHardware
{
    /* Public OpMode members. */
    public DcMotor        flDriveMtr        = null;
    public DcMotor        frDriveMtr        = null;
    public DcMotor        rlDriveMtr        = null;
    public DcMotor        rrDriveMtr        = null;


    public ModernRoboticsI2cGyro gyro = null;

    public static final double STOP_POWER=0.0;
    public static final double DRIVE_POWER=.70;
    public static final double STRAFE_POWER=.70;
    public static final double ROTATION_POWER=.70;
    public static final double WHEEL_DIA=100;
    public static final double WHEEL_BASE=225;
    public static final double CLICKS_PER_REV=288;
    public static final double PI=3.14159;
    public static final double CLICKS_PER_INCH_DIAG=CLICKS_PER_REV*25.4/(*PI*WHEEL_DIA);
    public static final double INCHES_PER_CLICK_DIAG=1/CLICKS_PER_INCH_DIAG;
    public static final double CLICKS_PER_DEGREE=(WHEEL_BASE*CLICKS_PER_REV)/WHEEL_DIA;
    public static final double CLICKS_PER_INCH_FWD=CLICKS_PER_INCH_DIAG/1.414;
    public static final double INCHES_PER_CLICK_FWD=1/CLICKS_PER_INCH_FWD
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public OctobotMasterHardware(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(OctobotHardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        flDriveMtr = hwMap.get(DcMotor.class, "lfdMtr");
        rrDriveMtr = hwMap.get(DcMotor.class, "rfdMtr");
        rlDriveMtr = hwMap.get(DcMotor.class, "lrdMtr");
        frDriveMtr = hwMap.get(DcMotor.class, "rrdMtr");
        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro.resetZAxisIntegrator();
        flDriveMtr.setDirection(DcMotor.Direction.REVERSE);
        frDriveMtr.setDirection(DcMotor.Direction.REVERSE); 
        rlDriveMtr.setDirection(DcMotor.Direction.FORWARD); 
        rrDriveMtr.setDirection(DcMotor.Direction.FORWARD);
        driveStop();
        driveReset();
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
    

    
    public void driveRun(double speed, int position){
        flDriveMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frDriveMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rlDriveMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rrDriveMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flDriveMtr.setPower(speed);
        frDriveMtr.setPower(speed);
        rlDriveMtr.setPower(speed);
        rrDriveMtr.setPower(speed);        
    }

    public void driveRun(double speed){
        flDriveMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frDriveMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlDriveMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrDriveMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flDriveMtr.setPower(speed);
        frDriveMtr.setPower(speed);
        rlDriveMtr.setPower(speed);
        rrDriveMtr.setPower(speed);
    }
    public void driveInchesFwd(double speed, double inches){
        driveStop();
        flEnc+=inches*CLICKS_PER_INCH_FWD;
        frEnc+=inches*CLICKS_PER_INCH_FWD;
        rlEnc+=inches*CLICKS_PER_INCH_FWD;
        rrEnc+=inches*CLICKS_PER_INCH_FWD;
        flDriveMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frDriveMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rlDriveMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rrDriveMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flDriveMtr.setPower(speed);
        frDriveMtr.setPower(speed);
        rlDriveMtr.setPower(speed);
        rrDriveMtr.setPower(speed);
    }
    public void driveInchesDiag(double speed, double inches){
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

