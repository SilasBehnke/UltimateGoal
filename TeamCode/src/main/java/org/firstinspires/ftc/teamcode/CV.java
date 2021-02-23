//Randomization A
//Move Wobble
//Park on Launch Line

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;

//This auto was made by SiB on 12/2/2020

@Autonomous(name = "CV")

public class CV extends LinearOpMode {


    public void runOpMode() //when you press init
    {
        RingDetectorV3 RingDetect = new RingDetectorV3("red",hardwareMap,telemetry,1,1,1);

        RingDetect.init();

        waitForStart(); //waits for t6he start button


        double rings = RingDetect.getRingPosition();
        telemetry.addData("Rings",rings);
    }
}
