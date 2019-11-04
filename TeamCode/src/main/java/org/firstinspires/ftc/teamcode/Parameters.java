package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Parameters {
    public static double kV = 1 / 2.05; //Units are V*s/in
    public static double kVIntercept = 0.449268; //Units are V
    public static double[] kVelocityPID = {0, 0, 0};
    public static double kWheelDiameter = 3.54; //Units are inches
    public static double kTicksPerRev = 537.6; //HD HEX 20:1
    public static double kInchesPerTick = (kWheelDiameter * Math.PI) / kTicksPerRev;
    public static double kTickPerInches = kTicksPerRev / (kWheelDiameter * Math.PI);

    public static double targetDistance = 60;
    public static double targetVelocity = 15;
    public static String kVuforiaKey = "AVXpNNv/////AAABmepa4M4Lb0Pji1YbsYaCl+8r5OjeMfRao7RA3siS0+MS1SCSgar/W48rh74JxmrcUczMHiI+i8exMdTIZBPGeBGIMgzq3zGckJ9v/5ry7uBCy4Db99U5jbTy4i+5VqGPzEYWhQgtD7t+3VoWWENGQiuawU33tAp4dHWPpe4gCbjF+MgTSp/SRrVZKPw4soYuoCr8JHhUbTiAFWMS4n3+P0Cxr+lxoYu9leKvaf1fiG9nEnb1uGf88N5UnaH5oA3uJ6KGS1ATgzRQIEkrTJElphxQb4zjOK8FyA+ERiVjJ6m7vwvmyokN6Dicm3xwS1cRPy3EBPrXTgM0TdhUWFVx30TqWrBz4KvEhNEnwzoFwHLM";
    // todo: write your code here
}