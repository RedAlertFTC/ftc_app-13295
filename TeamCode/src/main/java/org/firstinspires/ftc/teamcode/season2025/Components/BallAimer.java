package org.firstinspires.ftc.teamcode.season2025.Components;

public class BallAimer
{
    private double MaxRange = 123.5;
    private double AmountOfZones = 20;
    private double MaxPower = 1300;   // max launcher velocity
    private double MinPower = 1100;
    private double MaxAngle = 0.55;      // max angle in radians or servo units?

    public double DesiredRange;
    public double DesiredPower;
    public double DesiredAngle;
    public double PowerChange;
    public double PowerPerZone;
    public double PowerAddition;
    public int CurrentZone;

    public void calculateCurrentZone(double _desiredRange){
        DesiredRange = _desiredRange;

        // Determine zone:
        double zoneSize = MaxRange / AmountOfZones;
        CurrentZone = (int)Math.ceil(DesiredRange / zoneSize);

        // clamp
        CurrentZone = Math.max(1, Math.min(CurrentZone, (int)AmountOfZones));
    }

    public void calculateDesiredPower(){
        PowerChange = MaxPower - MinPower;
         PowerPerZone = PowerChange / AmountOfZones;
         PowerAddition = PowerPerZone * CurrentZone;
         DesiredPower = MinPower + PowerAddition;
    }

    public void calculateDesiredAngle(){
        // Scale linearly by zone
//        double anglePerZone = MaxAngle / AmountOfZones;
//        DesiredAngle = anglePerZone * CurrentZone;

    }
}
