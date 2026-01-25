package org.firstinspires.ftc.teamcode.season2025.Components;

public class BallAimer
{
    private double MaxRange = 150;
    private double AmountOfZones = 20;
    private double MaxPower = 2400;   // max launcher velocity
    private double MaxAngle = 1;      // max angle in radians or servo units?

    public double DesiredRange;
    public double DesiredPower;
    public double DesiredAngle;
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
        // Scale linearly by zone
        double powerPerZone = MaxPower / AmountOfZones;
        DesiredPower = powerPerZone * CurrentZone;
    }

    public void calculateDesiredAngle(){
        // Scale linearly by zone
        double anglePerZone = MaxAngle / AmountOfZones;
        DesiredAngle = anglePerZone * CurrentZone;
    }
}
