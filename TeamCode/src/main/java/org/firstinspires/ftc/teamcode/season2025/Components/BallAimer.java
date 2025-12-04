package org.firstinspires.ftc.teamcode.season2025.Components;

public class BallAimer
{
    private double MaxRange = 150;
    private double AmountOfZones = 8;
    private double MaxPower = 2400;
    private double MaxAngle = 1;
    private double DesiredRange;
    public double DesiredPower;
    public double DesiredAngle;
    public double CurrentZone;


    public void calculateCurrentZone(double _desiredRange){
        DesiredRange = _desiredRange;

        CurrentZone = DesiredRange / (MaxRange / AmountOfZones);
    }

    public void calculateDesiredPower(){
        DesiredPower = ((MaxPower / AmountOfZones) * DesiredRange) * CurrentZone;
    }

    public void calculateDesiredAngle(){
         DesiredAngle = ((MaxAngle / AmountOfZones) * DesiredAngle) * CurrentZone;
    }

}
