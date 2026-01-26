package org.firstinspires.ftc.teamcode.season2025.Components;

public enum LightColor
{
    OFF(0),
    RED(0.277),
    ORANGE(0.333),
    YELLOW(0.388),
    SAGE(0.444),
    GREEN(0.5),
    AZURE(0.555),
    BLUE(0.611),
    INDIGO(0.666),
    VIOLET(0.722),
    WHITE(1);

    private final double lightColor;

    public double getLightColor(){
        return lightColor;
    }

    private LightColor(double lightColor){
        this.lightColor = lightColor;
    }
}
