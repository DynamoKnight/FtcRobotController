package org.firstinspires.ftc.teamcode.Objects;

class Main {
    public static void main(String[] args) {
        Stage   map       = new Stage();
        Camera  vision    = new Camera();
        Motor   wheel     = new Motor();
        Robot   robo      = new Robot(map, vision, wheel);

        Distance distance = robo.getDistance(vision.findArtifact());

        robo.drive(distance);

        System.out.println( distance );

        System.out.println("Hello world!");
    }
}