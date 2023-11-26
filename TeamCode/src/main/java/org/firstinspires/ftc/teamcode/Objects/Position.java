package org.firstinspires.ftc.teamcode.Objects;

public class Position {
    //////////////////////////////
    // PROPERTIES
    //////////////////////////////
    private int x;
    private int y;

    ///////////////////////////////
    // CONSTRUCTOR(S)
    ///////////////////////////////
    public Position() {
    }

    public Position(int x, int y) {
        this.x = x;
        this.y = y;
    }

    ////////////////////////////////
    // METHODS
    ////////////////////////////////

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public void setX(int x) {
        this.x = x;
    }

    public void setY(int y) {
        this.y = y;
    }

    public void addX(){
        this.x++;
    }
    public void addY(){
        this.y++;
    }
    public void setPos(Position p){
        this.x = p.getX();
        this.y = p.getY();
    }

    @Override
    public String toString() {
        System.out.println("[" + x + "," + y + "]");
        return super.toString();
    }
}