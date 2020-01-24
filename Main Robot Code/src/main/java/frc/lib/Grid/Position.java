package frc.lib.Grid;

public class Position {
    double x;
    double y;
    double z;

    /**
    * Sets the x, y, z position to 0.
    */
    public Position() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
    }

    /**
    * Sets the x, y, z position.
    *
    * @param x position.
    * @param y position.
    * @param z position.
    */
    public Position(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
    * Sets the x position.
    *
    * @param x position.
    */
    public void setX(double x) {
        this.x = x;
    }

    /**
    * Sets the y position.
    *
    * @param y position.
    */
    public void setY(double y) {
        this.y = y;
    }

    /**
    * Sets the z position.
    *
    * @param z position.
    */
    public void setZ(double z) {
        this.z = z;
    }

    /**
    * Changes the x position by specified value.
    *
    * @param value value position hav been changed by.
    */
    public void changeX(double value) {
        this.x += value;
    }

    /**
    * Changes the x position by specified value.
    *
    * @param value value position hav been changed by.
    */
    public void changeY(double value) {
        this.y += value;
    }

    /**
    * Changes the x position by specified value.
    *
    * @param value value position hav been changed by.
    */
    public void changeZ(double value) {
        this.z += value;
    }

    /**
    * Returns the x position.
    *
    * @return x position
    */
    public double getX() {
        return x;
    }

    /**
    * Returns the y position.
    *
    * @return y position
    */
    public double getY() {
        return y;
    }

    /**
    * Returns the y position.
    *
    * @return y position
    */
    public double getZ() {
        return y;
    }
}
