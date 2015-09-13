package com.o3dr.services.android.lib.drone.property;

import android.os.Parcel;

/**
 * Created by fhuya on 10/28/14.
 */
public class Vector3 implements DroneAttribute {

    private  double x;
    private  double y;
    private  double z;

    public Vector3(){}

    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setZ(double z) {
        this.z = z;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Vector3)) return false;

        Vector3 vec = (Vector3) o;

        if (Double.compare(vec.y, y) != 0) return false;
        if (Double.compare(vec.x, x) != 0) return false;
        if (Double.compare(vec.z, z) != 0) return false;

        return true;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        temp = Double.doubleToLongBits(x);
        result = (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(y);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(z);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public String toString() {
        return "Vector3 {" +
                "x=" + x +
                ", y=" + y +
                ", z=" + z +
                '}';
    }


    @Override
    public int describeContents() {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel dest, int flags) {
        dest.writeDouble(this.x);
        dest.writeDouble(this.y);
        dest.writeDouble(this.z);
    }

    private Vector3(Parcel in) {
        this.x = in.readDouble();
        this.y = in.readDouble();
        this.z = in.readDouble();
    }

    public static final Creator<Vector3> CREATOR = new Creator<Vector3>() {
        public Vector3 createFromParcel(Parcel source) {
            return new Vector3(source);
        }

        public Vector3[] newArray(int size) {
            return new Vector3[size];
        }
    };
}
