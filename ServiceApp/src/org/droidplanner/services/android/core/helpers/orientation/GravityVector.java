package org.droidplanner.services.android.core.helpers.orientation;

/**
 * Created by mzqu on 8/6/2015.
 */
public class GravityVector extends Vector3f{
    public static final float G=9.89f;
    public static final float TOL_G=0.02f;

    /**
     * Initialises the vector with the given values
     *
     * @param x the x-component
     * @param y the y-component
     * @param z the z-component
     */
    public GravityVector(float x, float y, float z) {
        super(x,y,z);
    }

    /**
     * Instantiates a new vector3f.
     */
    public GravityVector() {
        super();
    }


    /**
     * Copy constructor
     */
    public GravityVector(Vector3f vector) {
        super(vector);
    }

    /**
     * Initialize with a float[]
     *
     */
    public GravityVector(float[] v) {
        super(v);
    }

}
