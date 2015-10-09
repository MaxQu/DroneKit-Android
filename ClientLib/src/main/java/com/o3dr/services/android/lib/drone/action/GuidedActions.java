package com.o3dr.services.android.lib.drone.action;

import com.o3dr.services.android.lib.util.Utils;

/**
 * Created by Fredia Huya-Kouadio on 1/19/15.
 */
public class GuidedActions {

    //Private to prevent instantiation
    private GuidedActions(){}

    public static final String ACTION_DO_GUIDED_TAKEOFF = Utils.PACKAGE_NAME + ".action.DO_GUIDED_TAKEOFF";
    public static final String EXTRA_ALTITUDE = "extra_altitude";

    public static final String ACTION_SEND_GUIDED_POINT = Utils.PACKAGE_NAME + ".action.SEND_GUIDED_POINT";
    public static final String EXTRA_GUIDED_POINT = "extra_guided_point";
    public static final String EXTRA_FORCE_GUIDED_POINT = "extra_force_guided_point";

    public static final String ACTION_SET_GUIDED_ALTITUDE = Utils.PACKAGE_NAME + ".action.SET_GUIDED_ALTITUDE";

    public static final String ACTION_FOLLOW_GCS_GESTURE = Utils.PACKAGE_NAME + ".action.FOLLOW_GCS_GESTURE";
    public static final String EXTRA_GCS_ATTITUDE = "extra_gcs_attitude";
    public static final String EXTRA_FORCE_GCS_ATTITUDE = "extra_force_gcs_attitude";
    public static final String EXTRA_GCS_ATTITUDE_LOCKED = "extra_gcs_attitude_locked";

    public static final String ACTION_RC_OVERRIDE = Utils.PACKAGE_NAME + ".action.RC_OVERRIDE";
    public static final String EXTRA_RC_CHANNEL = "extra_rc_channel";
    public static final String EXTRA_RC_VALUE = "extra_rc_value";
}
