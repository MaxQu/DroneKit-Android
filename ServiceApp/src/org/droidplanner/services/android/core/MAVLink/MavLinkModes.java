package org.droidplanner.services.android.core.MAVLink;

import org.droidplanner.services.android.core.drone.variables.ApmModes;
import com.MAVLink.common.msg_mission_item;
import com.MAVLink.common.msg_set_position_target_global_int;
import com.MAVLink.common.msg_gcs_gesture_ypr_local_bf;
import com.MAVLink.common.msg_set_mode;
import com.MAVLink.enums.MAV_CMD;
import com.MAVLink.enums.MAV_FRAME;
import com.o3dr.services.android.lib.drone.property.Attitude;
import com.o3dr.services.android.lib.model.ICommandListener;

import org.droidplanner.services.android.core.drone.autopilot.MavLinkDrone;

public class MavLinkModes {

    private static final int MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE = ((1 << 0) | (1 << 1) | (1 << 2));
    private static final int MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE = ((1 << 3) | (1 << 4) | (1 << 5));
    private static final int MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE = ((1 << 6) | (1 << 7) | (1 << 8));

    public static void setGuidedMode(MavLinkDrone drone, double latitude, double longitude, double d) {
        msg_mission_item msg = new msg_mission_item();
        msg.seq = 0;
        msg.current = 2; // TODO use guided mode enum
        msg.frame = MAV_FRAME.MAV_FRAME_GLOBAL;
        msg.command = MAV_CMD.MAV_CMD_NAV_WAYPOINT; //
        msg.param1 = 0; // TODO use correct parameter
        msg.param2 = 0; // TODO use correct parameter
        msg.param3 = 0; // TODO use correct parameter
        msg.param4 = 0; // TODO use correct parameter
        msg.x = (float) latitude;
        msg.y = (float) longitude;
        msg.z = (float) d;
        msg.autocontinue = 1; // TODO use correct parameter
        msg.target_system = drone.getSysid();
        msg.target_component = drone.getCompid();
        drone.getMavClient().sendMavMessage(msg, null);
    }

    public static void sendGuidedPosition(MavLinkDrone drone, double latitude, double longitude, double altitude){
        msg_set_position_target_global_int msg = new msg_set_position_target_global_int();
        msg.type_mask = MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE | MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        msg.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
        msg.lat_int = (int) (latitude * 1E7);
        msg.lon_int = (int) (longitude * 1E7);
        msg.alt = (float) altitude;
        msg.target_system = drone.getSysid();
        msg.target_component = drone.getCompid();
        drone.getMavClient().sendMavMessage(msg, null);
    }

    public static void sendGuidedVelocity(MavLinkDrone drone, double xVel, double yVel, double zVel){
        msg_set_position_target_global_int msg = new msg_set_position_target_global_int();
        msg.type_mask = MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE | MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        msg.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
        msg.vx = (float) xVel;
        msg.vy = (float) yVel;
        msg.vz = (float) zVel;
        msg.target_system = drone.getSysid();
        msg.target_component = drone.getCompid();
        drone.getMavClient().sendMavMessage(msg, null);
    }

    public static void sendGuidedPositionAndVelocity(MavLinkDrone drone, double latitude, double longitude, double altitude,
                                                     double xVel, double yVel, double zVel){
        msg_set_position_target_global_int msg = new msg_set_position_target_global_int();
        msg.type_mask = MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
        msg.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
        msg.lat_int = (int) (latitude * 1E7);
        msg.lon_int = (int) (longitude * 1E7);
        msg.alt = (float) altitude;
        msg.vx = (float) xVel;
        msg.vy = (float) yVel;
        msg.vz = (float) zVel;
        msg.target_system = drone.getSysid();
        msg.target_component = drone.getCompid();
        drone.getMavClient().sendMavMessage(msg, null);
    }

    public static void changeFlightMode(MavLinkDrone drone, ApmModes mode, ICommandListener listener) {
        msg_set_mode msg = new msg_set_mode();
        msg.target_system = drone.getSysid();
        msg.base_mode = 1; // TODO use meaningful constant
        msg.custom_mode = mode.getNumber();
        drone.getMavClient().sendMavMessage(msg, listener);
    }

    public static void sendGCSGesture(MavLinkDrone drone, Attitude gcsAttFreeze, Attitude gcsAttDiff) {
        msg_gcs_gesture_ypr_local_bf msg = new msg_gcs_gesture_ypr_local_bf();
        msg.time_boot_ms = 0L;
        //msg.target_system = drone.getSysid();
        msg.target_system = 1;
        //msg.target_component = drone.getCompid();
        msg.target_component = 0;
        //msg.coordinate_frame = MAV_FRAME.MAV_FRAME_BODY_NED;
        msg.coordinate_frame = 8;
        //msg.type_mask = MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE | MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        msg.type_mask = 455;
        msg.x = 0.0f;
        msg.y = 0.0f;
        msg.z = 0.0f;
        msg.vx = 0.0f;
        msg.vy = 0.0f;
        msg.vz = 0.0f;
        msg.afx = 0.0f;
        msg.afy = 0.0f;
        msg.afz = 0.0f;
        msg.lyaw = (float) gcsAttFreeze.getYaw();
        msg.lpitch = (float) gcsAttFreeze.getPitch();
        msg.lroll = (float) gcsAttFreeze.getRoll();
        msg.yaw = (float) gcsAttFreeze.getYaw()+ (float) gcsAttDiff.getYaw();
        msg.pitch = (float) gcsAttFreeze.getPitch()+ (float) gcsAttDiff.getPitch();
        msg.roll = (float) gcsAttFreeze.getRoll()+ (float) gcsAttDiff.getRoll();
        msg.vyaw = 0.0f;
        msg.vpitch = 0.0f;
        msg.vroll = 0.0f;
        drone.getMavClient().sendMavMessage(msg, null);

    }
}
