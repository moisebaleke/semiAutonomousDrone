from dronekit import connect
from dronekit import Command
from pymavlink import mavutil


def planSimpleMission(vehicle, destinationX, destinationY, destinationZ):
    #destinationX, destinationY, destinationZ are loongitude, latitude and altitude of the destination (floating points)

    cmds = vehicle.commands
    cmds.clear()

    # Get the set of commands from the vehicle
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    # Takeoff command
    takeoff_cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                          mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0,
                          0, 0, destinationZ)
    cmds.add(takeoff_cmd)

    # Waypoint command - fly to target
    waypoint_cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                           mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                           destinationX, destinationY, destinationZ)
    cmds.add(waypoint_cmd)

    # Return to launch command
    rtl_cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                      mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0,
                      0, 0, 0)
    cmds.add(rtl_cmd)

    # Upload the mission
    cmds.upload()

    # Upload mission
    cmds.upload()

    return