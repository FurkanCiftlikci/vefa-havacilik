from dronekit import connect,Command
import time
from pymavlink import mavutil

connection_string = "/dev/ttyACM0"
iha = connect(connection_string,wait_ready=True, timeout=100, baud=57600)

def gorev_ekle():
    iha.commands.clear()
    iha.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                0, 8, 1900, 0, 0, 0, 0, 0))
    iha.commands.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_CONDITION_DELAY, 0,
                0, 5, 0, 0, 0, 0, 0, 0))
    iha.commands.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                    0, 8, 1500, 0, 0, 0, 0, 0))
    # DOgRULAMA
    iha.commands.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
                0, 0, 0, 0, 0, 0, 0, 0))
    iha.commands.upload()

gorev_ekle()

iha.commands.next = 0

iha.mode = "AUTO"

while True:
    next_waypoint = iha.commands.next

    print("Siradaki komut {}".format(next_waypoint))
    time.sleep(1)

    if next_waypoint is iha.commands.count-1:
        print("Gorev bitti.")
        break


