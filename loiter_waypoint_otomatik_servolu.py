from dronekit import connect, VehicleMode,Command
import time
from pymavlink import mavutil
import math
from geopy import distance
import numpy as np
connection_string = "127.0.0.1:14550"
#connection_string = "/dev/ttyUSB0"
iha = connect(connection_string,wait_ready=True, timeout=100, baud=57600)

font=4
yazi="TEK"
takeoff=6
sayac=0
iki_harf_arasi_bosluk=2


def arm_ol_ve_yuksel(irtifa):
    iha.mode = VehicleMode("GUIDED")
    while iha.mode != 'GUIDED':
        print('Guided moduna gecis yapiliyor')
        time.sleep(1.5)

    iha.armed = True
    while iha.armed is not True:
        print("iHA arm ediliyor...")
        time.sleep(0.5)

    print("iHA arm edildi.")
    iha.simple_takeoff(irtifa)

    while iha.location.global_relative_frame.alt < irtifa * 0.9:
        print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
        print("iha hedefe yukseliyor.")
        time.sleep(1)
    print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
    print("Takeoff gerceklesti")

def belli_noktadan_uzaklik(coord,uzaklik, direction):
    #uzaklik = 1         #Metre
    belli_noktadan_uzaklik = distance.distance(meters=uzaklik).destination((coord),bearing=direction)  #0 – North, 90 – East, 180 – South, 270 or -90 – West
    return belli_noktadan_uzaklik

def iki_nokta_arasi_uzaklik_hesaplama_2d(coord_1,coord_2):

    distance_2d = distance.distance(coord_1[:2], coord_2[:2]).m
    #print("2D - " +str(distance_2d))
    return distance_2d

def iki_nokta_arasi_uzaklik_hesaplama_3d(coord_1,coord_2):
    distance_3d = np.sqrt(iki_nokta_arasi_uzaklik_hesaplama_2d(coord_1,coord_2)**2 + (coord_1[2] - coord_2[2])**2)
    #print("3D - "+str(distance_3d))
    return distance_3d

def get_distance_metres(aLocation1, aLocation2):  #konumun LocationGlobalRelative icinde gelmesi gerekiyor.
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

konum_array=[[iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon,takeoff]]#iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon


def harf_ciz(array, sayac, nozzle):
    test_array = []
    if (array[0] != 0 and array[1] == 0):
        konum_yeni = belli_noktadan_uzaklik((konum_array[sayac][0], konum_array[sayac][1]), array[0], 0)
        test_array.append(konum_yeni.latitude)
        test_array.append(konum_yeni.longitude)
        test_array.append(konum_array[sayac][2])
        test_array.append(nozzle)
    elif (array[0] == 0 and array[1] != 0):
        test_array.append(konum_array[sayac][0])
        test_array.append(konum_array[sayac][1])
        test_array.append(konum_array[sayac][2] - array[1])
        test_array.append(nozzle)
    elif (array[0] != 0 and array[1] != 0):
        konum_yeni = belli_noktadan_uzaklik((konum_array[sayac][0], konum_array[sayac][1]), array[0], 0)
        test_array.append(konum_yeni.latitude)
        test_array.append(konum_yeni.longitude)
        test_array.append(konum_array[sayac][2] - array[1])
        test_array.append(nozzle)
    konum_array.append(test_array)


position_array_T = [[iki_harf_arasi_bosluk,0, 0],
                    [font, 0, 1],
                    [-font / 2, 0, 0],
                    [0,font,1],
                    [font / 2,-font, 0]]

position_array_E = [[iki_harf_arasi_bosluk,0, 0],
                    [0, font, 1],
                    [font / 2, 0, 1],
                    [0, -font/2, 0],
                    [-font / 2, 0, 1],
                    [0, -font / 2, 0],
                    [font / 2, 0, 1]]

position_array_K = [[iki_harf_arasi_bosluk, 0, 0],
                    [0, font, 1],
                    [font/2, 0, 0],
                    [-font / 2, -font / 2, 1],
                    [font / 2, -font / 2, 1]]

position_array_N = [[iki_harf_arasi_bosluk, 0, 0],
                    [0, font, 1],
                    [0, -font, 0],
                    [font/2, font, 1],
                    [0, -font, 1]]

position_array_O = [[iki_harf_arasi_bosluk,0, 0],
                    [0, font, 1],
                    [font/2, 0, 1],
                    [0, -font, 1],
                    [-font/2, 0, 1],
                    [font/2, 0, 0]]

position_array_F = [[iki_harf_arasi_bosluk, 0, 0],
                    [0, font, 1],
                    [0, -font / 2, 0],
                    [font / 4, 0,1],
                    [-font / 4, -font/2, 0],
                    [font / 2, 0, 1]]


position_array_S = [[iki_harf_arasi_bosluk, 0, 0],
                    [0, font, 0],
                    [font/2, 0, 1],
                    [0, -font / 2, 1],
                    [-font / 2, 0, 1],
                    [0,  -font / 2, 1],
                    [font/2, 0,  1]]
for i in yazi:
    if i == "T" or i == "t":
        for i in position_array_T:
            harf_ciz(i,sayac,i[2])
            sayac+=1
    if i == "E" or i == "e":
        for i in position_array_E:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "K" or i == "k":
        for i in position_array_K:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "N" or i == "n":
        for i in position_array_N:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "O" or i == "o" or i == "0":
        for i in position_array_O:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "F" or i == "f":
        for i in position_array_F:
            harf_ciz(i,sayac,i[2])
            sayac += 1
    if i == "S" or i == "s":
        for i in position_array_S:
            harf_ciz(i,sayac,i[2])
            sayac += 1

def gorev_ekle():
    iha.commands.clear()
    wait_time=1
    iha.commands.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0,
                0, 0, 0, 0, 8))

    for i in range(1,len(konum_array)):
        if(konum_array[i][3]):
            iha.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                0, 8, 1700, 0, 0, 0, 0, 0))
        iha.commands.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME, 0, 0,0,
                0, 0, 0, konum_array[i][0], konum_array[i][1],konum_array[i][2]))
        iha.commands.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                    0, 8, 1500, 0, 0, 0, 0, 0))
        iha.commands.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_CONDITION_DELAY, 0,
                    0, 1, 0, 0, 0, 0, 0, 0))
    # RTL
    iha.commands.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
                0, 0, 0, 0, 0, 0, 0, 0))

    # DOgRULAMA
    iha.commands.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
                0, 0, 0, 0, 0, 0, 0, 0))
    iha.commands.upload()

arm_ol_ve_yuksel(6)

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


