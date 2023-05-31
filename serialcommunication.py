import serial
import time

# Konfigurasi port Serial
ser = serial.Serial('COM9', 9600) # Ganti dengan nama port Serial yang sesuai
time.sleep(5) # Tunggu 2 detik agar port Serial stabil

coords = [(1,1), (2,2), (3, 3), (4, 4), (5, 5), (6,6), (7, 7), (8, 8), 
          (9, 9), (10, 10), (11, 11), (12, 12), (13, 13), (14, 14), (15, 15), (16, 16), (17,17), (18, 18),
          (19,19), (20,20), (21,21), (23, 23), (24, 24), (25, 25), (26, 26), (27, 27), (28, 28), (29, 29),
          (30, 30),(31, 31), (32, 32), (33, 33), (34, 34), (35, 35), (36, 36),  (37, 37), (38, 38), (39, 39),
          (40, 40), (41, 41), (42, 42), (43, 43), (44, 44), (45, 45), (46,46), (47, 47), (48, 48), (49,49),
          (50, 50), (51, 51), (52, 52), (53,53), (54,54), (55, 55), (56,56), (57,57), (58,58), (59,59),
          (60,60), (61,61), (62,62), (63,63), (64,64), (65,65), (66,66), (67,67), (68,68), (69,69), (70, 70),
          (71,71), (72,72), (73,73), (74,74), (75,75), (76,76), (77,77), (78,78), (79,79), (80,80), (81,81),
          (82,82), (83,83), (84,84), (85,85), (86,86), (87,87), (88,88), (89,89), (90,90)]


# Mengirimkan koordinat ke Arduino
for coord in coords:
    # Membuat string dengan format "<nilai servo 1>,<nilai servo 2>\n"
    message = str(coord[0]) + ',' + str(coord[1]) + '\n'
    print(message)
    
    # Mengirimkan string ke Arduino
    ser.write(message.encode())
    
    time.sleep(0.1)
    # Menunggu 0.5 detik sebelum mengirim koordinat selanjutnya
    # time.sleep(0000.1)