#coconut

# pyproj
import math
from pyproj import Geod
#GPS
import time
import serial
from micropyGPS import MicropyGPS
#9jiku
import wiringpi as pi
from smbus import SMBus
import datetime
import csv

#画像
import os
import cv2 as cv
import numpy as np

import smbus

import RPi.GPIO as GPIO    
import datetime

#bmx_setup
ACCL_ADDR = 0x19
ACCL_R_ADDR = 0x02
MAG_ADDR = 0x13
MAG_R_ADDR = 0x42
GYRO_ADDR = 0x69
GYRO_R_ADDR = 0x02
i2c = SMBus(1)

#prsI2C設定
i2c = smbus.SMBus(1)
address = 0x5C
#センサーの設定
i2c.write_byte_data(address, 0x20, 0x90)

# GPIOピンの定義
AIN1 = 26
#AIN2 = 16
AIN2 = 21
PWMA = 19
STBY = 20
BIN1 = 13
BIN2 = 6
PWMB = 5

#ポート番号の定義
Led_pin = 25  

def bmx_setup():
    # mag_data_setup : 地磁気値をセットアップ
    data = i2c.read_byte_data(MAG_ADDR, 0x4B)
    if(data == 0):
        i2c.write_byte_data(MAG_ADDR, 0x4B, 0x83)
        time.sleep(0.5)
    i2c.write_byte_data(MAG_ADDR, 0x4B, 0x01)
    i2c.write_byte_data(MAG_ADDR, 0x4C, 0x00)
    i2c.write_byte_data(MAG_ADDR, 0x4E, 0x84)
    i2c.write_byte_data(MAG_ADDR, 0x51, 0x04)
    i2c.write_byte_data(MAG_ADDR, 0x52, 0x16)
    time.sleep(0.5)

def acc_value():
    data = [0, 0, 0, 0, 0, 0]
    acc_data = [0.0, 0.0, 0.0]
    try:
        for i in range(6):#各軸に関して2byteずつ存在している
            data[i] = i2c.read_byte_data(ACCL_ADDR, ACCL_R_ADDR + i) #1byteよんだら1byte隣に追加
        for i in range(3): #3軸
            acc_data[i] = ((data[2*i + 1] * 256) + int(data[2*i] & 0xF0)) / 16 
            if acc_data[i] > 2047: #+-
                acc_data[i] -= 4096
            acc_data[i] *= 0.0098 
    except IOError as e: #例外処理
        print("I/O error({0}): {1}".format(e.errno, e.strerror))
    return acc_data

def mag_value():
    data = [0, 0, 0, 0, 0, 0, 0, 0]
    mag_data = [0.0, 0.0, 0.0]
    try:
        for i in range(8):
            data[i] = i2c.read_byte_data(MAG_ADDR, MAG_R_ADDR + i)
        for i in range(3):
            if i != 2:
                mag_data[i] = ((data[2*i + 1] * 256) + (data[2*i] & 0xF8)) / 8
                if mag_data[i] >4095:
                    mag_data[i] -= 8192
            else:
                mag_data[i] = ((data[2*i + 1] * 256) + (data[2*i] & 0xFE)) / 2
                if mag_data[i] > 16383:
                    mag_data[i] -= 32768
    except IOError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))
    return mag_data

def gyro_value():
    data = [0, 0, 0, 0, 0, 0]
    gyro_data = [0.0, 0.0, 0.0]
    try:
        for i in range(6):
            data[i] = i2c.read_byte_data(GYRO_ADDR, GYRO_R_ADDR + i)
        for i in range(3):
            gyro_data[i] = (data[2*i + 1] * 256) + data[2*i]
            if gyro_data[i] > 32767:
                gyro_data[i] -= 65536
            gyro_data[i] *= 0.0038
    except IOError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))
    return gyro_data

def fix():
    print('=====offset start======')
    mag_data = []
    #ループ変数。v=0でlistのx成分v=1でlistのy成分を参照 
    v=0
    #0回目取得で無条件にmax,minにいれる
    i=0
    #最大値と最小値用,最終的なオフセット用リスト。x.y方向あるから各々2データ
    max_buffer = [0,0]
    min_buffer = [0,0] 
    fix_value = [0,0]

    #rangeの値は適当
    for value in range(70):
        mag_data = mag_value()
        time.sleep(0.5)
        
        #デバッグ
        # print('mag_data:' ,(mag_data))

        #ここからmagのリストのインデックス(xかyか)を見てる
        while(v<2):

            #iが1週ごとに回るからtmpは配列である必要なし
            tmp = mag_data[v]

            #一番最初の取得は何も入ってないから無条件でmaxとminに入る
            if(i==0):
                max_buffer[v] = tmp
                min_buffer[v] = tmp
            #1番最初以外はすでにmax.minに値が入ってるから判定してあげる
            else:
                if(tmp > max_buffer[v]):
                    max_buffer[v] = tmp
                if(tmp < min_buffer[v]):
                    min_buffer[v] = tmp
            v = v+1
        #v++の位置は間違えないように
        #vリセット忘れずに。iで何回目か
        v=0
        i = i+1

    #v=0でリストのx成分。v=1でリストのy成分を参照するためのloop
    v=0
    while(v<2):
        fix_value[v] = (max_buffer[v] + min_buffer[v])/2
        v = v+1
    
    #終了処理
    print("Stop!!")
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 0 )
    pi.softPwmWrite( PWMB, 0 )

    return fix_value

def GPS():
    print('=====GPS start======')
    # シリアル通信設定
    uart = serial.Serial('/dev/serial0', 9600, timeout = 10)
    # gps設定
    my_gps = MicropyGPS(9, 'dd')

    # 10秒ごとに表示
    tm_last = 0
    while True:
        sentence = uart.readline()
        if len(sentence) > 0:
            for x in sentence:
                if 10 <= x <= 126:
                    stat = my_gps.update(chr(x))
                    if stat:
                        tm = my_gps.timestamp
                        tm_now = (tm[0] * 3600) + (tm[1] * 60) + int(tm[2])
                        if (tm_now - tm_last) >= 10:
                            # print('=' * 20)
                            #print(my_gps.date_string(), tm[0], tm[1], int(tm[2]))
                            print("latitude:", my_gps.latitude[0], ", longitude:", my_gps.longitude[0])
                            #緯度経度が0以外の値が入ったらbreak
                #2個目のforのループを抜けたいから2個目のループの部分でbreak。forの部分はインデントで下げてるからforのましたではなく、forのtab1つ分の下
                if(my_gps.longitude[0] != 0 and my_gps.latitude[0] != 0):
                    break
        #最初のwhileループを抜けたいからWhileの部分でbreak。while部分はインデントで下げてるからwhileのましたではなく、whileのtab1つ分下
        if(my_gps.longitude[0] != 0 and my_gps.latitude[0] != 0):
            break
    return my_gps.longitude[0], my_gps.latitude[0]

def get_azimuth_distance():
    print('=====目標の方位角・距離を取得します=======')
    #葛飾講義棟内
    # obj_latitude = 35.771728333333336
    # obj_longitude = 139.86355666666665
        
    #goal座標、新宿未来公園の図書館側
    #obj_latitude = 35.77111166666667
    #obj_longitude = 139.86134833333332

    #goal座標,金町駅側
    # obj_latitude = 35.77056833333333 
    # obj_longitude = 139.86360833333333

    #goal,野田正門
    #obj_latitude = 35.91742166666667
    #obj_longitude = 139.90671833333334

    #goal,薬学部の方
    #obj_latitude = 35.92145333333333
    #obj_longitude = 139.91102666666666

    #goal,図書館前
    # obj_latitude = 35.91807166666667
    # obj_longitude = 139.90844

    #EtoE1回目
    # obj_latitude = 35.918205
    # obj_longitude = 139.90842666666666

    #2/16 にじゅくみらいこうえん
#    obj_latitude = 35.771231666666665
#    obj_longitude = 139.86195
    
    #3/4デバッグ
    # obj_latitude = 30.373891666666665 #30.374021666666668
    # obj_longitude = 130.960425 #130.96046
    
    #3/5試し
    obj_latitude = 30.374296666666666
    obj_longitude = 130.960115


    #現在地座標
    p1_longitude, p1_latitude = GPS()

    g = Geod(ellps='WGS84')
    # 戻り値は方位角(azimuth)、反方位角(back_azimuth)、距離(distance)の順番
    azimuth, back_azimuth, distance = g.inv(p1_longitude, p1_latitude, obj_longitude, obj_latitude)
    result = g.inv(p1_longitude, p1_latitude, obj_longitude, obj_latitude)
    azimuth = result[0]
    back_azimuth = result[1]
    distance = result[2]
    if(azimuth > -180 and azimuth <0):
        azimuth = azimuth + 360
    return azimuth, distance


def get_theta(azimuth,offset):

    #発散回避した時に変な値が入るのを回避
    theta = 360
    #磁気代入
    mag = mag_value()
    #arktanの補正
    if(mag[0]-offset[0] != 0):
        if(mag[0]-offset[0] < 0 and mag[1] - offset[1] > 0):
            rad1 = math.atan((mag[1]-offset[1])/(mag[0]-offset[0]))
            deg1 = math.degrees(rad1)
            azimuth_sat = deg1 + 180
        elif(mag[0] - offset[0] < 0 and mag[1] - offset[1] < 0):
            rad2 = math.atan((mag[1]-offset[1])/(mag[0]-offset[0]))
            deg2 = math.degrees(rad2)
            azimuth_sat = deg2 +180
        elif(mag[0]-offset[0] > 0 and mag[1] - offset[1] < 0):
            rad3 = math.atan((mag[1]-offset[1])/(mag[0]-offset[0]))
            deg3 = math.degrees(rad3)
            azimuth_sat = deg3 + 360
        else:
            rad4 = math.atan((mag[1]-offset[1])/(mag[0]-offset[0]))
            deg4 = math.degrees(rad4)
            azimuth_sat = deg4

        #(ii) absが180以上
        if(abs(azimuth-azimuth_sat) > 180):
            theta = 360 - abs(azimuth-azimuth_sat)
        #(i)補正なし
        else:
            theta = abs(azimuth-azimuth_sat)
        
        
        print('機体に対する方位角', azimuth_sat)
        # print('mag', mag)
    if(mag[0]-offset[0] == 0):
        print("分母が0になりました。theta=360を返します")
        print("magx",mag[0])
        print("offset_x",offset[0])
        print("mag-offset",mag[0]-offset[0])
    return theta

def menseki():
    #BBMで使ったプログラム：カメラモジュールで撮った画像ファイルを"EtoE.jpg"として保存する。以下BBM用プラグラムコピペ
    cap = cv.VideoCapture(0,cv.CAP_V4L2)
    ret, frame = cap.read()
    cv.imwrite('EtoE.jpg', frame)
    cap.release()

    #time.sleep(5)

    #画像データの読み込み
    img = cv.imread("EtoE.jpg")

    #BGR色空間からHSV色空間への変換
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    #色検出しきい値の設定
    lower = np.array([0,64,0])
    upper = np.array([5,255,255])

    #色検出しきい値範囲内の色を抽出するマスクを作成
    frame_mask1 = cv.inRange(hsv, lower, upper)

    #色検出しきい値の設定
    lower = np.array([160,64,0])
    upper = np.array([179,255,255])

    #色検出しきい値範囲内の色を抽出するマスクを作成
    frame_mask2 = cv.inRange(hsv, lower, upper)

    frame_mask = frame_mask1 + frame_mask2

    #論理演算で色検出
    dst = cv.bitwise_and(img, img, mask=frame_mask)

    #cv.imshow("img", dst)
    cv.imwrite('EtoE_after.jpg', dst)

    #面積を求める
    #画像の読み込み
    img=cv.imread('EtoE_after.jpg',0)
    ret1,img_th=cv.threshold(img,0,255,cv.THRESH_OTSU)
    #全体の画素数
    whole_area=img_th.size
    #赤部分の画素数
    Red_area=cv.countNonZero(img_th)
    #黒部分の画素数
    black_area=whole_area-Red_area
    Red = Red_area/whole_area*100

    #それぞれの割合を表示
    print('Red_Area='+str(Red)+'%')
    return Red

def zyushin():
    #画像処理（赤色の重心を求める）
    #from PIL import Image
    img = cv.imread("EtoE_after.jpg")

    img = 255-img
    cv.imwrite("BW.jpg", img)
    img = cv.imread("BW.jpg", 0)

    ret, EE1 = cv.threshold(img,250,255,cv.THRESH_BINARY_INV)

    # 重心を計算
    m = cv.moments(EE1)
    cog_x = int(m["m10"] / m["m00"])
    cog_y = int(m["m01"] / m["m00"])
    print(cog_x, cog_y)
    #画素 640*480

    # 重心点を画像に描画
    #center = (cog_x, cog_y)
    #radius = 5
    #color = (100, 0, 0)
    #cv.circle(EE1, center, radius, color, thickness=5)

    #画像を保存
    #cv.imwrite("img_change.jpg", EE1)
    return cog_x


def triangle(m ,h):
    #イメージを読み込む
    img = cv.imread("EtoE_after.jpg")
    #img = 255-img

    #グレースケールに変換
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    #ノイズの除去（ただし、赤コーンが小さすぎると認識できない。ただしksizeの値をいじれば対応可能）
    height, width = gray.shape
    ksize = int(max(height, width) * h)
    ksize = ksize // 2 * 2 + 1
    #オープニング（収縮後、膨張）
    gray = cv.morphologyEx(gray, cv.MORPH_OPEN, np.ones((ksize, ksize),dtype='uint8'))
    #cv.imwrite('gray.jpg', gray)

    height, width = gray.shape
    ksize = int(max(height, width) * 0.02)
    ksize = ksize // 2 * 2 + 1
    #クロージング（膨張後、収縮）
    gray = cv.morphologyEx(gray, cv.MORPH_CLOSE, np.ones((ksize, ksize), dtype='uint8'))
    #cv.imwrite('gray2.jpg', gray)
    
    #Canny法によるエッジの検出
    edge = cv.Canny(gray, 100, 200)

    #エッジの強調
    height, width = edge.shape
    ksize = int(max(height, width) * 0.01)
    ksize = ksize // 2 * 2 + 1
    #クロージング（膨張後、収縮）
    edge = cv.morphologyEx(edge, cv.MORPH_CLOSE, np.ones((ksize, ksize), dtype='uint8'))

    #輪郭線の抽出
    contours, _ = cv.findContours(edge, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    print('{:d} contours detected!'.format(len(contours)))
    result = img.copy()
    cv.drawContours(result, contours, -1, (255, 0, 0), 3, cv.LINE_AA)

    #輪郭線の分類
    result = img.copy()
    font = cv.FONT_HERSHEY_SIMPLEX
    triangle = 0
    for cnt in contours:
        # 輪郭線の長さを計算
        arclen = cv.arcLength(cnt, True)
        # 輪郭線の近似　数値を変える＝近似の精度を変える　値が小さいと小さな角も認識する
        approx = cv.approxPolyDP(cnt, arclen * m, True)
        # 何角形かをみる
        cv.drawContours(result, [approx], -1, (255, 0, 0), 3)
        #triangle = 0
        n_gon = len(approx)
        text = str(n_gon)
        if n_gon == 3:
            text = 'triangle'
            #print("found a cone")
            triangle = triangle + 1

        position = np.asarray(approx).reshape((-1, 2)).max(axis=0).astype('int32')
        px, py = position
        cv.putText(result, text, (px + 10, py + 10), font, 1.0, (255, 255, 255), 2, cv.LINE_AA)


    cv.imwrite('detection.jpg', result[:, :, ::-1])
    print("triangle=",triangle)
    return triangle , len(contours)


def motasetup():
    # GPIOピンの設定
    pi.wiringPiSetupGpio()
    pi.pinMode( AIN1, pi.OUTPUT )
    pi.pinMode( AIN2, pi.OUTPUT )
    pi.pinMode( PWMA, pi.OUTPUT )
    pi.pinMode( STBY, pi.OUTPUT )
    pi.pinMode( BIN1, pi.OUTPUT )
    pi.pinMode( BIN2, pi.OUTPUT )
    pi.pinMode( PWMB, pi.OUTPUT )

    # PWM端子に接続したGPIOをPWM出力できるようにする
    pi.softPwmCreate( PWMA, 0, 100 )
    pi.softPwmCreate( PWMB, 0, 100 )

def forward(PWM_forward):
    #距離から比例制御してPWMの価を渡す
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 1 )
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 0 )
    pi.digitalWrite( BIN2, 1 )
    pi.softPwmWrite( PWMB, 0 )
    #SW on
    pi.digitalWrite( STBY, 1 )
    # print("movefoward")
    pi.softPwmWrite( PWMA, PWM_forward )
    pi.softPwmWrite( PWMB, PWM_forward )
    #time.sleep(3) 

def back():
        #距離から比例制御してPWMの価を渡す
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 0 )
    pi.digitalWrite( AIN2, 1 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 1 )
    pi.digitalWrite( BIN2, 0 )
    pi.softPwmWrite( PWMB, 0 )
    #SW on
    pi.digitalWrite( STBY, 1 )
    # print("movefoward")
    pi.softPwmWrite( PWMA, 100 )
    pi.softPwmWrite( PWMB, 100 )


def backspin_R(PWM_spin):
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 1 )
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 1 )
    pi.digitalWrite( BIN2, 0 )
    pi.softPwmWrite( PWMB, 0 )
    #SW on
    pi.digitalWrite( STBY, 1 )

    #真理値変えるためにsetup関数を呼ぶのを忘れないように
    #theatが10°ぐらいまで回転
    # print("backspin_R")
    pi.softPwmWrite( PWMA, PWM_spin )
    pi.softPwmWrite( PWMB, PWM_spin )


def backspin_L(PWM_spin): 
    #自動制御させる分からpwmの値を渡す。その値用の変数用意
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 0 )
    pi.digitalWrite( AIN2, 1 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 0 )
    pi.digitalWrite( BIN2, 1 )
    pi.softPwmWrite( PWMB, 0 )
    #SW on
    pi.digitalWrite( STBY, 1 ) 

    #真理値変えるためにsetup関数を呼ぶのを忘れないように
    #theatが10°ぐらいまで回転
    # print("backspin_L")
    pi.softPwmWrite( PWMA, PWM_spin )
    pi.softPwmWrite( PWMB, PWM_spin )

def backspin_gazou():

    # スタンバイ状態にする
    #前進真理値
    #print("StandBy")
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN1, 1 )
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 1 )
    pi.digitalWrite( BIN2, 0 )
    pi.softPwmWrite( PWMB, 0 )
    #SW on
    pi.digitalWrite( STBY, 1 ) 

    #真理値変えるためにsetup関数を呼ぶのを忘れないように
    #theatが10°ぐらいまで回転
    print("backspin")
    pi.softPwmWrite( PWMA, 40 )
    pi.softPwmWrite( PWMB, 40 )

def stop():
    # print("Stop!!")
    pi.digitalWrite( STBY, 0 )
    pi.digitalWrite( AIN2, 0 )
    pi.softPwmWrite( PWMA, 0 )
    pi.digitalWrite( BIN1, 0 )
    pi.softPwmWrite( PWMB, 0 )

def forward_stack():
    #スタック回数、リセットの意味もある
    stack = 1
    acc = acc_value()
    while(acc[2] < 0):
        if(stack % 3 == 0):
            print("スタック判定-後退")
            back()
            time.sleep(1)
            stop()
        else:
            print("スタック判定-前進")
            forward(100)
            time.sleep(1)
            forward(80)
            time.sleep(0.5)
            forward(50)
            time.sleep(0.5)
            forward(30)
            time.sleep(0.5)
            stop()
        #とまってzとる
        time.sleep(1)
        acc = acc_value()
        stack = stack + 1

def makecontent(name,data):
    dt = datetime.datetime.now()
    return str(dt)+str("----")+str(name)+str("----")+str(data)+"\n"

def housyutu():
    #一時的な値です
    prs = 10000
    prs_before = 10000
    prs_after = 1023

    #何回目のループか知りたい
    i=0

    #閾値以上の変化を計測した回数
    m=0

    while m < 3:#数値は適当
        #データ読み込み
        pxl = i2c.read_byte_data(address, 0x28)
        pl = i2c.read_byte_data(address, 0x29)
        ph = i2c.read_byte_data(address, 0x2A)
        tl = i2c.read_byte_data(address, 0x2B)
        th = i2c.read_byte_data(address, 0x2C)

        #データ変換
        prs = ph << 16 | pl << 8 | pxl

        #物理量に変換
        prs = prs / 4096

    #一時停止。時間は要実験。()内単位は秒。
        time.sleep(1)

        if i>0:
            prs_after = (prs - prs_before)
            print(i,m,prs_after)        
            if prs_after>0.3 :#数値は適当
                m = m+1
            with open("sensor.txt","a",encoding='utf-8') as file:
                file.write(makecontent("気圧差", prs_after))         

        prs_before = prs


        i=i+1

    #d放出判定成功！
    print('Successful release!')

def tyakuti():
    #一時的な値です
    prs = 1023
    prs_before = 1023
    prs_after = 1023

    #何回目のループか知りたい
    i=0

    while True:
        #データ読み込み
        pxl = i2c.read_byte_data(address, 0x28)
        pl = i2c.read_byte_data(address, 0x29)
        ph = i2c.read_byte_data(address, 0x2A)
        tl = i2c.read_byte_data(address, 0x2B)
        th = i2c.read_byte_data(address, 0x2C)

        #データ変換
        prs = ph << 16 | pl << 8 | pxl

        #物理量に変換
        prs = prs / 4096

        #一時停止
        time.sleep(1)

        if i>0:
            prs_after = abs(prs - prs_before)
            print(prs_after)
            with open("sensor.txt","a",encoding='utf-8') as file:
                file.write(makecontent("気圧差", prs_after))         

        prs_before = prs
            

        if prs_after<0.05:
            print('Successful landing!')
            break

        i=i+1

#main関数-端末からスクリプトを実行した時のみ
if __name__ == '__main__':
    #ログファイル作成
    with open("sensor.txt","w",encoding='utf-8') as file:
        file.write(makecontent("-", "センサーログの作成を開始します"))
    with open("control.txt","w",encoding='utf-8') as file:
        file.write(makecontent("-", "制御ログの作成を開始します"))

    #放出判定
    housyutu()
    with open("control.txt","a",encoding='utf-8') as file:
        file.write(makecontent("-", "放出判定成功")) 

    #着地判定
    tyakuti()   
    with open("control.txt","a",encoding='utf-8') as file:
        file.write(makecontent("-", "着地判定成功")) 
    
    time.sleep(15)

    #溶断回路
    #GPIOの設定
    GPIO.setmode(GPIO.BCM)              #GPIOのモードを"GPIO.BCM"に設定
    GPIO.setup(Led_pin, GPIO.OUT)       #GPIO18を出力モードに設定

    #GPIOの電圧を制御
    GPIO.output(Led_pin, GPIO.HIGH)     #GPIO18の出力をHigh(3.3V)にする
    time.sleep(2)                       #2秒間待つ
    GPIO.output(Led_pin, GPIO.LOW)      #GPIO18の出力をLow(0V)にする

    #GPIOをクリーンアップ
    GPIO.cleanup()
    with open("control.txt","a",encoding='utf-8') as file:
        file.write(makecontent("-", "溶断回路成功"))

    #前進・旋回出力定義
    # PWM_off  = 80
    PWM_spin = 1
    # PWM_forward = 60
    # #モーターセットアップ
    motasetup()

#----------------------------------------パラシュート分離--------------------------------------------
    #パラシュートから離れる
    forward(100)
    time.sleep(3)
    forward(70)
    time.sleep(0.3)
    forward(50)
    time.sleep(0.3)
    forward(30)
    time.sleep(0.3)
    stop()
    with open("control.txt","a",encoding='utf-8') as file:
        file.write(makecontent("-", "パラシュート分離")) 
#----------------------------------------オフセットもろもろ----------------------
    #駆動
    bmx_setup()
    #現在地座標
    p1_longitude, p1_latitude = GPS()
    #オフセット定義
    offset = [0,0]
    #角速度定義
    gyro = [0,0,0]
    with open("control.txt","a",encoding='utf-8') as file:
        file.write(makecontent("-", "オフセット取得"))     
    #オフセット用回転
    backspin_R(80)
    #オフセット算出
    offset =  fix()
    with open("sensor.txt","a",encoding='utf-8') as file:
        file.write(makecontent("オフセット", offset)) 
#-----------------------------距離。ゴール方位角------------------------------------

    #距離・ゴールに対する方位角取得
    azimuth, distance = get_azimuth_distance()
    print("ゴールに対する方位角:",azimuth)
    print("距離:",distance)
    with open("sensor.txt","a",encoding='utf-8') as file:
        file.write(makecontent("距離", distance)) 

    #条件分岐誤作動防止
    theta = 360
    #奇数回目か偶数回目か
    i=1

#------------------------------駆動------------------------------------------------
    while(True):
        print('turn start')
        #thetaリセット
        theta = 360
        theta = get_theta(azimuth, offset)
        with open("sensor.txt","a",encoding='utf-8') as file:
            file.write(makecontent("方位角差", theta)) 

        while(theta>30):
            print('機体の方位角、方位角差を取得します')
            #旋回・奇数回目と偶数回目で向き変える
            if(i%2==0):
                backspin_L(PWM_spin)
            elif(i%2 == 1):            
                backspin_R(PWM_spin)
            #with open("control.txt","a",encoding='utf-8') as file:
            #    file.write(makecontent("-", "旋回"))

            #動いていない状態で角速度取るの回避

            if(PWM_spin>0 and PWM_spin<30):            
                time.sleep(0.5)
            else:
                time.sleep(0.25)
            gyro = gyro_value()     
            if(abs(gyro[2]) < 1.5):
                print('PWMを5上昇させます')
                print('PWM now:',PWM_spin)
                PWM_spin = PWM_spin + 5
                print('change +PWM:',PWM_spin)
                print('旋回を再度開始します')
                continue
            # time.sleep(0.5)
            stop()
            #if_gyroの中にリセット無くていい気がする。100でないと抜けられないかもしれない。60でリセットするとスタック回避できないかも
            #gyroをぬけてからリセットで十分に間に合う。
            if(PWM_spin > 60):
                PWM_spin = 1
                continue

            #z軸加速度、ひっくり返る前にacc取るの防ぐ
            time.sleep(1)
            forward_stack()

            #止まる前にはかることを防ぐ
            time.sleep(1)
            theta = get_theta(azimuth, offset)
            print('方位角差:', theta)
            #方位角さみる
            #time.sleep(3)
            with open("control.txt","a",encoding='utf-8') as file:
                file.write(makecontent("-", "旋回"))
            with open("sensor.txt","a",encoding='utf-8') as file:
                file.write(makecontent("方位角差", theta)) 
#------------------------------前進処理、----------------------------------------------------------------
        with open("control.txt","a",encoding='utf-8') as file:
            file.write(makecontent("-", "前進-"))
        forward(80)
        if(distance>30):
            print("15秒。distance",distance)
            time.sleep(15)
        elif(distance<=30):
            print("5秒。distance",distance)
            time.sleep(5)
        forward(50)
        time.sleep(0.5)
        forward(30)
        time.sleep(0.5)
        stop()
#--------------------------z軸スタック===============================================================
        #前進終了後にz軸見る
        #z軸加速度
        forward_stack()
#----------------------------------------------------------------------------------------------------

        #カウントアップして交互に右左旋回させる
        i=i+1

#---------------------------------------距離再計測------------------------------------------------

        azimuth, distance = get_azimuth_distance()
        with open("sensor.txt","a",encoding='utf-8') as file:
            file.write(makecontent("距離", distance)) 
        if(distance<5):
            break

    #-----------------------------------駆動終わって前に進む-------------------------------------
    time.sleep(1)
    theta = get_theta(azimuth, offset)
    while(theta>20):
        print("駆動終了後の方位角さ",theta)
        backspin_L(PWM_spin)
        if(PWM_spin>0 and PWM_spin<30):            
            time.sleep(0.5)
        else:
            time.sleep(0.25)
        gyro = gyro_value()     
        if(abs(gyro[2]) < 1.5):
            print('PWMを5上昇させます')
            print('PWM now:',PWM_spin)
            PWM_spin = PWM_spin + 5
            print('change +PWM:',PWM_spin)
            print('旋回を再度開始します')
            continue
        # time.sleep(0.5)
        stop()
        theta = get_theta(azimuth, offset)
        #if_gyroの中にリセット無くていい気がする。100でないと抜けられないかもしれない。60でリセットするとスタック回避できないかも
        #gyroをぬけてからリセットで十分に間に合う。
        if(PWM_spin > 60):
            PWM_spin = 1
            continue

    forward(80)
    time.sleep(3)
    forward(50)
    time.sleep(0.5)
    forward(30)
    time.sleep(0.5)
    stop()
    print("駆動後1ド前進")
    with open("control.txt","a",encoding='utf-8') as file:
        file.write(makecontent("-", "前進-"))



    #画像処理
    print('画像処理に移行します')
    with open("control.txt","a",encoding='utf-8') as file:
        file.write(makecontent("-", "画像処理に移行"))


    Red = 0
    i = 1
    k = 0
    cone = 0
    m = 0.05
    h = 0.005
    while(True):
        #赤色の比率をRedに代入
        Red = menseki()
        with open("sensor.txt","a",encoding='utf-8') as file:
            file.write(makecontent("赤面積", Red))
        if Red<=0.05:
            backspin_gazou()
            with open("control.txt","a",encoding='utf-8') as file:
                file.write(makecontent("-", "赤の面積0.05以下より旋回"))
            time.sleep(0.3)#値は適当。回す時間
            stop()
            time.sleep(0.3)
        

        elif(Red>=0.05 and Red<=5):#値は適当
            k = 0
            h = 0.005
            while k < 100:
                cone , kazu = triangle(m,h)
                if kazu > 5:
                    h = h + 0.001
                
                else :
                    break
                k = k + 1
            
            print("noise OK!!" , "size =" + str(h))
            cone , kazu = triangle(m,h)
            with open("sensor.txt","a",encoding='utf-8') as file:
                file.write(makecontent("赤コーンの数", cone))

            if cone == 0 :
                #if i%4 == 0:
                m = m + 0.01
                print("kinji =", m)#追加
                backspin_gazou()
                with open("control.txt","a",encoding='utf-8') as file:
                    file.write(makecontent("-", "赤コーンの数が0より旋回"))
                time.sleep(0.3)#値は適当。回す時間
                stop()
                time.sleep(0.3)
                i = i + 1
            
            elif cone > 0 :
                with open("control.txt","a",encoding='utf-8') as file:
                    file.write(makecontent("-", "赤コーンを発見し,直進"))
                forward(80)
                time.sleep(1.5)#値は適当
                forward(50)
                time.sleep(0.5)
                forward(30)
                time.sleep(0.5)
                stop()
                time.sleep(0.3)
                m = 0.05
            
        #画像は640*480、0<=cog_x<=640
        
        elif (Red>5 and Red <= 40):
            cog_x = zyushin()
            with open("sensor.txt","a",encoding='utf-8') as file:
                file.write(makecontent("重心のx座標", cog_x))
            if(cog_x>=160 and cog_x<=480):
                with open("control.txt","a",encoding='utf-8') as file:
                    file.write(makecontent("-", "重心が中央にあるため直進"))
                forward(80)
                time.sleep(1)#値は適当forward(80)
                time.sleep(1.5)#値は適当
                forward(50)
                time.sleep(0.5)
                forward(30)
                time.sleep(0.5)
                stop()
                time.sleep(0.3)
            
            elif(cog_x<160 or cog_x>480):
                with open("control.txt","a",encoding='utf-8') as file:
                    file.write(makecontent("-", "重心が外側にあるため旋回"))
                backspin_gazou()
                time.sleep(0.3)#値は適当。回す時間
                stop()
                time.sleep(0.3)
        
        
        elif(Red>40):#重心の値もゴール判定に入れるかどうか
            with open("control.txt","a",encoding='utf-8') as file:
                file.write(makecontent("-", "赤が4割よりゴール判定"))
            break

    print("goal!")

#お腹すいた
#ゼロメートルゴールしたいなあ
