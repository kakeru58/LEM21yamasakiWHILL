# ソケットを使うためにsocketをimportする。
import socket
import time
import os
import sys
import whillpy
import whill_data
whill = whillpy.connect(port='/dev/tty.usbserial-FT1WH33Q')
# 相手のIP
HOST = '192.168.1.12'
speed = 50
# ポートはサーバーで設定した9999に接続する。
PORT = 9999
# ソケットを生成する。
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# connect関数でサーバーに接続する。
client_socket.connect((HOST, PORT))
 
# 10回のループでsend、receiveをする。
while(1):
  # メッセージはhelloで送信
  data = client_socket.recv(1)
  # データを受信する。
  msg = int(data.decode())
  if(msg==2):
    msg=-1
          
  whill.move(straight=0, turn=int(msg * speed))
  time.sleep(0.001)
  # データをコンソールで出力する。
  print('Received from : ', msg)
# ソケットリソースを返却する。
client_socket.close()
