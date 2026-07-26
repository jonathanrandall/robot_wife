import sys
import json
import _thread
from machine import Pin, I2C, Timer
import time, gc
from micropython import const

###########

from Oled import OLED_I2C
import utime
oled_flag = True
i2c = I2C(0, scl=Pin(27), sda=Pin(14), freq=100000)
try:
  oled = OLED_I2C(128, 64, i2c)
except OSError:
  print('没有检测到OLED模块')
  oled_flag = False

###################
# print("Starting up...")

try:
  import usocket as socket
except:
  import socket
  
from time import sleep

from machine import Pin, I2C
import network

from wifi_stuff import ssid, password, html

import gc
gc.collect()

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)
while not wlan.isconnected():
    pass

ip_configuration = wlan.ifconfig()[0]
print("ip address", ip_configuration)

if oled_flag:
    oled.fill(0)
    oled.text("RED", 10, 11)
    oled.text("here now", 10,32)
    # oled.text(ip_configuration, 10,32)
    oled.show()
# 

########################################

from BusServo import BusServo

PAN_ID  = 6  # servo 6 = pan
TILT_ID = 5  # servo 5 = tilt
DEFAULT_MS = 500

bus_servo = BusServo(tx=26, rx=35, tx_en=25, rx_en=12)
bus_servo.run(PAN_ID, 500, DEFAULT_MS)
bus_servo.run(TILT_ID, 500, DEFAULT_MS)

name2bus_method = {name: getattr(bus_servo, name) for name in dir(bus_servo)
                   if callable(getattr(bus_servo, name)) and not name.startswith("__")}

from machine import UART
import sys, json

uart = UART(1, baudrate=115200, tx=17, rx=16)

def run_serial():
    buffer = ""
    while True:
        if uart.any():
            c = uart.read(1).decode()
            if c == '\n':
                line = buffer.strip()
                buffer = ""
                if not line:
                    continue
                try:
                    parts = line.split()
                    cmd = parts[0]
                    if cmd == 'p':
                        # p <pos> [ms]
                        pos = int(parts[1])
                        ms  = int(parts[2]) if len(parts) > 2 else DEFAULT_MS
                        bus_servo.run(PAN_ID, pos, ms)
                        uart.write("ok\n")
                    elif cmd == 't':
                        # t <pos> [ms]
                        pos = int(parts[1])
                        ms  = int(parts[2]) if len(parts) > 2 else DEFAULT_MS
                        bus_servo.run(TILT_ID, pos, ms)
                        uart.write("ok\n")
                    elif cmd == 'pt':
                        # pt <pan> <tilt> [ms]
                        pan_pos  = int(parts[1])
                        tilt_pos = int(parts[2])
                        ms       = int(parts[3]) if len(parts) > 3 else DEFAULT_MS
                        bus_servo.run(PAN_ID,  pan_pos,  ms)
                        bus_servo.run(TILT_ID, tilt_pos, ms)
                        uart.write("ok\n")
                    elif cmd == 'pos':
                        pan_pos  = bus_servo.get_position(PAN_ID)
                        tilt_pos = bus_servo.get_position(TILT_ID)
                        uart.write("%d %d\n" % (pan_pos, tilt_pos))
                    else:
                        uart.write("err unknown\n")
                except Exception as e:
                    uart.write("err %s\n" % str(e))
            else:
                buffer += c

_thread.start_new_thread(run_serial, ())


def run_webservice():
  # Setup the web server
  addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
  s = socket.socket()
  s.bind(addr)
  s.listen(1)

  print('listening on', addr)
  print('ip_configuration ', 'http://%s' %ip_configuration)
  
  while True:
    try:
      cl, addr = s.accept()
      print('client connected from', addr)
      request = cl.recv(1024)
      request = str(request)
      print('request:', request)     
      
      if 'GET / ' in request:
          response = html
          bus_servo.run(PAN_ID, 500, DEFAULT_MS)
          bus_servo.run(TILT_ID, 500, DEFAULT_MS)
      elif 'GET /slider?value=' in request:
          info = (request.split('value=')[1].split(' ')[0])
          servo_id = info.split('_')[0]
          servo_id = int(servo_id)
          angle = info.split('_')[1]
          angle = int(angle)

          bus_servo.run(servo_id,angle)
          # servo.write_angle(angle)
          response = "Servo ID set to " + str(servo_id) + " angle is " + str(angle)
          print("repsonse", response)
      elif 'GET /command?' in request:
          # Example request: GET /command?method=run&params=1,90
          command_info = request.split('command?')[1].split(' ')[0]
          command_params = {param.split('=')[0]: param.split('=')[1] for param in command_info.split('&')}
          method_name = command_params.get('method')
          params = command_params.get('params')
          if method_name in name2bus_method:
              try:
                  if params is None:
                    args = dict()
                  else:
                    args = [int(x) if x.isdigit() else x for x in params.split(',')]
                  if method_name == 'set_positions':
                    args = [args[:-1], args[-1]]
                  result = name2bus_method[method_name](*args)
                  response = json.dumps(result)
              except Exception as e:
                  response = "Error executing command: %s" %e 
                  sys.print_exception(e)
                  print('got error for params %s' % params)
                  
          else:
              response = "Invalid Command"
      else:
          response = "Invalid Request"          
      
      cl.send('HTTP/1.1 200 OK\n')
      cl.send('Content-Type: text/html\n')
      cl.sendall('Connection: keep-alive\n\n')
      cl.sendall(response)
      cl.close()
    except Exception as e:
      sys.print_exception(e) 


def run_socket():
    port = 5005
    addr = socket.getaddrinfo('0.0.0.0', port)[0][-1]
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(addr)
    print("Socket listening on", addr, 'and port', port)
    while True:
      try:
        data, addr = s.recvfrom(64)
        if data:
          req = data.decode().split('-')
          print(req)
          method_name = req[0]
          params = None if len(req) == 1 else req[1]
          print(params)
          if params is None:
              args = dict()
          else:
            args = [int(x) if x.isdigit() else x for x in params.split(',')]
          if method_name == 'set_positions':
            args = [args[:-1], args[-1]]
          result = name2bus_method[method_name](*args)
          response = json.dumps(result)
          s.sendto(response.encode(), addr)
      except Exception as e:
        sys.print_exception(e)

#_thread.start_new_thread(run_socket, ())
run_webservice()
