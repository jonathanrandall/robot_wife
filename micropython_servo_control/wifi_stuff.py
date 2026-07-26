import network
import socket
from machine import Pin, PWM


# Configure Wi-Fi connection
ssid = ''
password = ''


# # HTML Content
# html = """<!DOCTYPE html>
# <html>
# <head>
#     <title>Servo Control</title>
# </head>
# <body>
#     <h1>Servo Control</h1>
#     <input type="range" min="0" max="180" value="90" id="servoSlider" onchange="sendAngle(this.value)">
#     <script>
#         function sendAngle(angle) {
#             var xhr = new XMLHttpRequest();
#             xhr.open("GET", "/angle?value=" + angle, true);
#             xhr.send();
#         }
#     </script>
# </body>
# </html>
# """
# HTML Content
html = """<!DOCTYPE html>
<html>
  <head>
    <title>Pan-Tilt Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: Arial; text-align: center; margin: 0 auto; padding-top: 20px; }
      .control { margin: 30px auto; }
      label { font-size: 22px; font-weight: bold; display: block; margin-bottom: 10px; }
      .slider { -webkit-appearance: none; width: 400px; height: 20px; background: #FFD65C; outline: none; }
      .slider::-webkit-slider-thumb { -webkit-appearance: none; width: 35px; height: 35px; background: #003249; cursor: pointer; }
      .slider::-moz-range-thumb { width: 35px; height: 35px; background: #003249; cursor: pointer; }
      input[type=number] { width: 80px; font-size: 18px; padding: 4px; margin-left: 10px; }
    </style>
  </head>
  <body>
    <h1>Pan-Tilt Control</h1>

    <div class="control">
      <label>Pan</label>
      <input type="range" min="0" max="1000" value="500" step="1" class="slider"
             id="slider6" oninput="syncFromSlider(6)">
      <input type="number" min="0" max="1000" value="500" id="num6" onchange="syncFromNum(6)">
    </div>

    <div class="control">
      <label>Tilt</label>
      <input type="range" min="0" max="1000" value="500" step="1" class="slider"
             id="slider5" oninput="syncFromSlider(5)">
      <input type="number" min="0" max="1000" value="500" id="num5" onchange="syncFromNum(5)">
    </div>

    <script>
      function send(id, val) {
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/slider?value=" + id + "_" + val, true);
        xhr.send();
      }
      function syncFromSlider(id) {
        var val = document.getElementById("slider" + id).value;
        document.getElementById("num" + id).value = val;
        send(id, val);
      }
      function syncFromNum(id) {
        var val = parseInt(document.getElementById("num" + id).value);
        if (isNaN(val)) val = 500;
        if (val < 0) val = 0;
        if (val > 1000) val = 1000;
        document.getElementById("num" + id).value = val;
        document.getElementById("slider" + id).value = val;
        send(id, val);
      }
    </script>
  </body>
</html>
"""


# # Setup the web server
# addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
# s = socket.socket()
# s.bind(addr)
# s.listen(1)

# print('listening on', addr)

# while True:
#     cl, addr = s.accept()
#     print('client connected from', addr)
#     request = cl.recv(1024)
#     request = str(request)
#     print('request:', request)
    
#     if 'GET / ' in request:
#         response = html
#     elif 'GET /angle?value=' in request:
#         angle = int(request.split('value=')[1].split(' ')[0])
#         servo.write_angle(angle)
#         response = "Angle set to " + str(angle)
#     else:
#         response = "Invalid Request"
    
#     cl.send('HTTP/1.1 200 OK\n')
#     cl.send('Content-Type: text/html\n')
#     cl.send('Connection: close\n\n')
#     cl.sendall(response)
#     cl.close()

