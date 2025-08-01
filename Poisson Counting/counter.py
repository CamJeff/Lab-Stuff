import serial
import serial.tools.list_ports as lp
import time

mode = 'count'
# mode = 'interval'

ports = lp.comports()
arduino_port = None
for p in ports:
    print(p.device, p.description)
    if 'arduino' in p.description.lower():
        arduino_port = p.device
        print('\t⬆️  identified as arduino')

if arduino_port is None:
    print('No arduino found, exiting')
    exit()




file_path = f'data/{mode}_{time.strftime("%Y_%m_%d-%H_%M_%S")}.txt'
file = open(file_path, 'w')
def log(data):
    file.write(data.strip() + '\n')
    file.flush()
    print('logged [', data.strip(), ']')



buffer = ''
mode_set = False

s = serial.Serial()
try:
    s.port = arduino_port
    s.baudrate = 115200
    s.open()

    # read back data
    while True:
        while s.in_waiting > 0:
            piece = s.read()
            try:
                buffer += piece.decode('utf-8')
            except:
                print('read malformed data:', piece)
            
        if not mode_set:
            s.write(bytes(mode + '\n', 'utf-8'))

        while '\n' in buffer:
            idx = buffer.index('\n')
            packet = ''.join(buffer[:idx+1]).strip()
            buffer = buffer[idx+1:]
            
            if packet.startswith(mode):
                log(packet)
            else:
                print('received (but didn\'t log):', packet)
                if 'mode set' in packet:
                    mode_set = True

except KeyboardInterrupt:
    file.close()
    s.close()
    print('Exiting')

