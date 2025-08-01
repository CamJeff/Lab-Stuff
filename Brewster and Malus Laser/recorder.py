import serial
import serial.tools.list_ports as lp
import time

mode = 'transparent'
# mode = 'polarised'
delayTime = 10000 # delay between stepping and reading from analogIn

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




file_path = f'data/{mode}_{delayTime}_{time.strftime("%Y_%m_%d-%H_%M_%S")}.txt'
file = open(file_path, 'w')
def log(data):
    file.write(data.strip() + '\n')
    file.flush()
    print('logged [', data.strip(), ']')

buffer = ''
s = serial.Serial()
try:
    s.port = arduino_port
    s.baudrate = 115200
    s.open()

    s.write(bytes('100\n', 'utf-8'))

    # read back data
    while True:
        while s.in_waiting > 0:
            piece = s.read()
            try:
                buffer += piece.decode('utf-8')
            except:
                print('read malformed data:', piece)

        while '\n' in buffer:
            idx = buffer.index('\n')
            packet = ''.join(buffer[:idx+1]).strip()
            buffer = buffer[idx+1:]

            if packet.startswith('PD Analog read: '):
                log(packet[len('PD Analog read: '):])
            else:
                print('received (but didn\'t log):', packet)
                if 'Input a number of steps' in packet:
                    time.sleep(1)
                    print('sending steps')
                    if mode == 'transparent':
                        s.write(bytes(f'100 {delayTime}\n', 'utf-8'))
                    else:
                        s.write(bytes(f'400 {delayTime}\n', 'utf-8'))
                if "done" in packet:
                    if mode == 'transparent':
                        s.write(bytes(f'-100 0\n', 'utf-8'))
                    exit()

except KeyboardInterrupt:
    pass
finally:
    file.close()
    s.close()
    print('Exiting')

