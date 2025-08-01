import json
import matplotlib.pyplot as plt
import os
import serial
import serial.tools.list_ports as lp
import struct
import toml

from dataclasses import dataclass, fields
from typing import Optional, List, Tuple
import heapq


# ---------------------- seach for optimal PID parameters ----------------------

class Node:
    def __init__(self, bounds, corners):
        self.bounds = bounds
        self.center = tuple((b[0] + b[1]) / 2 for b in bounds)
        self.corners = corners
        self.center_val = 999999

    def split(self):
        # Choose the axis with the greatest range.
        ranges = [b[1] - b[0] for b in self.bounds]
        axis = ranges.index(max(ranges))

        print("Splitting along axis", axis, "at", self.center)

        # Create new bounds by bisecting along the chosen axis.
        left_bounds = list(self.bounds)
        right_bounds = list(self.bounds)
        left_bounds[axis] = (self.bounds[axis][0], self.center[axis])
        right_bounds[axis] = (self.center[axis], self.bounds[axis][1])
        left_bounds, right_bounds = tuple(left_bounds), tuple(right_bounds)

        def get_val(corner):
            return self.corners[corner]

        # For the left child, guess new corner values on the "upper" side of the splitting axis.
        corners_left = {}
        for i in (0, 1):
            for j in (0, 1):
                for k in (0, 1):
                    idx = [i, j, k]
                    if idx[axis] == 1:  # need to guess value along this axis
                        low_corner = list(idx); low_corner[axis] = 0
                        high_corner = list(idx); high_corner[axis] = 1
                        val_low = get_val(tuple(low_corner))
                        val_high = get_val(tuple(high_corner))
                        corners_left[(i, j, k)] = (self.center_val + val_low + val_high) / 3
                    else:
                        corners_left[(i, j, k)] = get_val((i, j, k))

        # For the right child, guess new corner values on the "lower" side of the splitting axis.
        corners_right = {}
        for i in (0, 1):
            for j in (0, 1):
                for k in (0, 1):
                    idx = [i, j, k]
                    if idx[axis] == 0:  # need to guess value along this axis
                        low_corner = list(idx); low_corner[axis] = 0
                        high_corner = list(idx); high_corner[axis] = 1
                        val_low = get_val(tuple(low_corner))
                        val_high = get_val(tuple(high_corner))
                        corners_right[(i, j, k)] = (self.center_val + val_low + val_high) / 3
                    else:
                        corners_right[(i, j, k)] = get_val((i, j, k))

        return Node(left_bounds, corners_left), Node(right_bounds, corners_right)

    def center_point(self):
        return tuple((b[0] + b[1]) / 2 for b in self.bounds)

    best_centre_val = 99999
    def value(self):
        if self.center_val < Node.best_centre_val:
            Node.best_centre_val = self.center_val
        # bias towards exploring larger volumes
        volume = (self.bounds[0][1] - self.bounds[0][0]) \
                * (self.bounds[1][1] - self.bounds[1][0]) \
                * (self.bounds[2][1] - self.bounds[2][0])

        v = (self.center_val - Node.best_centre_val) / Node.best_centre_val
        return (v + 0.1) / volume

    def __lt__(self, other):
        return self.value() < other.value()

# ------------------------------ packet processing -----------------------------

class Packet: pass

class SystemReadyPacket(Packet):
    def __str__(self):
        return "Packet(SystemReady)"

@dataclass
class CommandReceivedPacket(Packet):
    command: str

    def __str__(self):
        return f"Packet(CommandReceived: {self.command})"

@dataclass
class DataPacket(Packet):
    time: int
    temperature: float
    heater: float

    def __str__(self):
        return (f"Packet(time={self.time} ms, "
                f"temperature={self.temperature:.3f}c, "
                f"heater={100 * self.heater:.1f}%)")

def decode(line: str) -> Optional[Packet]:

    if line == "SYSTEM READY": return SystemReadyPacket()
    if line.startswith("COMMAND RECEIVED:"): return CommandReceivedPacket(line[len("COMMAND RECEIVED:"):].strip())
    try:
        data = {}
        for key, val in json.loads(line).items():
            if isinstance(val, str):
                # there's probably some nice programmatic way to do this, but
                # we only need a few cases.
                if key.endswith('_u16') or key.endswith('_u32') or key.endswith('_u64'):
                    data[key[:-4]] = int(val, 16)
                elif key.endswith('_f32'):
                    data[key[:-4]] = struct.unpack('!f', struct.pack('!I', int(val, 16)))[0]
            else:
                data[key] = val

        # Check we have all the required fields, and no more
        packet_fields = {f.name for f in fields(DataPacket)}
        data_fields = set(data.keys())

        extra_fields = data_fields - packet_fields
        if extra_fields: raise ValueError(f"Unrecognized fields in packet: {extra_fields}")

        missing_fields = packet_fields - data_fields
        if missing_fields: raise ValueError(f"Missing required fields: {missing_fields}")

        return DataPacket(**data)

    except (json.JSONDecodeError, ValueError) as e:
        print(f"Error processing packet: {e} | Line: {line}")

    return None



# -------------------------------- live plotting -------------------------------

plt.ion()
fig, ax1 = plt.subplots()
ax2 = None

times_ms = []
times_s = []
temperatures = []
pwms = []

current_output_name = None
log_file = None
temperature_line = None
pwm_line = None

def start_plot(file_name: str, target_temp: Optional[float] = None):

    # clear data
    times_ms.clear()
    times_s.clear()
    temperatures.clear()
    pwms.clear()

    # re set-up figure
    # bug: https://github.com/matplotlib/matplotlib/issues/28268 means we can't
    # just clear the subplots, since it messes up the titles.
    fig.clear()
    global ax1, ax2
    ax1 = fig.add_subplot(111)
    ax2 = ax1.twinx()
    ax2.set_ylim(-0.1, 1.1)

    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Temperature (°C)', color='b')
    ax2.set_ylabel('Heater PWM Fraction [0–1]', color='r')
    plt.tight_layout()

    global temperature_line, pwm_line, current_output_name, log_file

    temperature_line = ax1.plot([], [], 'b-', label='Temperature (°C)')[0]
    pwm_line = ax2.plot([], [], 'r-', label='Heater Power [0 - 1]')[0]

    if target_temp is not None:
        ax1.axhline(y=target_temp, color='black', linestyle='--', label='Target Temperature', linewidth=1)
        ax1.legend(loc='lower left')

    current_output_name = file_name
    log_file = open('data/' + current_output_name + ".csv", "w", encoding="utf-8")
    log_file.write("time_ms,temperature,heater\n")

def finish_plot():
    global current_output_name, log_file
    if current_output_name is not None:
        fig_filename = 'data/' + current_output_name + ".png"
        fig.savefig(fig_filename, dpi=300)
        print(f"Saved figure to {fig_filename}")
        current_output_name = None

    if log_file:
        log_file.close()
        log_file = None


def compute_and_write_metrics(target_temp: float, file_name: str, description: str):
    global times_ms, times_s, temperatures, pwms

    # -------------------------------- overshoot -------------------------------
    # how much did we go over the target?
    max_temp = max(temperatures)
    overshoot = max_temp - target_temp if max_temp >= target_temp else None

    # -------------------------------- rise time -------------------------------
    # the time from first crossing 70C until first entering ±1C of target
    rise_time = None
    rise_start = None
    for i in range(len(temperatures)):
        if temperatures[i] >= 70:
            rise_start = i
            break
    if rise_start is not None:
        for i in range(rise_start, len(temperatures)):
            if temperatures[i] >= target_temp - 1:
                rise_time = times_s[i] - times_s[rise_start]
                break
    if rise_time is None:
        print("Error: rise time not found")


    # ------------------------------ settling time -----------------------------
    # the time from first crossing 70C until we remain within ±1C of target
    # until the end of the data

    settling_time = None
    settling_idx = None
    band_low = target_temp - 1
    band_high = target_temp + 1

    if rise_start is None or not (band_low < temperatures[-1] < band_high):
        print("Error: temp never settled")
    else:
        for i, t in list(enumerate(temperatures))[-1:rise_start:-1]:
            if not (band_low < t < band_high):
                settling_time = times_s[i+1] - times_s[rise_start]
                settling_idx = i
                break

    # --------------------------- steady-state error ---------------------------
    # the average magnitude of of delta after we are settled
    # (or over the last 10% of the data if we never settle)
    if settling_idx is not None:
        settled_err = map(lambda t: abs(t - target_temp), temperatures[settling_idx:])
    else:
        settled_err = map(lambda t: abs(t - target_temp), temperatures[int(0.9 * len(temperatures)):])
    settled_err = list(settled_err)
    steady_state_error = sum(settled_err) / len(settled_err)

    # ------------------------------- power usage ------------------------------
    # the average power used from 70C to the end of the data
    power_usage = None
    if rise_start is not None:
        power_usage = sum(pwms[rise_start:]) / (times_s[-1] - times_s[rise_start])


    # --------------------------------- output ---------------------------------

    metrics_filename = 'data/' + file_name + ".txt"
    with open(metrics_filename, "w", encoding="utf-8") as f:
        f.write(f"{file_name}\n")
        f.write(f"{description}\n")
        f.write(f"Target Temperature: {target_temp:.2f}\n")
        if overshoot is not None:
            f.write(f"Overshoot: {overshoot:.2f} °C\n")
            f.write("    (difference between target, and maximum temperature)\n")
        else:
            f.write("Overshoot: never reached target temperature\n")

        if rise_time is not None:
            f.write(f"Rise Time: {rise_time:.2f} s\n")
            f.write("    (time from first crossing 70°C to first entering ±1°C of target)\n")
        else:
            f.write("Rise Time: never reached target temperature\n")

        if settling_time is not None:
            f.write(f"Settling Time: {settling_time:.2f} s\n")
        else:
            f.write("Settling Time: never settled within ±1°C of target\n")
            f.write("    (time from first crossing 70°C to remaining within ±1°C of target)\n")

        if power_usage is not None:
            f.write(f"Average Power Usage: {power_usage:.2f}\n")
            f.write("    (average power used from 70°C to end of data)\n")
        else:
            f.write("Average Power Usage: never reached target temperature\n")

        f.write(f"Steady-state Error: {steady_state_error:.2f} °C\n")


    print(f"Metrics written to {metrics_filename}")

    # the overall "quality" of the controller, for grid search
    # (lower is better)
    return (rise_time if rise_time is not None else 10000000) + 200 * steady_state_error

def log_point(time: int, temperature: float, heater_pwm: float):

    if temperature_line is None or pwm_line is None:
        return

    if log_file:
        log_file.write(f"{time},{temperature},{heater_pwm}\n")

    times_ms.append(time)
    times_s.append(time / 1000)
    temperatures.append(temperature)
    pwms.append(heater_pwm)

    temperature_line.set_xdata(times_s)
    temperature_line.set_ydata(temperatures)
    pwm_line.set_xdata(times_s)
    pwm_line.set_ydata(pwms)

    ax1.relim()
    ax1.autoscale_view()

    plt.pause(0.01)




# --------------------------------- experiments --------------------------------

# cycle through experiments, yield things to be sent to the device
def run():

    target_temp = 115.0
    tail_length = 90 # how long after target temp is reached to keep running,
                     # before moving to next experiment, in seconds

    def es_print(*args):
        print('\033[91mExperiment Scheduler:', *args, '\033[0m')

    es_print("Starting experiment scheduler")

    tests: List[Tuple[str, str, str]] = []
    # command sent to device, filename for output, description of test

    tests.append((
        f"ON-OFF {target_temp}",
        f"ON-OFF_basic",
        "ON-OFF controller with no hysteresis"
    ))

    for h in [0.2, 0.5, 1, 2]:
        tests.append((
            f"ON-OFF-HYST {target_temp} {h}",
            f"ON-OFF-HYST_window_width_{str(h).replace('.', 'point')}",
            f"ON-OFF controller with a hysteresis window of {h}°C"
        ))

    for band_width in [20, 10, 5, 2, 1, 0.5, 0.2]:
        tests.append((
            f"PROPORTIONAL {target_temp} {band_width}",
            f"PROPORTIONAL_band_width_{str(band_width).replace('.', 'point')}",
            f"Proportional controller with a band width of {band_width}°C"
        ))

    # usage: PI <target> <Kp> <Ki> <i_inBandOnly=0 or 1>
    pi_configs = [
        (0.1, 0.01),
        (0.5, 0.05),
    ]
    for (kp, ki) in pi_configs:
        for i_inBandOnly in [1, 0]:
            tests.append((
                f"PI {target_temp} {kp} {ki} {i_inBandOnly}",
                f"PI_with_Kp-{str(kp).replace('.', 'point')}_Ki-{str(ki).replace('.', 'point')}_{'I-inBandOnly' if i_inBandOnly else 'I-all'}",
                f"PI controller with Kp={kp}, Ki={ki}, integral term {'only when 0 < P < 1' if i_inBandOnly else 'always'}"
            ))


    def tests_gen():
        for command, filename, description in tests:
            yield command, filename, description

        # -------------------------- parameter search --------------------------
        # Starting bounds are in [0,1] for each axis.
        bounds = ((0, 1), (0, 1), (0, 1))
        # eval function at each corner 

        def point_to_output(bounds):
            p, i, d = bounds

            if p != 0:
                p = 0.625 * 2 ** (1 - 1 / p)
            if p < 1 / (115 - 75):
                p = 1 / (115 - 75)

            if i != 0:
                i /= 4
                i = 2 ** (1 - 1 / i)

            # d = 2 * d - 1
            # if d < 0:
            #     d = -0.1 * 2 ** (1 - 1 / abs(d))
            # elif d > 0:
            #     d = 2 ** (1 - 1 / abs(d))
            d = d / 2

            p = float(p)
            i = float(i)
            d = float(d)

            # usage: PID <target> <Kp> <Ki> <Kd>
            p = f"{p:0.6f}"
            i = f"{i:0.6f}"
            d = f"{d:0.6f}"
            command = f"PID {target_temp:0.6f} {p} {i} {d}"
            p_s = p.replace('.', 'point')
            i_s = i.replace('.', 'point')
            d_s = d.replace('.', 'point')
            filename = f"PID_with_Kp-{p_s}_Ki-{i_s}_Kd-{d_s}"
            description = f"PID controller with Kp={p}, Ki={i}, Kd={d}, integral term only when -1 < P < 1"
            return command, filename, description
            

        corners = {}
        for i in (0, 1):
            for j in (0, 1):
                for k in (0, 1):
                    pt = (bounds[0][i], bounds[1][j], bounds[2][k])
                    corners[(i, j, k)] = yield point_to_output(pt)

        root = Node(bounds, corners)
        root.center_val = yield point_to_output(root.center)
        
        # queue ordered by the node's center evaluation.
        nodes_heap = [root]

        # Expand nodes (split them) until we get bored and kill the program
        while True:
            nodes_heap = list(sorted(nodes_heap))

            # debug
            print("-" * 40, "State of queue:")
            for n in nodes_heap:
                print(n.center, point_to_output(n.center)[0], n.center_val, n.value())
            print("-" * 40, "end state of queue")

            node = nodes_heap[0]
            nodes_heap = nodes_heap[1:]
            left, right = node.split()
            left.center_val = yield point_to_output(left.center)
            right.center_val = yield point_to_output(right.center)
            nodes_heap.append(left)
            nodes_heap.append(right)


    tests_g = tests_gen()
    command, filename, description = tests_g.send(None)
    values_cache = toml.load('data/values_cache.toml')
    while True:

        es_print('-----------------------')
        best_so_far = None
        for i in values_cache.items():
            if (type(i[1]) == type(0.1)):
                if (best_so_far is None) or i[1] < best_so_far[1]:
                    best_so_far = i
        es_print(f"Best command & score so far: {best_so_far}")
        values_cache['best_so_far'] = best_so_far[0]
        es_print('-----------------------')



        if os.path.exists('data/' + filename + ".txt") or command in values_cache:
            es_print(f"Skipping {filename}, already completed")
            command, filename, description = tests_g.send(values_cache.get(command, None))
            continue

        while True:
            packet = yield
            if isinstance(packet, SystemReadyPacket):
                es_print("Received SYSTEM READY packet")
                break
            log_point(packet.time, packet.temperature, packet.heater)

        es_print(f"Sending command: {command}")
        yield command


        exit_time = None
        while True:

            packet = yield

            if not isinstance(packet, DataPacket): continue

            if len(times_ms) == 0 or packet.time < times_ms[-1]:
                es_print("Starting new plot")
                start_plot(filename, target_temp)

            log_point(packet.time, packet.temperature, packet.heater)

            if exit_time is None:
                if packet.temperature > target_temp - 1:
                    es_print("Target temperature reached, starting stable period")
                    exit_time = packet.time + (tail_length * 1000)
            else:
                if packet.time > exit_time:
                    es_print("Stable period complete, starting cool-down")
                    break

            if packet.time > 300 * 1000 and exit_time is None:
                es_print("Target temperature not reached in 5 minutes, stopping test")
                break


        finish_plot()
        value = compute_and_write_metrics(target_temp, filename, description)
        values_cache[command] = value
        with open('data/values_cache.toml', 'w') as f:
            toml.dump(values_cache, f)
        command, filename, description = tests_g.send(value)

        yield "COOL-DOWN"

    yield "done"



# ---------------------------------- main loop ---------------------------------


BAUD_RATE = 115200

ports = lp.comports()
arduino_port = "COM3"
for p in ports:
    print(p.device, p.description)
    if 'arduino' in p.description.lower():
        arduino_port = p.device
        print('\t⬆️  identified as arduino')

if arduino_port is None:
    print('No arduino found, exiting')
    exit()

experiment = run()
experiment.send(None) # start the generator

s = None
next_command = None
try:
    s = serial.Serial(arduino_port, BAUD_RATE, timeout=1)
    print(f"Opened {arduino_port} at {BAUD_RATE} baud. Reading...")

    while True:
        try:
            line = s.readline().decode("utf-8").strip()
        except UnicodeDecodeError:
            print("Error decoding line")
            continue

        if line.startswith('LOG:'):
            print('\033[93mLOG from device:' + line[4:] + '\033[0m')
            continue

        print(f'\033[90mAttempting to decode: {line}\033[0m')
        packet = decode(line)
        if packet is None:
            print("Decode failed.")
            continue

        print(packet)

        if isinstance(packet, CommandReceivedPacket):
            print(f"Command received: {packet.command}")
            if next_command is not None and next_command.strip().split(' ')[0] == packet.command:
                print("Command acknowledged.")
                next_command = None
            continue

        nc = experiment.send(packet)
        if nc is not None: next_command = nc

        if next_command == "done":
            print("All experiments complete.")
            break

        if next_command is not None:
            print(f"Sending command to device: {next_command}")
            s.write(bytes(next_command + '\n', 'utf-8'))

except KeyboardInterrupt:
    print("KeyboardInterrupt: stopping script.")
except Exception as e:
    print("Exception:", e)
finally:

    finish_plot()

    if s and s.is_open:
        s.close()
        print("Serial connection closed.")

    plt.ioff()
