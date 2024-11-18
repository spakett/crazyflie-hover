import sys
import logging
import time
from threading import Thread

import matplotlib.pyplot as plt
from datetime import datetime
from numpy import mean

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

uri = 'radio://0/80/2M/E7E7E7E7E7'

current_thrust = 0
target_altitude = 0
current_altitude = 0
average_altitude = 0
altitude_history = [0, 0, 0, 0, 0]
step_time = 0.01
operation_time = 0

logging.basicConfig(level=logging.ERROR)

class HoverPid:
    """Connect to a Crazyflie and hover at the specified altitude for the specified duration."""

    def __init__(self, link_uri):
        """ Initialize and  with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connection-related callbacks
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)

        print(f'Connecting to {link_uri}')

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        print(f'Connection established with {link_uri}')

        # Start a separate thread to do logging
        print('Starting data logging (1 s interval)')
        Thread(target = self._logging).start()

        # Start a separate thread to do the hovering
        print(f'Starting motor control test ({operation_time} s duration')
        Thread(target=self._hover).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print(f'Connection to {link_uri} failed: {msg}')

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print(f'Connection to {link_uri} lost: {msg}')

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print(f'Disconnected from {link_uri}')

    def _data_log_data(self, timestamp, data, logconf):
        """Callback from the log API when data arrives"""
        global current_altitude
        current_altitude = data['stateEstimate.z']
        global altitude_history
        altitude_history.pop(0)
        altitude_history.append(current_altitude)

        global average_altitude
        average_altitude = mean(altitude_history)

        global current_thrust
        current_thrust = data['stabilizer.thrust']

    def _data_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print(f'Error when logging {logconf.name}: {msg}')

    def _logging(self):
        # Define the variables that will be logged: the z-coordinate estimate and thrust
        self._data_log = LogConfig(name = 'Sensor data', period_in_ms = int(step_time * 1000))
        self._data_log.add_variable('stateEstimate.z', 'float')
        self._data_log.add_variable('stabilizer.thrust', 'float')

        try:
            self._cf.log.add_config(self._data_log)

            # Callback for receiving data
            self._data_log.data_received_cb.add_callback(self._data_log_data)

            # Callback for errors
            self._data_log.error_cb.add_callback(self._data_log_error)

            # Start logging
            self._data_log.start()

        except KeyError as e:
            print(f'Could not start log configuration, {str(e)} not found in TOC.')
        except AttributeError:
            print('Could not add log config, bad configuration.')

    def _hover(self):

        # Thrust values go between 10001 and 60000 where 10001 is the minimum and 60000 is the maximum
        # The drone has a startup thrust protection that disables motor control until setpoint(0, 0, 0, 0) has been given
        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)

        # Plotting values
        current_time = datetime.now()
        flight_data = []

        # PID values
        ku = 16000
        tu = 3.116

        ti = 0.5 * tu
        td = 0.125 * tu

        kp = 0.6 * ku
        ki = ku / ti
        kd = ku * td

        ki = 0.7 * ki
        kd = 0.7 * kd

        print(f'{kp} {ki} {kd}')

        integral = 0
        prev_error = 0
        base_thrust = 38250 # 38250 This power is required to take off
        time_steps = range(int(operation_time / step_time))
        
        # Take off and hovering
        for i in time_steps:
            error = target_altitude - average_altitude
            proportional = kp * error
            integral = integral + ki * error * step_time
            derivative = kd * (error - prev_error) / step_time
            output = base_thrust + proportional + integral + derivative
            output = max(min(output, 60000), 0) # Clamp the value between 0 and 60000
            prev_error = error

            print(f'PID: {output} || Power: {current_thrust} || Alt: {current_altitude:2f} || Error: {error:2f}')    
            self._cf.commander.send_setpoint(0, 0, 0, int(output))
            flight_data.append([output, current_thrust, current_altitude, error])
            time.sleep(step_time)

        # Landing
        while output > 20000:
            output -= 75
            self._cf.commander.send_setpoint(0, 0, 0, int(output))
            time.sleep(step_time)

        # Cut the power
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(step_time)

        # Flight graph
        print('Saving flight data...')
        altitude_values = [x[2] for x in flight_data]
        # error_values = [x[3] for x in flight_data]
        time_steps = [x / 100 for x in time_steps]
        plt.plot(time_steps, altitude_values, label = "Altitude")
        # plt.plot(time_steps, error_values, label = "Error")
        plt.axhline(y = target_altitude, label = "Target Altitude", color = 'red' )
        plt.title(f'Time-altitude plot')
        plt.xlabel('Time (s)')
        plt.ylabel('Altitude (m)')
        plt.legend()
        plt.savefig(f'results/flight_{current_time}.png')

        print('All done! Disconnecting...')
        self._cf.close_link()


def main():
    # Call program with arguments: target_alt hover_time
    # python pid_hover.py 0.5 10
    args = sys.argv[1:]
    
    if len(args) != 2:
        print("Bad arguments! Exit.")
        return
    
    global target_altitude
    target_altitude = float(args[0])
    global operation_time
    operation_time = int(args[1])

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # Start the main loop
    hp = HoverPid(uri)

if __name__ == '__main__':
    main()