# crazyflie-hover
Hovering feature for The Crazyflie 2.1 based on a PID controller. 
Video demonstration available [here](https://youtu.be/_ShpK5zyPHQ).
Bachelor's thesis that discusses the implementation available [here](https://trepo.tuni.fi/handle/10024/162266).

## Requirements
- Crazyflie 2.x (Crazyflie 1.0 aka Crazyflie Nano not supported)
- Crazyradio PA or Crazyradio 2.0
- The [cflib](https://github.com/bitcraze/crazyflie-lib-python) python library

## Usage
After all of the requirements have been met, run pid_hover.py with arguments target_altitude(meters) hover_time(seconds)

For example: `python pid_hover.py 0.4 15` will make the quadcopter hover for 15 seconds at an altitude of 0.4 meters.
