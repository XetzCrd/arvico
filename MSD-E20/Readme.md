# MSD-E20

[Info](https://makermotor.com/brushed-10-40v-20a-dc-motor-servo-control-driver-closed-loop-feedback/)
[Info x2](http://cc-smart.net/en/san-pham/msde20.html)

# Configuration Steps

* [PuTTY Conf](https://youtu.be/Y0_Qw1RHois?t=29)
* N0 ? (x = MSD address)
* Nx ? (All parameters Conf.)

## Tunning

* Nx $002=val (val = Encoder resolution per round)
* Nx $007=val (val = Error)
* Nx $011=val (val = Test velocity)
* Nx $012=val (val = acceleration)
* Nx O [T] (Get PID parameters)
* Nx O [S] (Save changes)

## Communication

* Nx $005=val (val = 0: PULSE/DIR, 1: UART Network, 2: None, 3: Analog (Just for velocity Mode))
* Nx $009=val (val = UART baud rate)
* Nx O [S] (Save changes)

## Current Limit

* Nx $006 = val (val in mA)
* Nx O [S] (Save changes)

## Change Address

* Nx $001 = val (val is the new x address)
* Nx O [S] (Save changes)
