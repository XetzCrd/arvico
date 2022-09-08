# MSD-E20

* [Info](http://cc-smart.net/en/san-pham/msde20.html)
* [Info x2](https://makermotor.com/brushed-10-40v-20a-dc-motor-servo-control-driver-closed-loop-feedback/)

# Configuration Steps

* [PuTTY Conf](https://youtu.be/Y0_Qw1RHois?t=29)
* N0 ? (x = MSD address)
* Nx ? (All parameters)

## Calibrate
* Nx $001 = val (val is the new x address)
* Nx $002=val (val = Encoder counts per round)
* Nx O T (Get PID parameters)
* Nx O S (Save changes)

## Speed Control Mode

* Nx O M3 S

Example: Nx [p value] [v value] [a value] 
* N1 v10 [O G3] ([optional] gives current information)
* N1 v10 a5

## Position Control Mode

* Nx O M4 S

Example: Nx [p value] [v value] [a value] 
* N1 p500 [O G3] ([optional] gives current information)
* N1 p1000 v80 

## None Control Mode

* Nx O M5 S
