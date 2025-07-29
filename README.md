<img src="https://raw.githubusercontent.com/qqqlab/ESPnow-RC/refs/heads/main/extras/img/1.jpg" width="100%" />

# ESPnow-RC

<p align="center">&star;&nbsp;&star;&nbsp;&star;&nbsp;</p>
<p align="center">If you like <i>ESPnow-RC</i>, please give it a &star; star</p>
<p align="center">or fork it and contribute!</p>
<p align="center">&star;&nbsp;&star;&nbsp;&star;&nbsp;</p>

This project uses ESP-NOW to create a RC radio link. It is thought to be directly integrated:
- Place the ESP transmitter in an existing (toy) radio by connecting the exiting potmeters and buttons to the ESP
- Use the ESP receiver as [flight controller](https://github.com/qqqlab/madflight), or as interface to a flight controller

If you want to add TX/RX module to your exiting RC transmitter then https://github.com/RomanLut/hx_espnow_rc might be a better starting point.

## Automatic Binding

The transmitter sends out broadcast packages and waits for a receiver to respond. Once a response is received, the transmitter and receiver are bound, and will communicate via the MAC address of the peer.

## Building a Transmitter

Open your (toy) RC transmitter or gamepad. Locate the main chips, and remove them. Solder wires to the existing potmeters and switches, and connect them to the ESP board. If the RC transmitter was powered by 3 or 4 (i.e. 4.5V - 6V) penlite batteries, you probably can connect this to the 5V input of the ESP board. If in doubt, use a voltage regulator.

<img src="https://raw.githubusercontent.com/qqqlab/ESPnow-RC/refs/heads/main/extras/img/2.jpg" width="100%" />
