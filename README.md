# Wi-SUN MBED SIM (TCP, MQTT and FHSS ready)

* You need mbedtls for this repo.
* See license before fork/copy.

## Compile mbedtls

0) Install dependencies:
```bash
sudo apt-get install libnl-3-dev libnl-route-3-dev libcap-dev \
    libsystemd-dev libdbus-1-dev cargo cmake ninja-build pkg-config lrzsz
   ```
1) Clone mbetls (ver.3) in the pwd `git clone --branch=v3.0.0 https://github.com/ARMmbed/mbedtls`
2) Do cd into mbedtls dir and `cmake -G Ninja .`
3) Then do: `ninja`

## Compile mbed OS

1) Compilation of the simulation tools is now enabled to build the relevant binaries. Return from mbedtls dir to this repo's dir and:

```bash
cmake -DCMAKE_PREFIX_PATH="$(pwd)/mbedtls/" -G Ninja .
```

and then: `ninja`

* FHSS mode is ON and channel hopping is working. CSMA-CA MAC mode disabled.

## Run

### Script for running the simulations up to 255 nodes

Run `bash tunup.sh` to setup a TUN interface (This needs sudo). Then run `bash run.sh`. This will open the simulation server and MAC layer processes in new tabs, and the mbed nanostack in separate windows.

### Example script of 3 nodes (old method)

In the main dir of this repo, you see a file called `run.sh`. This file contains enough lines of bash code to run 3 nodes: One BR and 2 Router nodes.\
To do this, overall we need 7 process:

1) Simulation server: `wssimserver` (1 instances)
2) Wi-SUN MAC simulator and PHY model: `wshwsim` (3 instances)
3) Wi-SUN Router node: `wsnode` (2 instances)
4) Wi-SUN BR node: `wsbrd` (1 instances)

* Note that BR process may need root access to create a tun interface, so `sudo` is used for the case of BR process and it will prompt for your password.
* There must be different config files for Router nodes and BR. An example of this file `examples` dir. All Router nodes can use the same config file.

## Run in docker

It is possible to run multiple simulations in parallel. Meaning that a mesh network per docker instance. This is realized by using Docker which allows the networks to be run isolated from eachother. At the end of the simulation, the results and logs will be saved in a directory in the host, then docker container will exit and removed.

First, build a docker image. For this see the `docker` directory.

To run, modify the available parameters in `run_docker.bash`, and run it.


### Topology setup

The topology of the network is defined within the `run.sh` script.

Here is a short explanation on the script content. The line

```bash
`gnome-terminal --tab -- $DIR/wssimserver -g 0-1 -g 1-2 /tmp/sim_socket --dump`
```

runs `wssimserver` which is a server to make connection between nodes RF interfaces. The `-g` option can be used to group the nodes and make a custom topology.\
For example `-g 0-1 -g 1-2` will make a line topology:

```text
0 --- 1 --- 2
```

But the result of `-g 0-2 -g 1-3` will be:

```text
     0
   /   \
  1 --- 2
   \   /
     3
```

The `-g` option also supports providing an RSSI value for a specified edge. The format `-g 2,5:-67.0` will create an edge from node 2 to 5 with an RSSI of -67.0 [dBm]. By default, if no value is provided, the RSSI value is `1 [dBm]` to guarantee a very good link quality.

The RSSI is used within the simulator to calculate a frame transmission error rate at PHY layer. The formula used to compute the frame error rate is based on the non-coherente FSK-2 BER formula and is the following:

$FER(P_r) = \frac{1}{2} e^{\frac{1}{2} (10 \cdot \log_{10}(kTB) + 30 + NF_{LNA} + 10\cdot \log_{10}​(\frac{B}{R_b} ) - P_{r})} \cdot 8 \cdot L_{frame}$

where $P_{r} \approx RSSI$ [dBm], $k$ = Bolzman constant ($k = 1.3806 \cdot 10^{−23}$ [J/K]), $T$ = temperature [K], $B$ = channel bandwith [Hz], $NF_{LNA}$ = noise figure of the LNA, $R_b$ = transmission bitrate [bit/s], $L_{frame}$ = frame byte count

In this simulator, we use $T$ = 293 [K], a channel $B$ = 100 [kHz], a $R_b$ 100 [kbit/s] and $NF_{LNA}$ = 4 [dB] (the formula can be edited in `sl_rf_driver.c`).

The following table presents the probability of packet loss of a 100 bytes frame depending on the RSSI value:

| RSSI [dBm] | Frame error rate [-] |
|------|------------------|
| 0.0  | 3.63e-24 |
| -10.0  | 5.38e-22 |
| -20.0  | 7.99e-20 |
| -30.0  | 1.19e-17 |
| -40.0  | 1.76e-15 |
| -50.0  | 2.61e-13 |
| -60.0  | 3.88e-11 |
| -70.0  | 5.75e-09 |
| -80.0  | 8.54e-07 |
| -90.0  | 1.27e-04 |
| -100.0  | 1.88e-02 |

> **_NOTE:_** be aware that, by default, Wi-SUN does not accept a neighbour as a parent if the RSSI is below -80 dBm (edit `DEVICE_MIN_SENS` variable to change the limit).

### Wireshark

You can add `-w` option to every node's `wshwsim` process and it will bring up a Wireshark window for you. But Note that you need to run the corresponding node processes manually in another terminal window. Otherwise it will not connect to its MAC/PHY.

### GTK and GAK keys

Wi-SUN uses GAK instead of GTK, so even if you set a known GTK in the config file, you can't use it directly in the Wireshark to decrypt the packets. Thus, a python code is added to example dir in this repo to generate GAK from GTK.

### Features, current status and known bugs

RF driver implemented using fd and sockets.

FHSS timing is working well, channel hopping is OFF at the start, but gets ON after configurable `x` minutes. Because sometimes ASYNC packets are not sent over all available channels which makes it difficult to join. This `x` should be proportional to the location of the node in the network: distant nodes need to spend more time on FHSS OFF mode.


### References

1. Simulator by [SiLabs](https://github.com/SiliconLabs/wisun-br-linux) (version 1.2)
2. TCP re-add by [Eric](https://github.com/ercclpn)
3. FHSS MAC and all related functions and modifications added and applied by Mahboob Karimian
4. MQTT adaptation to work with timers by Mahboob Karimian ([source](https://github.com/eclipse/paho.mqtt.embedded-c))
5. RF driver added by Mahboob Karimian inspired by EFR32 driver implementation (private) by [Yann Charbon](https://github.com/YannCharbon), and open source S2LP RF driver by STM
