# **Master Thesis Code Base**

## Table of Contents

-   [Code](#code)
-   [Resources](#resources)

## Application setup

### Flashing steps

The steps that have been followed to test the code:

-   Create a new project via the nRF Connect SDK extension in VSCode.
-   Copy the provided C and prj.conf files into the new project.
-   Select the 2.5.0 version for the SDK and the toolchain.
    -   First copy the "nrf_modem_dect_phy.h" file into the correct SDK folder.
    -   Replace the "libmodem.a" that can be found in the "dect_modem" zip file in the lib folder of the correct SDK.
-   Build the project and flash it to the device. After this when a terminal is connected to the device it can be clearly seen that the device is correctly flashed.

## Code

### broadcast.c

Simple code that broadcasts a counter and turns on the LEDs of the device while transmitting.
Device choses to be RX or TX in the first 10sec based on if there's another TX in the network.

### hello_world.c

Simple piece of software provided by NOrdic with the release of the version $1.0$ of the DECT NR+ technology. This code is used to test a bidirectional communication between two RDs.

#### Functionalities

-   Bidirectional communication between two RDs in the same channel.
-   There's no type of reporting
-   The code is very simple and can be used as a base for further development.
-   Shows how to transmit simple text and how to format it correctly to be sent.
-   Improvement compared to broadcast.c in relation of the transmit and receive functions.

### light_control_broadcast.c

Send a broadcast and every device turns on that specific light that corresponds to the button that has been pressed.
Only one script and the devices can listen and transmit, therefor the lights of all the devices from the same network light up at the same time and can be controlled from any device of the network.

### light_control_unicast.c

One device is the sink and controls the LEDs of the nodes of the network. To achieve this the nodes send a empty broadcast message and the sink saves that information so later it can connect each button with each LEDs.

This is the first script that uses the filtering for the receiver id so only the device that needs to take information takes and if it's not for them it ignores it.

## Resources

-   [Hello world sample code](https://github.com/nrfconnect/sdk-nrf/tree/main/samples/dect/dect_phy/hello_dect)
