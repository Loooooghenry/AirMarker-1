# AirMarker

# Introduction

AirMarker is a wireless system for transmitting stimulus markers over infrared. This system was initially intended to be used with Emotiv EPOC headsets since they have no connection for external markers. A detailed description of the system is available from a [PeerJ Preprint in 2013](https://peerj.com/preprints/32/).

AirMarker and EPOC have been proven to successfully record audio event related potentials in [adults](https://peerj.com/articles/38/) and [children](https://peerj.com/articles/907/) and [visual event related potentials](https://www.ncbi.nlm.nih.gov/pubmed/26057115). They have also been used for [laboratory teaching in Neuroscience](http://www.funjournal.org/wp-content/uploads/2017/01/june-15-104.pdf). 

# Hardware 

There are three hardware parts, namely, 

1. Transmitter

   1. [Arduino Uno](http://au.element14.com/arduino-org/a000073/arduino-uno-smd-dev-kit/dp/2285200) 
   2. [enclosure](http://au.element14.com/arduino-org/a000009/arduino-enclosure-abs-pc-black/dp/1848692)
   3. Shield board; the board design and BOM are available in hardware/Airmarker-transmitter-shield.zip
   4. [Audio lead 3.5mm stereo to 3.5mm stereo](http://au.element14.com/pro-signal/av13646/lead-3-5mm-s-jack-jack-1-2m/dp/3712278)
   5. Visual stimulus detector
   6. USB A to B lead
   
2. Receiver

   1. [Arduino pro mini](https://littlebirdelectronics.com.au/products/arduino-pro-mini-328-3-3v-8mhz-1)
   2. Enclosure; the design file is available in hardware/Airmarker-receiver-enclosure.zip
   3. Shield board; the board design and BOM are available in hardware/Airmarker-receiver-shield.zip
   4. CR2032 3V battery
   5. Velcro for attaching the receiver to EPOC

3. Biasing wires

   1. Solder 3x 4.7 kOhm resistors together to form a Y-network
   2. Solder a short wire (5-10 cm) to one of the resistor for connection to DRL.
   3. Solder two longer wires (20 cm) to the other two resistors for connection to any two electrodes.
   

# Arduino code

