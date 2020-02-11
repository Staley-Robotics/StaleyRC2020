# Vision 2020

This year's vision code used chameleon-vision. We chose it because it was easy.

## Setup

To install:
Head to
https://chameleon-vision.readthedocs.io/en/latest/installation/coprocessor-setup.html

1. Get raspberry pi, ethernet cords, microSD and microSD writer, HDMI monitor and cords, keyboard.
- Make sure camera is plugged into the same port every time otherwise this will not work.
- Current camera USB goes in top right port.

2. Install **buster lite** onto the microSD card (https://www.raspberrypi.org/downloads/raspbian/) using **balena etcher** (https://www.balena.io/etcher/).

3. Place microSD card back in pi. Attach ethernet cord between pi and the internet (most likely the wall) and attach the monitor and keyboard to the pi.

4. Run the following on the raspberry pi.
```
wget https://git.io/JeDUk -O install.sh

chmod +x install.sh

sudo ./install.sh

sudo reboot now
```

5. Exchange the ethernet cord between the pi and the internet with an ethernet cord between the pi and the robot's radio. Then make sure that the robot's radio is attached via another ethernet cable to the robot's roborio.

6. On the pi, type ```sudo nano /etc/dhcpcd.conf``` to get to where the pi's network settings are. Then, make it look like this picture to set the pi's IP to **static**. ![](img/IMG_20200208_114922.jpg?raw=true)

- If that doesn't work, try following this guide. You'll be using interface **eth0**. https://thepihut.com/blogs/raspberry-pi-tutorials/how-to-give-your-raspberry-pi-a-static-ip-address-update

## Running Vision

1. Click to get back: Ctrl X --> Y --> Enter.

2. Now you can run **chameleon vision** by typing ```sudo java -jar chameleon-vision.jar``` on the raspberry pi.

2. This will start a camera server on the static ip you set earlier on port **5800**. Access it on a laptop connected via wifi to the robot using a browser on the laptop. For example, this year we set our static ip to **10.49.59.69**. To access **chameleon vision** via a browser, we typed **10.49.59.69:5800** into the browser's search bar.

3. Look at **chameleon vision documentation** (https://chameleon-vision.readthedocs.io/en/latest/getting-started/building-2d-pipeline.html) to see how to make a **pipeline**.

4. Once you're satisfied, you can access data found by the process in java using **network tables**. See our **Vision** subsystem for more.

## Interaction with Robot Code

1. Import **network tables** in the **Vision** subsystem. 

2. Pull data from **network tables** to use in methods. Example from our Vision code:
```
NetworkTableInstance inst = NetworkTableInstance.getDefault();
table = inst.getTable("chameleon-vision/Microsoft LifeCam HD-3000");
```
 
3. Use data from **network tables** to:
- Determine if camera stream should be processed. Example of method from our Vision code:
```
public void setTapeProcessing(final boolean trackTape) {
    NetworkTableEntry targetProcessing = table.getEntry("driverMode");
    targetProcessing.setBoolean(trackTape);
  }
```
- Check if target is detected or not. Example of method from our Vision code:
```
public boolean tapeDetected() {
    NetworkTableEntry targetDetected = table.getEntry("isValid");
    return targetDetected.getBoolean(false);

  }
```
- Make calculations pitch, distance, and yaw). Example of methods from our Vision code:
```
public double getPitch() {
    targetPitch = table.getEntry("targetPitch");
    return targetPitch.getDouble(0.0);
  }
```
```
public double calculateDistance() {
    distance = (bottomOfTargetHeight - fixedCameraHeight) / (Math.tan(Math.toRadians(pitch + fixedCameraAngle)));
    return distance;
  }
```
```
public double getYaw() {
    NetworkTableEntry targetYaw = table.getEntry("targetYaw");
    return targetYaw.getDouble(0.0);
  }
```

For more information visit our Vision subsystem.


