# HoloLens2 Tool Tracking Unity part

# Version:
- Unity 2019.4
- Visual Studio 2019
- MRTK 2.7.3

# Building
Build the unity project, open the visual studio solution. Swicth to Release, ARM64 platform. In the Solution Explorer on the right side, find the 'Package.appxmanifest' file, right click, select "Open with", use XML (Text) Editor to open the file. Change the file according to the following:

- Add restricted capabilities package:
```xml 
<Package 
  xmlns:mp="http://schemas.microsoft.com/appx/2014/phone/manifest" 
  xmlns:uap="http://schemas.microsoft.com/appx/manifest/uap/windows10" 
  xmlns:uap2="http://schemas.microsoft.com/appx/manifest/uap/windows10/2" 
  xmlns:uap3="http://schemas.microsoft.com/appx/manifest/uap/windows10/3" 
  xmlns:uap4="http://schemas.microsoft.com/appx/manifest/uap/windows10/4" 
  xmlns:iot="http://schemas.microsoft.com/appx/manifest/iot/windows10" 
  xmlns:mobile="http://schemas.microsoft.com/appx/manifest/mobile/windows10" 
  xmlns:rescap="http://schemas.microsoft.com/appx/manifest/foundation/windows10/restrictedcapabilities" 
  IgnorableNamespaces="uap uap2 uap3 uap4 mp mobile iot rescap" 
  xmlns="http://schemas.microsoft.com/appx/manifest/foundation/windows10"> 
```
- Modified capabilities with with new package:
```xml
  <Capabilities>
    <rescap:Capability Name="perceptionSensorsExperimental" />
    <Capability Name="internetClient" />
    <Capability Name="internetClientServer" />
    <Capability Name="privateNetworkClientServer" />
    <uap2:Capability Name="spatialPerception" />
    <DeviceCapability Name="webcam" />
  </Capabilities>
```

Deploy to the HoloLens2

# Setting up IP address
Connect the HoloLens2 with a USB-C to ethernet adpater, then connect it with a Ubuntu PC with a ethernet cable.

Set the IPv4 address of HoloLens manually to 192.168.1.66.
(TODO: need a figure)

Set the IPv4 address of the Ubuntu PC to 192.168.1.29
(TODO: need a figure)


# Usage

1. Click connect holoLens in the Python UI
2. Raise your left hand, click start sensor button with your right hand.
3. In Python UI, you should be able to see messages when the connection is established successfully. Check the boxes of image stream on the UI (LF, RF etc.)
