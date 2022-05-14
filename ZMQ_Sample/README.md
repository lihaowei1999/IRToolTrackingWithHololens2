# Unity project using NetMQ and pyzmq backend

## Unity project
Change `Host` field of `MixedRealitySceneContent/Menu`'s `Client` component and `MixedRealitySceneContent/Tool`'s `IR Tool Controller` component to your laptop's IP address.

**Unity version: 2019.4**

## Python
```
pip install pyzmq
```
- Update the IP address to HoloLens IP in the following line of `PythonServer/main.py`'s `main()` function:
  ```python
  receive_depth("[HoloLens IP address]", args.path)
  ```
- Update the tools in `PythonServer/IRTrack/ToolList` folder accordingly. 

## Usage
- On laptop, activate the conda environment.
  ```
  cd PythonServer
  python main.py --path tool
  ```
- On HoloLens, open tha app and raise you left palm. Click the `IR Tracking` toggle on the hand menu to start tracking.
- Click `Toggle Preview` button on hand menu to toggle the preview of AHAT camera frame on HoloLens.
