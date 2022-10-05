# Unity project using NetMQ and pyzmq backend

## Unity project
Change `Host` field of `MixedRealitySceneContent/Menu`'s `Client` component and `MixedRealitySceneContent/Tool`'s `IR Tool Controller` component to your laptop's IP address.

**Unity version: 2019.4**

## Python
```
pip install pyzmq open3d
```
- Update the IP address to HoloLens IP in the following line of `PythonServer/main.py`'s `main()` function:
  ```python
  parser.add_argument('--ip', default='[HoloLens IP]',
                        help='IP address of the HoloLens')
  ```
- Update the tools in `PythonServer/IRTrack/ToolList` folder accordingly. 

## Usage
- On laptop, activate the conda environment.
  ```
  cd PythonServer
  python main.py --ip [HoloLens IP]
  ```
- On HoloLens, open tha app and raise you left palm. Click the `IR Tracking` toggle on the hand menu to start tracking.
- Click `Toggle Preview` button on hand menu to toggle the preview of AHAT camera frame on HoloLens.
