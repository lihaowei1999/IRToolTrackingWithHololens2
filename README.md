# IRToolTrackingWithHololens2

This is a multi-threaded holoLens 2 sensor data transfer system, implemented in Python via TCP socket connection, enabled the acquisition of the sensors' data at a high frame rate and with low latency. 

## Installing
We suggest using a virtual python environmnet. Here we provide an instruction based on conda.
```
git clone https://github.com/lihaowei1999/IRToolTrackingWithHololens2.git
cd IRToolTrackingWithHololens2
conda create --name hl2sensor python=3.8
conda activate hl2sensor
pip install -r requirements.txt
```

## Running
```
cd UI_python
python main.py
```
- The code is tested on Ubuntu 20.04 and window 10. There remains to be some bugs for windows 10, but should run well for ubuntu

## UI
### Connection to Hololens2 Sensors
- Keep Hololens and PC in one subnet, set the ip of PC as `192.168.1.29`
- Click `Connect to Hololens View` on UI
- Click `Sensor` on Hololens hand menu
- Now the Connection is established
- Click the check box in `Open Sensors` area, the image for the each sensor would be shown in UI
- If you want to change ip in your own program, please substitue the plugin and script in Unity program with those in folder `./ChangeIP`. I'm sure that this work, but I haven't tested for several months so I do want to merge that into main branch. You can control the ip and ports in the script. Every sensor uses a different port.

### Data Saving
- The data saving is only prepared for AHAT camera depth and reflectivity image
- Keep the AHAT sensor on
- Change the file name and frame names in line `offline data`
- Click `Collect`
- The file would be saved under folder `UI_python\Cache\`

### Running with offline data
- Keep the recorded data in folder `UI_python\Cache\`
- Fill the name of the data file in the place of `OfflineVideoName` in line `offline data`
- Click `Provide`

### Tool definition
- The tool definition is generated with a recorded AHAT file of the tool (see `Data Saving`)
- Collect data when put the tool at the center of the image, around 400mm away, for about 300 frames
- Put the filename in the place of `OfflineVideoName` in line `Tool Def`, put the tool name you want in the place of `ToolName`
- Click `ConstructTool`
- The tool definition file would be saved in folder `UI_python\ToolList\xxx.mat`

### Tool tracking
- Modify the file `\ToolList\config.json` according to the tools you want to track, in format `["Tool_1.mat","Tool_2.mat",...]`
- Set the AHAT stream on with either online data or offline data
- Click `Track`, you can choose whether to set Kalman Filter on
- The tracking data can be viewed in `Slicer` (scene provided in folder `\SlicerScene`) with following steps
- Slicer should be implemented with extensions `SlicerIGT` and `SlicerOpenIGTLink`
- Open Module `IGT\OpenIGTLinkF`
- Click the existed `IGTLConnector` and set it active (do not change the port and keep the port free)
- Click the check box `Slicer Display` in the UI
- Now the transform matrix have been sent to Slicer, the transformation of Tool_1 is linked to an coordinate model, you will find it moving in the scene if the tool can be recoganized and tracked


## Citation
If you use this for research, please cite. Here is an example BibTeX entry:
```
To be updated
```