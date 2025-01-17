# BEFORE CLONING THIS REP
Install Git LFS in your local environment for handling large files
```
git lfs install
```
alternative in linux: 
```
sudo apt install git-lfs
```
Followed by:
```
git lfs track "*.zip"
git add .gitattributes
git commit -m "Add Git LFS tracking for large files"
```

# AFTER CLONING THIS REP
# Install SOFA_v22.12.00_Win64.exe from Resources folder
Follow the README file for SOFA and Python Plugin

# install these
```
pip install opencv-python
pip install numpy
pip install matplotlib
pip install matlab
pip install pyserial
pip install inputs
pip install keyboard
pip install pygame
pip install transforms3d
```

# Install eve and controller toolboxes from Lennart
Read and follow the README files inside each toolbox folder.

# install JoyToKey
In JoyToKey folder, run "JoyToKey" or copy the wireless.cfg file to the JoyToKey installed config folder

# Note
- Visual Code needs to "trust" the source to run/debug it (click on "manage" in the banner and hit "trust")
- If you do not have a NVidia GPU, install the PyTorch CPU-only version
```
pip uninstall torch torchvision torchaudio
pip cache purge
pip install torch==2.0.0 --index-url https://download.pytorch.org/whl/cpu 
```
