# Install SOFA_v22.12.00_Win64.exe from Resources folder
Follow the README file for SOFA and Python Plugin

# install these
* pip install opencv-python
* pip install numpy
* pip install matplotlib
* pip install matlab
* pip install pyserial
* pip install inputs
* pip install keyboard
* pip install pygame
* pip install transforms3d

# if you do not have a NVidia GPU, install the PyTorch CPU-only version
* pip uninstall torch torchvision torchaudio
* pip cache purge
* pip install torch==2.0.0 --index-url https://download.pytorch.org/whl/cpu 

# Install eve and controller toolboxes from Lennart
Read the README fileinside each toolbox folder

# install JoyToKey
in JoyToKey folder, run "JoyToKey" or copy the wireless.cfg file to the JoyToKey installed config folder

# Note
Visual Code needs to "trust" the source to run/debug it (click on "manage" in the banner and hit "trust")