# hsr

[マニュアル(esa)](https://demulab.esa.io/posts/235)
## サンプルプログラムのimport
(hsr.ioに登録したメアドとパスワードが必要)
```
 git clone https://git.hsr.io/tmc/hsrb_samples.git
```

## YOLO
```
cd ~/dspl_ws/src/hsr
mkdir darknet
cd darknet
git clone https://git.hsr.io/tmc/tmc_darknet.git
git clone https://git.hsr.io/tmc/tmc_darknet_ros.git
git clone https://git.hsr.io/tmc/hsrb_tk1_tutorials.git
cd ../../../
catkin build
```

# Downloading the detctoron2_ros
1.Install python Virtual Environment
```
sudo apt-get install python-pip
sudo pip install virtualenv
mkdir ~/.virtualenvs
sudo pip install virtualenvwrapper
export WORKON_HOME=~/.virtualenvs
echo '. /usr/local/bin/virtualenvwrapper.sh' >> ~/.bashrc 
```
2.Creating Virtual Environment
```
mkvirtualenv --python=python3 detectron2_ros
```
3.Install the dependencies in the virtual environment
```
pip install -U torch==1.4+cu100 torchvision==0.5+cu100 -f https://download.pytorch.org/whl/torch_stable.html
pip install cython pyyaml==5.1
pip install -U 'git+https://github.com/cocodataset/cocoapi.git#subdirectory=PythonAPI'
pip install detectron2 -f https://dl.fbaipublicfiles.com/detectron2/wheels/cu100/index.html
pip install opencv-python
pip install rospkg

```
4.
```
cd ~/dspl_ws/src/hsr/vision_pkg/include
git clone https://github.com/DavidFernandezChaves/detectron2_ros.git
cd detectron2_ros
git pull --all
git submodule update --init
```
https://github.com/Hibikino-Musashi-Home/hma_wrs_sim_ws
