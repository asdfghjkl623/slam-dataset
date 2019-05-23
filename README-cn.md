

# Awesome SLAM Datasets Chinese [![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/sindresorhus/awesome)

![image](awesome_datasets_thumbnails.png)
>复杂城市的缩略图来自 NCLT, Oxford robotcar, KiTTi, Cityscapes 数据集.


此 GitHub Repo 是 SLAM 相关数据集的整合。我们整理出了提供姿位姿和地图信息的各种SLAM数据集。 在本 Repo 中，整个数据集图表表示为简化版本。 您可以在 [awesome-slam-datasets](https://sites.google.com/view/awesome-slam-datasets/) 中查看完整版表格（需要科学上网）。我们并对数据集进行了不同分类。


 

## 类别
- [评估工具](#评估工具)
  - Evaluation Methods of SLAM
- [主题](#按主题分类)
  - [里程计](#里程计): Dataset for odometry Benchmark
  - [建图](#建图): Dataset for mapping task
  - [多样场景](#多样场景): Dataset gives correspondences of places (images)
  - [定位](#定位): Dataset for metric-level localization
  - [感知](#感知): Dataset with semantic labels / correspondences

- [特点](#按特点分类)
  - [大规模](#大规模): City-scale map, kilometer level Map
  - [长时间](#长时间): Multi-session, long-term data collection
  - [复杂地图](#复杂地图): Variation of mapping structures
  - [极端情况](#极端情况): Extreme environment, motions

- [载体平台](#按载体平台分类)
    - [汽车](#汽车): Commercial Vehicle (Four-wheeled on the road)
    - [移动机器人](#移动机器人): Mobile Robots (Ex. Husky, Rover.. )
    - [无人机](#无人机): Unmanned aerial robots include drone.
    - [水下自主航行器](#水下自主航行器): Underwater robots include ROV for simplicity.
    - [水面无人艇](#水面无人艇): Water surface vehicle such as canoe and boat.
    - [手持设备](#手持设备): Hand-held platform by human

- [环境](#按环境分类)
    - [城市](#城市): City, campus, town, and infrastructures
    - [室内](#室内): Indoor environment
    - [地形](#地形): Rough terrain, underground, lake and farm
    - [水下](#水下): Underwater floor, cave

## 数据集表格总览 (简版)
点击[查看完整版链接](https://sites.google.com/view/awesome-slam-datasets/)

| 名称                                                                                                | 机构  | 年份 | 载体平台   | 出版来源 | 环境           | 是否有 GT-Pose | GT-Map | IMU | GPS | 标签 | Lidar      | Camera | RGBD | 事件相机 | Radar | 声呐 | 多普勒速度记录 | 其他                   |
|----------------------------------------------------------------------------------------------------------|--------------|------|------------|-------------|-----------------------|---------|--------|-----|-----|--------|------------|---------|------|-------|-------|-------|-----|-------------------------|
| [Collaborative SLAM Dataset (CSD)](https://github.com/torrvision/CollaborativeSLAMDataset)               | Oxford       | 2018 | Hand       | TVCG/ISMAR  | Indoor                | O       | O      | O   |     |        |            | O       | O    |       |       |       |     | Tango (Asus ZenFone AR)
| [ADVIO Dataset](https://github.com/AaltoVision/ADVIO)               | Aalto U          | 2018 | Hand | ECCV                | Urban      | O | O | O |   |   |   | O |   |   |   |   |   | iPhone, Tango, Pixel                     |
| [DeepIO Dataset](http://deepio.cs.ox.ac.uk/)                        | Oxford           | 2018 | Hand | Arxiv               | Indoor     | O |   | O |   |   |   |   |   |   |   |   |   |                                          |
| [Aqualoc Dataset](http://www.lirmm.fr/aqualoc/)                     | ONERA-DTIS       | 2018 | ROV  | IROS WS             | Underwater | O |   | O |   |   |   | O |   |   |   |   |   | Pressure Sensor                          |
| [Rosario Dataset](http://www.cifasis-conicet.gov.ar/robot/doku.php) | CONICET-UNR      | 2018 | Mob  | IJRR (Under Review) | Terrain    | O |   | O |   |   |   | O |   |   |   |   |   | Encoder                                  |
| [InteriorNet](https://interiornet.org/)                             | Imperial College | 2018 | Hand | BMVC                | Indoor     | O | O | O |   | O |   | O | O | O |   |   |   | Texture, Lighting, Context, Optical Flow |
| [SPO Dataset](https://www.hs-karlsruhe.de/odometry-data/)           | TUM, Karlsruhe   | 2018 | Hand | Arxiv               | Urban      | O |   |   |   |   |   | O |   |   |   |   |   | Plenoptic Camera                         |
| [Complex Urban](http://irap.kaist.ac.kr/dataset/)                                                        | KAIST-IRAP   | 2018 | Veh        | ICRA        | Urban                 | O       | O      | O   | O   |        | O          |         |      |       |       |       |     | Encoder                 |
| [KAIST Day/Night](https://sites.google.com/view/multispectral/home)                                      | KAIST-RCV    | 2018 | Veh        | T-ITS       | Urban                 | O       |        | O   | O   | O      | O          | O       |      |       |       |       |     | Thermal Camera          |
| [TUM-Visual-Inertial](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)                    | TUM          | 2018 | Hand       | Arxiv       | Indoor, Urban         |         |        | O   |     |        |            |         | O    |       | O     |       |     |                         |
| [Multi Vech Event](https://daniilidis-group.github.io/mvsec/)                                            | Upenn        | 2018 | Veh        | RA-L        | Urban                 | O       |        | O   | O   |        | O          | O       |      | O     |       |       |     |                         |
| [VI Canoe](https://databank.illinois.edu/datasets/IDB-9342111)                                           | UIUC         | 2018 | USV        | IJRR        | Terrain               | O       |        | O   | O   |        |            | O       |      |       |       |       |     |                         |
| [MPO-Japan](http://robotics.ait.kyushu-u.ac.jp/kurazume_lab/research-e.php?content=db)                   | ETH-RPG      | 2017 | UAV / Hand | IJRR        | Indoor                | O       |        | O   |     |        |            | O       |      | O     |       |       |     |                         |
| [Underwater Cave](http://cirs.udg.edu/caves-dataset/)                                                    | UDG          | 2017 | AUV        | IJRR        | Underwater            | O       |        | O   |     |        |            | O       |      |       |       | O     | O   | Profiling Sonar         |
| [Robot @ Home](http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/203-robot-at-home-dataset) | MRPT         | 2017 | Mob        | IJRR        | Indoor                | O       | O      |     |     | O      | O          |         | O    |       |       |       |     | Semantic Labels         |
| [Zurich Urban MAV](http://rpg.ifi.uzh.ch/zurichmavdataset.html)                                          | ETH-RPG      | 2017 | UAV        | IJRR        | Urban                 | O       |        | O   | O   |        |            | O       |      |       |       |       |     | Streetview images       |
| [Chilean Underground](http://dataset.amtc.cl/#)                                                          | Trimble      | 2017 | Mob        | IJRR        | Terrain (Underground) | O       |        |     |     |        | O          | O       |      |       | O     |       |     | Encoder                 |
| [SceneNet RGB-D](https://robotvault.bitbucket.io/scenenet-rgbd.html)                                     | Imperial     | 2017 | Hand       | ICCV        | Indoor                | O       |        |     |     | O      |            |         | O    |       |       |       |     |                         |
| [Symphony Lake](http://dream.georgiatech-metz.fr/?q=node/79)                                             | Georgia Tech | 2017 | USV        | IJRR        | Terrain (Lake)        |         |        | O   | O   |        | O          | O       |      |       |       |       |     | PTZ camera, Longterm    |
| [Agricultural robot](http://www.ipb.uni-bonn.de/data/sugarbeets2016/)                                    | Bonn         | 2017 | Mob        | IJRR        | Terrain               | O       |        |     | O   | O      | O          | O       | O    |       |       |       |     | Multispectral camera    |
| [Beach Rover](https://robotics.estec.esa.int/datasets/katwijk-beach-11-2015/)                            | TEC-MMA      | 2017 | Mob        |             | Terrain               | O       |        | O   | O   |        | O          | O       | O    |       |       |       |     | Encoder                 |
| [EuRoC](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)                     | ETH-ASL      | 2016 | UAV        | IJRR        | Indoor                | O       | O      | O   |     |        |            | O       |      |       |       |       |     |                         |
| [Cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/data.html)                     | Google      | 2016 | Hand        | ICRA        | Indoor                |         |        | O   |     |        | O          | O       |      |       |       |       |     |                         |
| [TUM-Mono](https://vision.in.tum.de/data/datasets/mono-dataset)                                          | TUM          | 2016 | Hand       | Arxiv       | Indoor, Urban         |         |        |     |     |        |            |         |      |       | O     |       |     | Photometric Calibration |
| [Cityscape](https://www.cityscapes-dataset.com/)                                                         | Daimler AG   | 2016 | Veh        | CVPR        | Urban                 | O       |        |     | O   | O      |            | O       |      |       |       |       |     |                         |
| [Solar-UAV](http://projects.asl.ethz.ch/datasets/doku.php?id=fsr2015)                                    | ETHZ         | 2016 | UAV        | CVPR        | Terrain               | O       | O      | O   | O   |        | O          |         |      |       |       |       |     |                         |
| [CoRBS](http://corbs.dfki.uni-kl.de/?pagerd_tumlltzzf42zsv6de7b9)                                        | DFKI         | 2016 | Hand       | WACV        | Indoor                | O       | O      |     |     |        |            |         | O    |       |       |       |     |                         |
| [Oxford-robotcar](http://robotcar-dataset.robots.ox.ac.uk)                                               | Oxford       | 2016 | Veh        | IJRR        | Urban                 | O       |        | O   | O   |        | O          | O       |      |       |       |       |     |                         |
| [NCLT](http://robots.engin.umich.edu/nclt/)                                                              | UMich        | 2016 | Mob        | IJRR        | Urban                 | O       |        | O   | O   |        | O          |         |      |       |       |       |     | FOG                     |
| [RPG-event](http://rpg.ifi.uzh.ch/davis_data.html)                                                       | Kyushu U     | 2016 | Veh        | IROS        | Urban, Terrain        |         |        | O   | O   |        | O          | O       |      |       |       |       |     | FARO 3D                 |
| [CCSAD](http://aplicaciones.cimat.mx/Personal/jbhayet/ccsad-dataset)                                     | CIMAT        | 2015 | Veh        | CAIP        | Urban                 |         |        | O   | O   |        |            | O       |      |       |       |       |     |                         |
| [TUM-Omni](https://vision.in.tum.de/data/datasets/omni-lsdslam)                                          | TUM          | 2015 | Hand       | IROS        | Indoor, Urban         |         |        |     |     |        |            | O       |      |       |       |       |     |                         |
| [Augmented ICL-NUIM](http://redwood-data.org/indoor/index.html)                                          | Redwood      | 2015 | Hand       | CVPR        | Indoor                | O       | O      |     |     |        |            |         | O    |       |       |       |     |                         |
| [Cambridge Landmark](http://mi.eng.cam.ac.uk/projects/relocalisation/)                                   | Cambridge    | 2015 | Hand       | ICCV        | Urban                 | O       | O      |     |     |        |            | O       |      |       |       |       |     |                         |
| [ICL-NUIM](https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html)                                         | Imperial     | 2014 | Hand       | ICRA        | Indoor                | O       | O      |     |     |        |            |         | O    |       |       |       |     |                         |
| [MRPT-Malaga](https://www.mrpt.org/robotics_datasets)                                                    | MRPT         | 2014 | Veh        | AR          | Urban                 |         |        | O   | O   |        | O          | O       |      |       |       |       |     |                         |
| [KITTI](http://www.cvlibs.net/datasets/kitti/index.php)                                                  | KIT          | 2013 | Veh        | IJRR        | Urban                 | O       |        | O   | O   | O      | O          | O       |      |       |       |       |     |                         |
| [Canadian Planetary](http://asrl.utias.utoronto.ca/datasets/3dmap/#Datasets)                             | UToronto     | 2013 | Mob        | IJRR        | Terrain               | O       |        | O   | O   |        | O (sensor) | O       |      |       |       |       |     |                         |
| [Microsoft 7 scenes](https://www.microsoft.com/en-us/research/project/rgb-d-dataset-7-scenes/)                               | Microsoft    | 2013 | Hand       | CVPR        | Indoor                | O       | O      |     |     |        |            | O       |      |       |       |       |     |                         |
| [SeqSLAM](https://wiki.qut.edu.au/display/cyphy/Open+datasets+and+software)                              | QUT          | 2012 | Veh        | ICRA        | Urban                 |         |        |     |     | O      |            | O       |      |       |       |       |     |                         |
| [ETH-challenging](http://projects.asl.ethz.ch/datasets/doku.php?id=laserregistration:laserregistration)  | ETH-ASL      | 2012 | Hand       | IJRR        | Urban, Terrain        |         |        | O   | O   |        | O          | O       | O    |       |       |       |     |                         |
| [TUM-RGBD](https://vision.in.tum.de/data/datasets/rgbd-dataset)                                          | TUM          | 2012 | Hand / Mob | IROS        | Indoor                | O       |        | O   |     |        |            |         | O    |       |       |       |     |                         |
| [ASRL-Kagara-airborne](http://asrl.utias.utoronto.ca/~mdw/kagarudataset.html)                            | UToronto     | 2012 | UAV        | FSR         | Terrain               |         |        | O   | O   |        |            | O       |      |       |       |       |     |                         |
| [Devon Island Rover](http://asrl.utias.utoronto.ca/datasets/devon-island-rover-navigation/)              | UToronto     | 2012 | Mob        | IJRR        | Terrain               | O       |        |     | O   |        |            | O       |      |       |       |       |     | Sunsensor, Inclinometer |
| [ACFR Marine](http://marine.acfr.usyd.edu.au/datasets/)                                                  | ACFR         | 2012 | AUV        |             | Underwater            | O       |        | O   |     | O      |            | O       |      |       |       | O     |     |                         |
| [UTIAS Multi-Robot](http://asrl.utias.utoronto.ca/datasets/mrclam/)                                      | UT-IAS       | 2011 | Mob        | IJRR        | Urban                 | O       |        |     |     | O      |            |         |      |       |       |       |     |                         |
| [Ford Campus](http://robots.engin.umich.edu/SoftwareData/Ford)                                           | UMich        | 2011 | Veh        | IJRR        | Urban                 | O       |        | O   | O   |        | O          | O       |      |       |       |       |     |                         |
| [San francisco](https://sites.google.com/site/chenmodavid/datasets)                                      | Stanford     | 2011 | Veh        | CVPR        | Urban                 | O       |        | O   | O   | O      |            | O       |      |       |       |       |     | DMI                     |
| [Annotated-laser](http://any.csie.ntu.edu.tw/data)                                                       | NTU          | 2011 | Veh        | IJRR        | Urban                 | O       |        |     |     | O      | O          | O       |      |       |       |       |     |                         |
| [MIT-DARPA](http://grandchallenge.mit.edu/wiki/index.php?title=PublicData)                               | MIT          | 2010 | Veh        | IJRR        | Urban                 | O       |        | O   | O   | O      | O          | O       |      |       |       |       |     |                         |
| [St Lucia Stereo](http://asrl.utias.utoronto.ca/~mdw/uqstluciadataset.html)                              | UToronto     | 2010 | Veh        | ACRA        | Urban                 |         |        | O   | O   |        |            | O       |      |       |       |       |     |                         |
| [St Lucia Multiple Times](https://wiki.qut.edu.au/display/cyphy/St+Lucia+Multiple+Times+of+Day)          | QUT          | 2010 | Veh        | ICRA        | Urban                 |         |        |     | O   |        |            | O       |      |       |       |       |     |                         |
| [Marulan](http://sdi.acfr.usyd.edu.au/)                                                                  | ACFR         | 2010 | Mob        | IJRR        | Terrain               | O       |        | O   | O   |        | O          | O       |      |       | O     |       |     | IR                      |
| [COLD](https://www.pronobis.pro/#data)                                                                   | KTH          | 2009 | Hand       | IJRR        | Indoor                | O       |        |     |     | O      | O          | O       |      |       |       |       |     |                         |
| [NewCollege](http://www.robots.ox.ac.uk/NewCollegeData/)                                                 | Oxford-Robot | 2009 | Mob        | IJRR        | Urban                 | O       |        |     | O   |        | O          | O       |      |       |       |       |     |                         |
| [Rawseeds-indoor](http://www.rawseeds.org/home/category/benchmarking-toolkit/datasets/)                  | Milano       | 2009 | Mob        | IROSW       | Indoor                | O       | O      | O   |     |        | O          | O       |      |       |       | O     |     |                         |
| [Rawseeds-outdoor](http://www.rawseeds.org/home/category/benchmarking-toolkit/datasets/)                 | Milano       | 2009 | Mob        | IROSW       | Urban                 | O       | O      | O   | O   |        | O          | O       |      |       |       | O     |     |                         |
| [FABMAP](http://www.robots.ox.ac.uk/~mobile/IJRR_2008_Dataset/)                                          | Oxford-Robot | 2008 | Veh        | IJRR        | Urban                 |         |        |     | O   |        |            | O       |      |       |       |       |     |                         |


## 评估工具
_Evaluation methods for SLAM benchmarks_
- Trajectory Evaluation with Alignment [[Paper](http://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf)], [[Code](https://github.com/uzh-rpg/rpg_trajectory_evaluation)]
- Python package for the evaluation of odometry and SLAM [[Code](https://michaelgrupp.github.io/evo/)]
- SLAMBench2.0: SLAM performance evaluation framework [[Code](https://github.com/pamela-project/slambench2)]

## 按主题分类

### 里程计
_Dataset for odometry Benchmark_
- [TUM-Visual-Inertial](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)
- [Visual-Inertial Canoe Dataset](https://databank.illinois.edu/datasets/IDB-9342111)
- [Multi Vehicle Stereo Event Camera Dataset](https://docs.google.com/spreadsheets/d/1mudM7LxXv09ywuQGDp3t_RlIjIdwzg_ZaMu78agLmH4/edit#gid=0)
- [Zurich Urban Micro Aerial Vehicle Dataset](http://rpg.ifi.uzh.ch/zurichmavdataset.html)
- [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
- [TUM Monocular Cameras Dataset](https://vision.in.tum.de/data/datasets/mono-dataset)
- [Event-Camera Dataset and Simulator](http://rpg.ifi.uzh.ch/davis_data.html)
- [TUM Omnidirectional Cameras Dataset](https://vision.in.tum.de/data/datasets/omni-lsdslam)
- [ICL-NUIM RGBD Dataset](https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html)
- [TUM RGB-D SLAM Dataset and Benchmark](https://vision.in.tum.de/data/datasets/rgbd-dataset)
- [Google Cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/data.html)
- [ADVIO Dataset](https://github.com/AaltoVision/ADVIO)
- [Deep Inertial Odometry Dataset](http://deepio.cs.ox.ac.uk/)
- [Aqualoc Underwater Dataset](http://www.lirmm.fr/aqualoc/)
- [Rosario Agricultural Dataset](http://www.cifasis-conicet.gov.ar/robot/doku.php)
- [Stereo Plenoptic Odometry Dataset](https://www.hs-karlsruhe.de/odometry-data/)


### 建图
_Dataset for mapping task_
- [Collaborative SLAM Dataset (CSD)](https://github.com/torrvision/CollaborativeSLAMDataset)
- [Complex Urban](http://irap.kaist.ac.kr/dataset/)
- [Multi-modal Panoramic 3D Outdoor Dataset (MPO)](http://robotics.ait.kyushu-u.ac.jp/kurazume_lab/research-e.php?content=db)
- [Underwater Caves SONAR and Vision Dataset](http://cirs.udg.edu/caves-dataset/)
- [Chilean Underground Mine Dataset](http://dataset.amtc.cl/#)
- [Oxford Robotcar Dataset](http://robotcar-dataset.robots.ox.ac.uk/)
- [University of Michigan North Campus Long-Term (NCLT) Vision and LIDAR Dataset](http://robots.engin.umich.edu/nclt/)
- [Málaga Stereo and Laser Urban Data Set](https://www.mrpt.org/MalagaUrbanDataset)
- [KITTI Vision Benchmark Suite](http://www.cvlibs.net/datasets/kitti/index.php)
- [Challenging data sets for point cloud registration algorithms](https://projects.asl.ethz.ch/datasets/doku.php?id=laserregistration:laserregistration)
- [ACFR Marine Robotics Dataset](http://marine.acfr.usyd.edu.au/datasets/)
- [Ford Campus Vision and Lidar Dataset](http://robots.engin.umich.edu/SoftwareData/Ford)
- [InteriorNet](https://interiornet.org/)

### 多样场景
_Dataset gives correspondences of places (images)_
- [Visual-Inertial Canoe Dataset](https://databank.illinois.edu/datasets/IDB-9342111)
- [Symphony Lake Dataset](http://dream.georgiatech-metz.fr/?q=node/79)
- [Alderley Day/Night Dataset](https://wiki.qut.edu.au/pages/viewpage.action?pageId=181178395)
- [St Lucia Multiple Times of Day](https://wiki.qut.edu.au/display/cyphy/St+Lucia+Multiple+Times+of+Day)
- [New College Vision and Laser Data Set](http://www.robots.ox.ac.uk/NewCollegeData/)
- [FABMAP Dataset](http://www.robots.ox.ac.uk/~mobile/IJRR_2008_Dataset/)

### 定位
_Dataset for metric-level localization_
- [Cambridge Landmark Dataset](http://mi.eng.cam.ac.uk/projects/relocalisation/)
- [KITTI Vision Benchmark Suite](http://www.cvlibs.net/datasets/kitti/index.php)
- [Microsoft 7 scenes](https://www.microsoft.com/en-us/research/project/rgb-d-dataset-7-scenes/)
- [San Francisco Landmark Dataset](https://sites.google.com/site/chenmodavid/datasets)


### 感知
_Dataset with semantic labels / correspondences_
- [KAIST Day/Night Dataset](https://sites.google.com/view/multispectral/home)
- [Robot @ Home Dataset](http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/203-robot-at-home-dataset)
- [SceneNet RBG-D Dataset](https://robotvault.bitbucket.io/scenenet-rgbd.html)
- [Sugar Beets 2016, Agricultural Robot Dataset](http://www.ipb.uni-bonn.de/data/sugarbeets2016/)
- [CityScapes Dataset](https://www.cityscapes-dataset.com/)
- [KITTI Vision Benchmark Suite](http://www.cvlibs.net/datasets/kitti/index.php)
- [Multi-Sensor Perception (Marulan) Dataset ](http://sdi.acfr.usyd.edu.au/)
- [InteriorNet](https://interiornet.org/)

## 按特点分类

### 大规模
_City-scale map, kilometer level Map_
- [Complex Urban](http://irap.kaist.ac.kr/dataset/)
- [Multi Vehicle Stereo Event Camera Dataset](https://docs.google.com/spreadsheets/d/1mudM7LxXv09ywuQGDp3t_RlIjIdwzg_ZaMu78agLmH4/edit#gid=0)
- [Multi-modal Panoramic 3D Outdoor Dataset (MPO)](http://robotics.ait.kyushu-u.ac.jp/kurazume_lab/research-e.php?content=db)
- [CityScapes Dataset](https://www.cityscapes-dataset.com/)
- [Solar-powered UAV Sensing and Mapping Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=fsr2015)
- [Oxford Robotcar Dataset](http://robotcar-dataset.robots.ox.ac.uk/)
- [CCSAD (Stereo Urban) Dattaset](http://aplicaciones.cimat.mx/Personal/jbhayet/ccsad-dataset)
- [Málaga Stereo and Laser Urban Data Set](https://www.mrpt.org/MalagaUrbanDataset)
- [KITTI Vision Benchmark Suite](http://www.cvlibs.net/datasets/kitti/index.php)
- [Kagaru Airborne Stereo Dataset Dataset](http://asrl.utias.utoronto.ca/~mdw/kagarudataset.html)
- [ACFR Marine Robotics Dataset](http://marine.acfr.usyd.edu.au/datasets/)

### 长时间
_Multi-session, long-term data collection_
- [KAIST Day/Night Dataset](https://sites.google.com/view/multispectral/home)
- [Visual-Inertial Canoe Dataset](https://databank.illinois.edu/datasets/IDB-9342111)
- [Symphony Lake Dataset](http://dream.georgiatech-metz.fr/?q=node/79)
- [Oxford Robotcar Dataset](http://robotcar-dataset.robots.ox.ac.uk/)
- [University of Michigan North Campus Long-Term (NCLT) Vision and LIDAR Dataset](http://robots.engin.umich.edu/nclt/)
- [Alderley Day/Night Dataset](https://wiki.qut.edu.au/pages/viewpage.action?pageId=181178395)
- [St Lucia Multiple Times of Day](https://wiki.qut.edu.au/display/cyphy/St+Lucia+Multiple+Times+of+Day)

### 复杂地图
_Variation of mapping structures_
- [Complex Urban](http://irap.kaist.ac.kr/dataset/)
- [Multi Vehicle Stereo Event Camera Dataset](https://docs.google.com/spreadsheets/d/1mudM7LxXv09ywuQGDp3t_RlIjIdwzg_ZaMu78agLmH4/edit#gid=0)
- [Multi-modal Panoramic 3D Outdoor Dataset (MPO)](http://robotics.ait.kyushu-u.ac.jp/kurazume_lab/research-e.php?content=db)
- [Málaga Stereo and Laser Urban Data Set](https://www.mrpt.org/MalagaUrbanDataset)
- [KITTI Vision Benchmark Suite](http://www.cvlibs.net/datasets/kitti/index.php)
- [Challenging data sets for point cloud registration - algorithms](https://projects.asl.ethz.ch/datasets/doku.php?id=laserregistration:laserregistration)

### 极端情况
_Extreme environment, motions_
- [Underwater Caves SONAR and Vision Dataset](http://cirs.udg.edu/caves-dataset/): Underwater Environment
- [Chilean Underground Mine Dataset](http://dataset.amtc.cl/#): Underground Environment
- [CityScapes Dataset](https://www.cityscapes-dataset.com/): Foggy Scene
- [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets): Fast motion
- [Multi-Sensor Perception (Marulan) Dataset ](http://sdi.acfr.usyd.edu.au/):  Smoky, dust, and Rain condition

## 按载体平台分类

### 汽车
_Commercial Vehicle (Four-wheeled on the road)_
- [Complex Urban Dataset](http://irap.kaist.ac.kr/dataset/)
- [Multi Vehicle Stereo Event Camera Dataset](https://docs.google.com/spreadsheets/d/1mudM7LxXv09ywuQGDp3t_RlIjIdwzg_ZaMu78agLmH4/edit#gid=0)
- [KAIST Day/Night Dataset](https://sites.google.com/view/multispectral/home)
- [Multi-modal Panoramic 3D Outdoor Dataset (MPO)](http://robotics.ait.kyushu-u.ac.jp/kurazume_lab/research-e.php?content=db)
- [Oxford Robotcar Dataset](http://robotcar-dataset.robots.ox.ac.uk/)
- [CityScapes Dataset](https://www.cityscapes-dataset.com/)
- [CCSAD (Stereo Urban) Dattaset](http://aplicaciones.cimat.mx/Personal/jbhayet/ccsad-dataset)
- [Málaga Stereo and Laser Urban Data Set](https://www.mrpt.org/MalagaUrbanDataset)
- [KITTI Vision Benchmark Suite](http://www.cvlibs.net/datasets/kitti/index.php)
- [Day and Night with Lateral Pose Change Dataset](https://wiki.qut.edu.au/display/cyphy/Day+and+Night+with+Lateral+Pose+Change+Datasets)
- [Alderley Day/Night Dataset](https://wiki.qut.edu.au/pages/viewpage.action?pageId=181178395)
- [Annotated-laser Dataset](http://any.csie.ntu.edu.tw/data) (Link Broken)
- [San Francisco Landmark Dataset](https://sites.google.com/site/chenmodavid/datasets)
- [Ford Campus Vision and Lidar Dataset](http://robots.engin.umich.edu/SoftwareData/Ford)
- [St Lucia Stereo Vehicular Dataset](http://asrl.utias.utoronto.ca/~mdw/uqstluciadataset.html)
- [St Lucia Multiple Times of Day](https://wiki.qut.edu.au/display/cyphy/St+Lucia+Multiple+Times+of+Day)
- [MIT DARPA Urban Challenge Dataset](http://grandchallenge.mit.edu/wiki/index.php?title=PublicData)
- [FABMAP Dataset](http://www.robots.ox.ac.uk/~mobile/IJRR_2008_Dataset/)


### 移动机器人
_Mobile Robots (Ex. Husky, Rover.. )_
- [Rosario Dataset](http://www.cifasis-conicet.gov.ar/robot/doku.php)
- [Sugar Beets 2016, Agricultural Robot Dataset](http://www.ipb.uni-bonn.de/data/sugarbeets2016/)
- [Chilean Underground Mine Dataset](http://dataset.amtc.cl/#)
- [Katwijk Beach Planetary Rover Dataset](https://robotics.estec.esa.int/datasets/katwijk-beach-11-2015/)
- [Robot @ Home Dataset](http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/203-robot-at-home-dataset)
- [University of Michigan North Campus Long-Term (NCLT) Vision and LIDAR Dataset](http://robots.engin.umich.edu/nclt/)
- [Rawseeds In/Outdoor Dataset](http://www.rawseeds.org/home/category/benchmarking-toolkit/datasets/)
- [Canadian Planetary Emulation Terrain 3D Mapping Dataset](http://asrl.utias.utoronto.ca/datasets/3dmap/#Datasets)
- [Devon Island Rover Navigation Dataset](http://asrl.utias.utoronto.ca/datasets/devon-island-rover-navigation/)
- [Multi-Robot Cooperative Localization and Mapping Dataset](http://asrl.utias.utoronto.ca/datasets/mrclam/)
- [Multi-Sensor Perception (Marulan) Dataset ](http://sdi.acfr.usyd.edu.au/)
- [TUM RGB-D SLAM Dataset and Benchmark](https://vision.in.tum.de/data/datasets/rgbd-dataset)
- [New College Vision and Laser Data Set](http://www.robots.ox.ac.uk/NewCollegeData/)

### 无人机
_Unmanned aerial robots include drone_
- [Zurich Urban Micro Aerial Vehicle Dataset](http://rpg.ifi.uzh.ch/zurichmavdataset.html)
- [Event-Camera Dataset and Simulator](http://rpg.ifi.uzh.ch/davis_data.html)
- [Solar-powered UAV Sensing and Mapping Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=fsr2015)
- [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
- [Kagaru Airborne Stereo Dataset Dataset](http://asrl.utias.utoronto.ca/~mdw/kagarudataset.html)




### 水下自主航行器
_Underwater robots include ROV for simplicity_
- [Aqualoc Underwater Dataset](http://www.lirmm.fr/aqualoc/)
- [Underwater Caves SONAR and Vision Dataset](http://cirs.udg.edu/caves-dataset/)
- [ACFR Marine Robotics Dataset](http://marine.acfr.usyd.edu.au/datasets/)


### 水面无人艇
_Water surface vehicle such as canoe and boat_
- [Visual-Inertial Canoe Dataset](https://databank.illinois.edu/datasets/IDB-9342111)
- [Symphony Lake Dataset](http://dream.georgiatech-metz.fr/?q=node/79)


### 手持设备
_Hand-held platform by human_
- [Collaborative SLAM Dataset (CSD)](https://github.com/torrvision/CollaborativeSLAMDataset)
- [SceneNet RBG-D Dataset](https://robotvault.bitbucket.io/scenenet-rgbd.html)
- [Event-Camera Dataset and Simulator](http://rpg.ifi.uzh.ch/davis_data.html)
- [Comprehensive RGB-D Benchmark (CoRBS)](http://corbs.dfki.uni-kl.de/?pagerd_tumlltzzf42zsv6de7b9)
- [Augmented ICL-NUIM Reconstruction Dataset](http://redwood-data.org/indoor/index.html)
- [ICL-NUIM RGBD Dataset](https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html)
- [Challenging data sets for point cloud registration algorithms](https://projects.asl.ethz.ch/datasets/doku.php?id=laserregistration:laserregistration)
- [Cosy Localization Database (COLD)](https://www.pronobis.pro/#data)
- [ADVIO Dataset](https://github.com/AaltoVision/ADVIO)
- [Deep Inertial Odometry Dataset](http://deepio.cs.ox.ac.uk/)
- [InteriorNet](https://interiornet.org/)
- [Stereo Plenoptic Dataset](https://www.hs-karlsruhe.de/odometry-data/)

## 按环境分类
### 城市
_City, campus, town, and infrastructures_
- [ADVIO Dataset](https://github.com/AaltoVision/ADVIO)
- [Stereo Plenoptic Dataset](https://www.hs-karlsruhe.de/odometry-data/)
- [KAIST Day/Night Dataset](https://sites.google.com/view/multispectral/home)
- [TUM-Visual-Inertial](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)
- [Complex Urban](http://irap.kaist.ac.kr/dataset/)
- [Multi Vehicle Stereo Event Camera Dataset](https://docs.google.com/spreadsheets/d/1mudM7LxXv09ywuQGDp3t_RlIjIdwzg_ZaMu78agLmH4/edit#gid=0)
- [Zurich Urban Micro Aerial Vehicle Dataset](http://rpg.ifi.uzh.ch/zurichmavdataset.html)
- [TUM Monocular Cameras Dataset](https://vision.in.tum.de/data/datasets/mono-dataset)
- [CityScapes Dataset](https://www.cityscapes-dataset.com/)
- [Oxford Robotcar Dataset](http://robotcar-dataset.robots.ox.ac.uk/)
- [University of Michigan North Campus Long-Term (NCLT) Vision and LIDAR Dataset](http://robots.engin.umich.edu/nclt/)
- [Event-Camera Dataset and Simulator](http://rpg.ifi.uzh.ch/davis_data.html)
- [CCSAD (Stereo Urban) Dattaset](http://aplicaciones.cimat.mx/Personal/jbhayet/ccsad-dataset)
- [TUM Omnidirectional Cameras Dataset](https://vision.in.tum.de/data/datasets/omni-lsdslam)
- [Cambridge Landmark Dataset](http://mi.eng.cam.ac.uk/projects/relocalisation/)
- [Málaga Stereo and Laser Urban Data Set](https://www.mrpt.org/MalagaUrbanDataset)
- [KITTI Vision Benchmark Suite](http://www.cvlibs.net/datasets/kitti/index.php)
- [Alderley Day/Night Dataset](https://wiki.qut.edu.au/pages/viewpage.action?pageId=181178395)
- [Challenging data sets for point cloud registration algorithms](https://projects.asl.ethz.ch/datasets/doku.php?id=laserregistration:laserregistration)
- [Multi-Robot Cooperative Localization and Mapping Dataset](http://asrl.utias.utoronto.ca/datasets/mrclam/)
- [Ford Campus Vision and Lidar Dataset](http://robots.engin.umich.edu/SoftwareData/Ford)
- [San Francisco Landmark Dataset](https://sites.google.com/site/chenmodavid/datasets)
- [Annotated-laser Dataset](http://any.csie.ntu.edu.tw/data) (Link Broken)
- [MIT DARPA Urban Challenge Dataset](http://grandchallenge.mit.edu/wiki/index.php?title=PublicData)
- [St Lucia Stereo Vehicular Dataset](http://asrl.utias.utoronto.ca/~mdw/uqstluciadataset.html)
- [St Lucia Multiple Times of Day](https://wiki.qut.edu.au/display/cyphy/St+Lucia+Multiple+Times+of+Day)
- [New College Vision and Laser Data Set](http://www.robots.ox.ac.uk/NewCollegeData/)
- [Rawseeds In/Outdoor Dataset](http://www.rawseeds.org/home/category/benchmarking-toolkit/datasets/)
- [FABMAP Dataset](http://www.robots.ox.ac.uk/~mobile/IJRR_2008_Dataset/)

### 室内
_Indoor environment_
- [Collaborative SLAM Dataset (CSD)](https://github.com/torrvision/CollaborativeSLAMDataset)
- [InteriorNet](https://interiornet.org/)
- [TUM-Visual-Inertial](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)
- [Multi-modal Panoramic 3D Outdoor Dataset (MPO)](http://robotics.ait.kyushu-u.ac.jp/kurazume_lab/research-e.php?content=db)
- [Robot @ Home Dataset](http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/203-robot-at-home-dataset)
- [SceneNet RBG-D Dataset](https://robotvault.bitbucket.io/scenenet-rgbd.html)
- [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
- [TUM Monocular Cameras Dataset](https://vision.in.tum.de/data/datasets/mono-dataset)
- [Comprehensive RGB-D Benchmark (CoRBS)](http://corbs.dfki.uni-kl.de/?pagerd_tumlltzzf42zsv6de7b9)
- [TUM Omnidirectional Cameras Dataset](https://vision.in.tum.de/data/datasets/omni-lsdslam)
- [Augmented ICL-NUIM Reconstruction Dataset](http://redwood-data.org/indoor/index.html)
- [ICL-NUIM RGBD Dataset](https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html)
- [Microsoft 7 scenes](https://www.microsoft.com/en-us/research/project/rgb-d-dataset-7-scenes/)
- [TUM RGB-D SLAM Dataset and Benchmark](https://vision.in.tum.de/data/datasets/rgbd-dataset)
- [Cosy Localization Database (COLD)](https://www.pronobis.pro/#data)
- [Rawseeds In/Outdoor Dataset](http://www.rawseeds.org/home/category/benchmarking-toolkit/datasets/)
- [Google Cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/data.html)


### 地形
_Rough terrain, underground, lake and farm_
- [Rosario Agricultural Dataset](http://www.cifasis-conicet.gov.ar/robot/doku.php)
- [Visual-Inertial Canoe Dataset](https://databank.illinois.edu/datasets/IDB-9342111)
- [Chilean Underground Mine Dataset](http://dataset.amtc.cl/#)
- [Symphony Lake Dataset](http://dream.georgiatech-metz.fr/?q=node/79)
- [Sugar Beets 2016, Agricultural Robot Dataset](http://www.ipb.uni-bonn.de/data/sugarbeets2016/)
- [Katwijk Beach Planetary Rover Dataset](https://robotics.estec.esa.int/datasets/katwijk-beach-11-2015/)
- [Solar-powered UAV Sensing and Mapping Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=fsr2015)
- [Event-Camera Dataset and Simulator](http://rpg.ifi.uzh.ch/davis_data.html)
- [Canadian Planetary Emulation Terrain 3D Mapping Dataset](http://asrl.utias.utoronto.ca/datasets/3dmap/#Datasets)
- [Challenging data sets for point cloud registration - algorithms](https://projects.asl.ethz.ch/datasets/doku.php?id=laserregistration:laserregistration)
- [Kagaru Airborne Stereo Dataset Dataset](http://asrl.utias.utoronto.ca/~mdw/kagarudataset.html)
- [Devon Island Rover Navigation Dataset](http://asrl.utias.utoronto.ca/datasets/devon-island-rover-navigation/)
- [Multi-Sensor Perception (Marulan) Dataset ](http://sdi.acfr.usyd.edu.au/)


### 水下
_Underwater floor, cave_
- [Aqualoc Underwater Dataset](http://www.lirmm.fr/aqualoc/)
- [Underwater Caves SONAR and Vision Dataset](http://cirs.udg.edu/caves-dataset/)
- [ACFR Marine Robotics Dataset](http://marine.acfr.usyd.edu.au/datasets/)

## Contributing
Please Feel free to send a [pull request](https://github.com/youngguncho/awesome-slam-datasets/pulls) to modify the list or add datasets.


## License
[![CC0](http://mirrors.creativecommons.org/presskit/buttons/88x31/svg/cc-zero.svg)](https://creativecommons.org/publicdomain/zero/1.0/)

To the extent possible under law, [Younggun Cho](https://github.com/youngguncho) has waived all copyright and related or neighboring rights to this work.

Authorized Translation by [Yvon-Shong](https://github.com/Yvon-Shong)