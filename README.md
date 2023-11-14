# Drost PPF

PPF (point pair feature) is a point registration method proposed by Drost. et al. in 2010 
>[Model Globally, Match Locally: Efficient and Robust 3D Object Recognition](https://www.researchgate.net/profile/Slobodan-Ilic-3/publication/221363843_Model_Globally_Match_Locally_Efficient_and_Robust_3D_Object_Recognition/links/09e41509a3767e96e2000000/Model-Globally-Match-Locally-Efficient-and-Robust-3D-Object-Recognition.pdf)

Thanks for the great effort of pcl(point cloud lib), the method has been implemented. You can find the usage of PPF in their document: [here](https://pointclouds.org/documentation/classpcl_1_1_p_p_f_estimation.html)
>3D is here: Point Cloud Library (PCL)

Although the algorithm in the PCL works well and has been well managed, I tried to reproduce the method and figure out the some details about the article.
That is the reason for establishing this repo. :smiley: 

## Dependence 
+ Cmake 3.5
+ PCL 1.8
+ Boost (Although boost has been in integrated in C++ standard(std::), unfortunately, I still use many func in the boost)
+ OpenMP 
+ Eigen3

## Usage

### Compile the code
```bash
    mkdir build && cd build && cmake .. && make
```

### Run the code
```bash
    ./Drost-PPF /path_to_src_point_cloud /path_to_tgt_point_cloud
```
+ The **.pcd point cloud file** is needed
+ arg1 is the model and arg2 is scene point cloud

### Warning
+ This is not a complete project and I just finished the key code and check the correctness.
+ If you wanna use Drost PPF to establish some test **for research**, **PCL lib is recommended to use.**
+ I did not write the Config file , so some param need to be adjusted from the code and don't forget recompile.
  + Point Cloud downsample leaf size : ./Drost_PPF.cpp -> line 123. 
  + OpenMP process number can be adjusted in the file ./PPFRegistration.cpp -> line 132.
  + Many params are set in the file ./Drost_PPF.cpp's "test" function.


