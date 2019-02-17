# Paper Review: Large Pose 3D Face Reconstruction from a Single Image via Direct Volumetric Regression

*Aaron S. Jackson, Adrian Bulat, Vasileios Argyriou and Georgios Tzimiropoulos*

*Try out the code without running it!* Check out the autors online demo [here](http://www.cs.nott.ac.uk/~psxasj/3dme/).

http://aaronsplace.co.uk/papers/jackson2017recon/preview.png

[![VIDEO DEMO](https://img.youtube.com/vi/YOUTUBE_VIDEO_ID_HERE/0.jpg)](https://www.youtube.com/watch?v=XWRRvUBN1wk)

Please visit the autor [project webpage](http://aaronsplace.co.uk/papers/jackson2017recon/) for a link to the paper and any information related to it.

# Our Approach testing

This is an unguided version of the Volumetric Regression Network (VRN)
for 3D face reconstruction from a single image. This method approaches
the problem of reconstruction as a segmentation problem, producing a
3D volume, spatially aligned with the input image. A mesh can then be
obtained by taking the isosurface of this volume.

![alt text](https://github.com/roxanasoto/P1_CalibracionCamara/blob/master/Trabajo%202/codigo/Avance1/vrn_torch/Selection_021.png)


Several example images are included in the examples folder. The good example would be get on this site
[3DDFA](http://www.cbsr.ia.ac.cn/users/xiangyuzhu/projects/3DDFA/main.htm), but we tested with the more real images took with a mobile phone photos.

Unfortunately, we don't get errors as the original version says it would calculate in matlab, so we just review in the model and test it for several images with unguided model, if you want to see errors and any other things related to it you may go to the original repo of the autor.

## Dependencies (Software Requirements)

* pytorch (>=0.2 recommended)
* Python 3.5+ or Python 2.7 (use preferible 3.3  with anaconda)
* Linux Ubuntu 18.4 (we dont test in other OS)

## Getting Started

### model pretrained 
the pre trained model rvn_unguided is in the /model folder. and the source may is difficutl to get your own
### Running with Python 
this method is for running individually and get the output in just one compilation , the test data is located in /data folder and the also has /crops and /landmark folders for the correpondinng image finally the result is saved as mesh 3D voxel in /output folder.
first of all clone this repo , then go to the main folder "vr_torch".
```
cd /vr_torch

python demo.py

```
### jupyter notebook 
if you feel more confortable with the jupyter notebook or anaconda you can use the notebook version for testing porpuse we generate this and setup for images locatedin /example folder.to run that just type.

```

jupyter notebook demo.ipynb 

```
## alternative source 
* [2D and 3D Face alignment library build using pytorch](https://github.com/1adrianb/face-alignment)
* [Large Pose 3D Face Reconstruction from a Single Image via Direct Volumetric Regression ](https://github.com/AaronJackson/vrn)
