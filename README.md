# How to use:

Run it in Matlab R201x version.

The package included 4 test images, each with paired depth image and RGB image.

Compute the 3D point cloud first, use function 

```
>> compute_point_cloud(3)
```

and using this command to produce the rotation and translation reprojection:

```
>> compute_2D_projection(3,0.1*pi,'y',[1,0,0])
```

Note that 3 is the image directory name.

