# LibIGL Renderer

Minhyuk Sung

mhsung@kaist.ac.kr

### Dependencies


### Build (Ubuntu)
mkdir build && cd build </br>
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo </br>
make -j </br>

### Build (Mac OS X)
mkdir build && cd build </br>
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \ </br>
  -DFREEGLUT_INCLUDE_DIR=/usr/local/Cellar/freeglut/3.0.0/include \ </br>
  -DFREEGLUT_LIBRARY=/usr/local/Cellar/freeglut/3.0.0/lib/libglut.dylib </br>
make -j</br>


### Demo
`./GlutViewer -point_set=../data/pointcloud/07dcf9cafc575f8194e2f8da375e27cfad99f9b1/Parts/B7760557CDDC5C13AEEE8174B9F37076A045E843/shape.ply \
-run_primitive_fitting -azimuth_deg=120 -elevation_deg=30 -theta_deg=0 -snapshot=test`

- point_set: Input point set PLY file.
- run_primitive_fitting: Run RANSAC using PCL.
- azimuth_deg: Azimuth angle in degree for rendering.
- elevation_deg: Elevation angle in degree for rendering.
- theta_deg: Theta angle in degree for rendering.
- snapshot: Output snapshot file (`(shapshot).png` will be saved).
