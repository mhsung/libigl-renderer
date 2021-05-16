# Offscreen Renderer

Minhyuk Sung (mhsung@kaist.ac.kr)


### Dependencies

```
sudo apt-get update && \
sudo apt-get install -y \
    xorg-dev \
    freeglut3 \
    freeglut3-dev \
    mesa-common-dev \
    libosmesa6-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libx11-dev \
    libxi-dev \
    libxmu-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libgmp3-dev \
    libmpfr-dev \
    libhdf5-serial-dev && \
sudo apt-get -y autoremove && sudo apt-get -y clean
```


### Compile
```
git clone --recursive https://github.com/mhsung/libigl-renderer.git
cd libigl-renderer
mkdir build && cd build
cmake ..
make -j4
cd ..
```

### Demo

```
mkdir snapshots
cd build

# Rendering a mesh.
# The rendered image will be stored in '../snapshots/161_mesh.png'.
./OSMesaRenderer \
    --mesh=../data/LabeledPSBDataset/161.off \
    --snapshot=../snapshots/161_mesh

# Rendering a point cloud.
./OSMesaRenderer \
    --point_cloud=../data/LabeledPSBDataset/161.pts \
    --snapshot=../snapshots/161_pc

# Rendering a mesh with face labels.
./OSMesaRenderer --mesh=../data/LabeledPSBDataset/161.off \
    --face_labels=../data/LabeledPSBDataset/161_labels.txt \
    --snapshot=../snapshots/161_mesh_fl

# Rendering a point cloud with point scalar values.
./OSMesaRenderer --point_cloud=../data/LabeledPSBDataset/161.off \
    --point_values=../data/LabeledPSBDataset/161_y.txt \
    --snapshot=../snapshots/161_pc_pv

# Changing the view point.
./OSMesaRenderer \
    --mesh=../data/LabeledPSBDataset/161.off \
    --azimuth_deg=-30 \
    --elevation_deg=30 \
    --theta_deg=0  \
    --snapshot=../snapshots/161_mesh_view

cd ..
```
