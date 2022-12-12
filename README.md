# g2opy

This is a python binding of graph optimization C++ framework [g2o](https://github.com/RainerKuemmerle/g2o) forked from https://github.com/uoip/g2opy 

## License
* For g2o's original C++ code, see [License](#g2oLicense).
* The binding code and python example code of this project is licensed under BSD License.  


## Requirements
* [C++ requirements](#g2oRequirements).   
([pybind11](https://github.com/pybind/pybind11) is also required, but it's built in this repository, you don't need to install) 


## Installation
```
git clone https://github.com/uoip/g2opy.git
cd g2opy
mkdir build
cd build
cmake ..
make -j8
cd ..
python setup.py install
```
Tested under Ubuntu 16.04, Python 3.6+.


## Get Started
The code snippets below show the core parts of BA and Pose Graph Optimization in a SLAM system.
#### Bundle Adjustment
```python
import numpy
import g2o

class BundleAdjustment(g2o.SparseOptimizer):
    def __init__(self, ):
        super().__init__()
        solver = g2o.BlockSolverSE3(g2o.LinearSolverCSparseSE3())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super().set_algorithm(solver)

    def optimize(self, max_iterations=10):
        super().initialize_optimization()
        super().optimize(max_iterations)

    def add_pose(self, pose_id, pose, cam, fixed=False):
        sbacam = g2o.SBACam(pose.orientation(), pose.position())
        sbacam.set_cam(cam.fx, cam.fy, cam.cx, cam.cy, cam.baseline)

        v_se3 = g2o.VertexCam()
        v_se3.set_id(pose_id * 2)   # internal id
        v_se3.set_estimate(sbacam)
        v_se3.set_fixed(fixed)
        super().add_vertex(v_se3) 

    def add_point(self, point_id, point, fixed=False, marginalized=True):
        v_p = g2o.VertexSBAPointXYZ()
        v_p.set_id(point_id * 2 + 1)
        v_p.set_estimate(point)
        v_p.set_marginalized(marginalized)
        v_p.set_fixed(fixed)
        super().add_vertex(v_p)

    def add_edge(self, point_id, pose_id, 
            measurement,
            information=np.identity(2),
            robust_kernel=g2o.RobustKernelHuber(np.sqrt(5.991))):   # 95% CI

        edge = g2o.EdgeProjectP2MC()
        edge.set_vertex(0, self.vertex(point_id * 2 + 1))
        edge.set_vertex(1, self.vertex(pose_id * 2))
        edge.set_measurement(measurement)   # projection
        edge.set_information(information)

        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super().add_edge(edge)

    def get_pose(self, pose_id):
        return self.vertex(pose_id * 2).estimate()

    def get_point(self, point_id):
        return self.vertex(point_id * 2 + 1).estimate()
```

#### Pose Graph Optimization
```python
import numpy
import g2o

class PoseGraphOptimization(g2o.SparseOptimizer):
    def __init__(self):
        super().__init__()
        solver = g2o.BlockSolverSE3(g2o.LinearSolverCholmodSE3())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super().set_algorithm(solver)

    def optimize(self, max_iterations=20):
        super().initialize_optimization()
        super().optimize(max_iterations)

    def add_vertex(self, id, pose, fixed=False):
        v_se3 = g2o.VertexSE3()
        v_se3.set_id(id)
        v_se3.set_estimate(pose)
        v_se3.set_fixed(fixed)
        super().add_vertex(v_se3)

    def add_edge(self, vertices, measurement, 
            information=np.identity(6),
            robust_kernel=None):

        edge = g2o.EdgeSE3()
        for i, v in enumerate(vertices):
            if isinstance(v, int):
                v = self.vertex(v)
            edge.set_vertex(i, v)

        edge.set_measurement(measurement)  # relative pose
        edge.set_information(information)
        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super().add_edge(edge)

    def get_pose(self, id):
        return self.vertex(id).estimate()
```
For more details, checkout [python examples](python/examples) or project [stereo_ptam](https://github.com/uoip/stereo_ptam).  
Thanks to [pybind11](https://github.com/pybind/pybind11), g2opy works seamlessly between numpy and underlying Eigen.  

### <span id="g2oLicense">G2O License</span>
g2o is licensed under the BSD License. However, some libraries are available
under different license terms. See below.

The following parts are licensed under LGPL3+:
- csparse\_extension

The following parts are licensed under GPL3+:
- g2o_viewer
- g2o_incremental
- slam2d_g2o (example for 2D SLAM with a QGLviewer GUI)

Please note that some features of CHOLMOD are licensed under the GPL. 
To avoid that your binary has to be licensed under the GPL, you may have to 
re-compile CHOLMOD without including its GPL features. The CHOLMOD library d
istributed with, for example, Ubuntu or Debian includes the GPL features. 
The supernodal factorization is considered by g2o, if it is available.


