# LINEMOD detector in ROS2

This detector is a port of the `linemod` detector in ORK. It also has some
components taken from the OpenCV linemod example detector.

Because of the sources used to create this package, it has some code under
Apache 2.0 and some code under BSD.

## Installation

This package requires OpenCV's linemod implementation, so you will need
a version of OpenCV built from source that includes the `rgbd` module from
`opencv_contrib`. For instructions on building, please see
https://github.com/opencv/opencv_contrib.

## Training

Training can be done with either a Kinect sensor or virtually using an OpenGL
renderer. The virtual trainer will result in less realistic training data, but
also has the ability to automatically encode position and rotation information,
which can then be retreived when doing recognition. For this reason, the
virtual trainer is the preferred method. Run it with the following command

```
linemod_train_virtual
```

The Kinect version can be run as

```
linemod_train_standalone_kinect
```

## Testing
You can run the detector by using
```
linemod_node
```

Although to save memory and overhead, we highly recommend incorporating the
detector into a single-process pipeline. See the `linemod_pipeline` in
[picky_robot](https://github.com/Kukanani/picky_robot) for an example.

## Tips

 - When using virtual training, ensure that the paths to textures are specified
   as absolute.
 - The detector must be run with the same set of modalities as were originally
   used to train. The system cannot currently select a subset of training
   modalities for testing purposes, you must use them all (this should
   theoretically be possible, it just hasn't been implemented yet).