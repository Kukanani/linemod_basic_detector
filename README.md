# LINEMOD detector in ROS2

## Tips

 - When using virtual training, ensure that the paths to textures are specified
   as absolute.
 - The detector must be run with the same set of modalities as were originally
   used to train. The system cannot currently select a subset of training
   modalities for testing purposes, you must use them all (this should
   theoretically be possible, it's just not implemented).