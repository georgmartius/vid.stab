# VidStab

Vidstab is a video stabilization library which can be plugged-in with Ffmpeg and Transcode.

**Why is it needed**

A video acquired using a hand-held camera or a camera mounted on a vehicle, typically suffers from undesirable shakes and jitters. Activities such as surfing, skiing, riding and walking while shooting videos are especially prone to erratic camera shakes. Vidstab targets these video contents to help create smoother and stable videos.

**Some of the features include:**

 * Fast detection of subsequent transformations e.g. translation and rotations up to a given extent.
 * Low pass filtered smoothing with adjustable horizon.
 * Detection algorithms:
  * Smart and fast multi measurement fields algorithm with contrast selection.
  * Brute force algorithm only for translations.
 * Clipping options: keep blank (black) or keep from previous frames.
 * Optional drawing of measurement fields and detected transformations for visual analysis.
 * Zooming possible to get rid of jiggling borders (automatic mode).
 * Resulting images are interpolated (different algorithms).
 * Sharpening of the stabilized movie to compensate for interpolation effects due to rotation/zooming (only with Transcode).
 * Single pass filter for streaming applications(only with Transcode).
 * Virtual-tripod-mode to get a tripod experience.

**NOTE:** This readme focuses mainly on using vidstab with Ffmpeg. See 
[here](http://public.hronopik.de/vid.stab) for information regarding installation, usage and examples for using vidstab with Transcode. Or contact me at georg dot martius @ web dot de
  
## System Requirements
 * A Linux-based system
 * ffmpeg source code
 * Cmake
  
## Installation Instructions

For using vidstab library with ffmpeg, ffmpeg must to be configured using `--enable-libvidstab ` option.

### Default Build and Installation:
##### Installing vidstab library:
    
```shell    
cd path/to/vid.stab/dir/
cmake .
make
sudo make install
```

##### Installing ffmpeg:   
   
```shell    
cd path/to/ffmpeg/dir/
./configure --enable-gpl --enable-libvidstab <other configure options>
make
sudo make install
```

### Alternatively one can install vidstab into a custom directory this way:
##### Installing vidstab library:

```shell
cd path/to/vid.stab/dir/
cmake -DCMAKE_INSTALL_PREFIX:PATH=path/to/install_dir/
make
sudo make install
```

##### Installing ffmpeg:

```shell
cd path/to/ffmpeg/dir/
PKG_CONFIG_PATH="path/to/install_dir/lib/pkgconfig" \
./configure --enable-gpl --enable-libvidstab <other optionalconfigure options>
make
sudo make install
```      
      
Before running ffmpeg for the first time, make sure to export `LD_LIBRARY_PATH` to point to vidstab library, e.g.,
    
```shell   
export LD_LIBRARY_PATH=path/to/install_dir/lib:$LD_LIBRARY_PATH
```    

## Usage instructions

**Currently with ffmpeg, vidstab library must run in two-pass mode.** The first pass employs the **vidstabdetect** filter and the second pass uses the **vidstabtransform** filter. 

*Single pass filter with vidstab library is only available with Transcode. The 
[deshake](http://www.ffmpeg.org/ffmpeg-filters.html#deshake) filter of ffmpeg can be used for a single-pass encoding, though using the vidstab two-pass filters will give superior results.*

The vidstabdetect filter (in first pass) will generate a file with relative-translation and rotation-transform information about subsequent frames. This information will then be read by vidstabtransform filter (in second pass) to compensate for the jerky motions and produce a stable video output.

Make sure that you use [unsharp](http://www.ffmpeg.org/ffmpeg-filters.html#unsharp-1) filter provided by ffmpeg for best results (only in second pass).

*See [the list of ffmpeg filters](http://www.ffmpeg.org/ffmpeg-filters.html) to know more about vidstabdetect, vidstabtransform and all other filters available with ffmpeg.*

### Available options with vidstab filters:

##### First pass (vidstabdetect filter):

<dl>
  <dt><b>result</b></dt>
  <dd>Set the path to the file used to write the transforms information. Default value is <b>transforms.trf</b>.</dd>
  <dt><b>shakiness</b></dt>
  <dd>Set the shakiness of input video or quickness of camera. It accepts an integer in the range 1-10, a value of 1 means little shakiness, a value of 10 means strong shakiness. Default value is 5.</dd>
  <dt><b>accuracy</b></dt>
  <dd>Set the accuracy of the detection process. It must be a value in the range 1-15. A value of 1 means low accuracy, a value of 15 means high accuracy. Default value is 15.</dd>
  <dt><b>stepsize</b></dt>
  <dd>Set stepsize of the search process. The region around minimum is scanned with 1 pixel resolution. Default value is 6.</dd>
  <dt><b>mincontrast</b></dt>
  <dd>Set minimum contrast. Any measurement field having contrast below this value is discarded. Must be a floating point value in the range 0-1. Default value is 0.3.</dd>
  <dt><b>tripod</b></dt>
  <dd>  Set reference frame number for tripod mode.  If enabled, the motion of the frames is compared to a reference frame in the filtered stream, identified by the specified number. The intention is to compensate all movements in a more-or-less static scene and keep the camera view absolutely still. If set to 0, it is disabled. The frames are counted starting from 1.
  <br>NOTE: If this mode is used in first pass then it should also be used in second pass.</dd>
  <dt><b>show</b></dt>
  <dd>Show fields and transforms in the resulting frames for visual analysis. It accepts an integer in the range 0-2. Default value is 0, which disables any visualization.</dd>
</dl>

  

##### Examples:
  Use default values:
```shell
ffmpeg -i input.mp4 -vf vidstabdetect -f null -
```  
  
  *` -f null - ` makes sure that no output is produced as this is just the first pass. This in-turn results in faster speed.*
  
  Analyzing strongly shaky video and putting the results in file `mytransforms.trf`:
```shell
ffmpeg -i input.mp4 -vf vidstabdetect=shakiness=10:accuracy=15:result="mytransforms.trf" -f null -
```
  
  Visualizing the result of internal transformations in the resulting video:
```shell
ffmpeg -i input.mp4 -vf vidstabdetect=show=1 dummy_output.mp4
```

  Analyzing a video with medium shakiness:
```shell
ffmpeg -i input.mp4 -vf vidstabdetect=shakiness=5:show=1 dummy_output.mp4
```  
  
##### Second pass (vidstabtransform filter): 
<dl>
  <dt><b>input</b></dt>
  <dd>Set path to the file used to read the transforms. Default value is <b>transforms.trf</b>.</dd>
  <dt><b>smoothing</b></dt>
  <dd>Set the number of frames (value*2 + 1), used for lowpass filtering the camera movements. Default value is 10.<br>For example, a number of 10 means that 21 frames are used (10 in the past and 10 in the future) to smoothen the motion in the video. A larger value leads to a smoother video, but limits the acceleration of the camera (pan/tilt movements). 0 is a special case where a static camera is simulated.</dd>
  <dt><b>optalgo</b></dt>
  <dd>Set the camera path optimization algorithm. Accepted values are:
  <br><i><b>gauss:</b></i> Gaussian kernel low-pass filter on camera motion (default).
  <br><i><b>avg:</b></i> Averaging on transformations.</dd>
  <dt><b>maxshift</b></dt>
  <dd>Set maximal number of pixels to translate frames. Default value is -1, meaning: no limit.</dd>
  <dt><b>maxangle</b></dt>
  <dd>Set maximal angle in radians (degree*PI/180) to rotate frames. Default value is -1, meaning: no limit.</dd>
  <dt><b>crop</b></dt>
  <dd>  Specify how to deal with empty frame borders that may be shrinked-in due to movement compensation. Available values are:
  <br><i><b>keep</b></i>: Keep image information from previous frame (default).
  <br><i><b>black</b></i>: Fill the border-areas black.</dd>
  <dt><b>invert</b></dt>
  <dd>Invert transforms if set to 1. Default value is 0.</dd>
  <dt><b>relative</b></dt>
  <dd>Consider transforms as relative to previous frame if set to 1, absolute if set to 0. Default value is 0.</dd>
  <dt><b>zoom</b></dt>
  <dd>Set percentage to zoom. A positive value will result in a zoom-in effect, a negative value in a zoom-out effect. Default value is 0 (no zoom).</dd>
  <dt><b>optzoom</b></dt>
  <dd>Set optimal zooming to avoid blank-borders. Accepted values are:
  <br><i><b>0</b></i>: Disabled.
  <br><i><b>1</b></i>: Optimal static zoom value is determined (only very strong movements will lead to visible borders) (default).
  <br><i><b>2</b></i>: Optimal adaptive zoom value is determined (no borders will be visible), see <b>zoomspeed</b>.
  <br>Note that the value given at zoom is added to the one calculated here.</dd>
  <dt><b>zoomspeed</b></dt>
  <dd>Set percent to zoom maximally each frame (enabled when optzoom is set to 2). Range is from 0 to 5, default value is 0.25.</dd>
  <dt><b>interpol</b></dt>
  <dd>Specify type of interpolation. Available values are:
  <br><i><b>no</b></i>: No interpolation.
  <br><i><b>linear</b></i>: Linear only horizontal.
  <br><i><b>bilinear</b></i>: Linear in both directions (default).
  <br><i><b>bicubic</b></i>: Cubic in both directions (slow speed).
  <dt><b>tripod</b></dt>
  <dd>Enables virtual tripod mode if set to 1, which is equivalent to <b>relative=0:smoothing=0</b>. Default value is 0.
  <br>NOTE: If this mode has been used in first pass then only it should be used in second pass.</dd>
  <dt><b>debug</b></dt>
  <dd>Increase log verbosity if set to 1. Also the detected global motions are written to the temporary file  <b>global_motions.trf</b> . Default value is 0. </dd>
 
</dl>  
  
##### Examples:
  Using default values:
```shell  
ffmpeg -i input.mp4 -vf vidstabtransform,unsharp=5:5:0.8:3:3:0.4 out_stabilized.mp4
```
Note the use of the ffmpeg's unsharp filter which is always recommended.


Zooming-in a bit more and load transform data from a given file:
```shell
ffmpeg -i input.mp4 -vf vidstabtransform=zoom=5:input="mytransforms.trf" out_stabilized.mp4
```

Smoothening the video even more:
```shell
ffmpeg -i input.mp4 -vf vidstabtransform=smoothing=30:input="mytransforms.trf" out_stabilized.mp4
```
## Developement/Contributing

Vidstab is an open source library - pull requests are very welcome. Some things you might like to help us out with:

 * Specific video clips where vidstab is not up-to the mark.
 * Bugs/fixes.
 * New features and improvements.
 * Documentation.
  
  
## License

See [LICENSE](./LICENSE).
