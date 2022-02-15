# Image Lens Reprojection Tool
![GitHub Workflow Status](https://img.shields.io/github/workflow/status/IDLabMedia/image-lens-reproject/CMake)
[![GitHub license](https://img.shields.io/github/license/IDLabMedia/image-lens-reproject)](https://github.com/IDLabMedia/image-lens-reproject/blob/main/LICENSE)
[![GitHub stars](https://img.shields.io/github/stars/IDLabMedia/image-lens-reproject)](https://github.com/IDLabMedia/image-lens-reproject/stargazers)

## What is lens-reproject?

lens-reproject is a tool to reproject images taken with a known lens to a new
lens. The process is unprojects every pixel coordinate back to the light ray in
spherical coordinates -- with the origin set to the center of projection --
after which it projects the light ray back onto the sensor using the new lens
paramaters. A variety of lenses is supported:

 - Rectilinear (The default perspective projection lenses).
 - Equisolid (Fisheye)
 - Equidistant (Fisheye)

## Build
You will need a C++ compiler and CMake. All other dependencies are included as
submodules.

```sh
git clone https://github.com/IDLabMedia/image-lens-reproject.git
cd image-lens-reproject
git submodule update --init --recursive
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
```

## CLI interface

```
Usage:
  ./reproject [OPTION...]

 Input/output options:
      --input-cfg json-file   Input JSON file containing lens and camera 
                              settings of the input images.
      --output-cfg json-file  Output JSON file containing lens and camera 
                              settings of the input images.
  -i, --input-dir file        Input directory containing images to 
                              reproject.
  -o, --output-dir file       Output directory to put the reprojected 
                              images.

 Output optics options:
      --rectilinear focal_length,sensor_width
                                Output rectilinear images with given 
                                focal_length,sensor_width tuple.
      --equisolid focal_length,sensor_width,fov
                                Output equisolid images with given 
                                focal_length,sensor_width,fov tuple.
      --equidistant fov         Output equidistant images with given fov 
                                value.

 Runtime options:
  -j, --parallel threads  Number of parallal images to process. (default: 
                          1)
      --dry-run           Do not actually reproject images. Only produce 
                          config.
  -h, --help              Show help

 Sampling options:
  -s, --samples number    Number of samples per dimension for interpolating 
                          (default: 1)
      --nn                Nearest neighbor interpolation
      --bl                Bilinear interpolation
      --bc                Bicubic interpolation (default)
      --scale percentage  Output scale, as a percentage of the input size. 
                          It is recommended to increase --samples to 
                          prevent aliassing in case you are downscaling. 
                          Eg: --scale 50 --samples 2 or --scale 33.334 
                          --samples 3 or --scale 25 --samples 4. Final 
                          dimensions are rounded towards zero. (default: 
                          100)
```

### Configuration JSON
The configuration JSON files required in the input of the CLI interface are
flexible. The application will read in the JSON, extract the lens information
from the `"camera"`, `"resolution"`, and `"sensor_size"` keys in the JSON root
object, rewrite the values of those keys  to match the new settings, and
finally write out the updated JSON to a new file. This way, all custom
information is left untouched and it is more easy to integrate it in other
pipelines and systems.

The `"camera"` object in the root object follows the Blender camera settings
structure. This is not always the clearest, but we chose it as we work a lot
with Blender and it was straightforward to export the camera and lens
information straight from Blender. Next we show the different lens
configuration templates for the JSON file:

### `rectilinear`
```json
{
  "camera": {
    "focal_length": 36.0,
    "lens_unit": "MILLIMETERS",
    "projection_matrix": [
      [ 2.0, 0.0, 0.0, 0.0 ],
      [ 0.0, 2.0, 0.0, 0.0 ],
      [ 0.0, 0.0, 0.0, 0.0 ],
      [ 0.0, 0.0, 0.0, 1.0 ]
    ],
    "type": "PERSP"
  },
  "resolution": [ 2048, 2048 ],
  "sensor_size": [ 36.0, 36.0 ]
}
```
(Note that more elements may be present in the root object, as the tool just
ignores them, but will copy them over to the output JSON.)

### `equidistant`
```json
{
  "camera": {
    "type": "PANO",
    "panorama_type": "FISHEYE_EQUIDISTANT",
    "fisheye_fov": 3.1415927410125732
  },
  "resolution": [ 2048, 2048 ],
  "sensor_size": [ 36.0, 36.0 ]
}
```
Note: This lens is not expressed by a focal length. Instead the lens perfectly
fits a circle of projection on the sensor. The equidistant lens is only
specified by a field of view (fov).

### `equisolid`
```json
{
  "camera": {
    "type": "PANO",
    "panorama_type": "FISHEYE_EQUISOLID",
    "lens": 12.5,
    "fisheye_fov": 3.1415927410125732
  },
  "resolution": [ 2048, 2048 ],
  "sensor_size": [ 36.0, 36.0 ]
}
```
Note: `lens` is the focal length of the lens, expressed in the same unit as the
`sensor_size` element (typically millimeters). The naming is taken from Blender.


## License

See the [`LICENSE`](https://github.com/IDLabMedia/image-lens-reproject/blob/main/LICENSE) file.
