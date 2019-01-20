device_comm_videostream
==============
This is the Ubitrack device_comm_videostream submodule.

Description
----------
The device_comm_videostream contains components for video streaming over networks.

- NVEncSources/Sinks:

  Hardware accelerated h264 streaming using Nvidia NVEnc via NvPipe. Uses a simple (not yet stable) UDP transport protocol.

- NDISources/Sinks:

  CPU-based video compression/streaming using Newtek NDI SDK. Relatively efficient streaming for color-video streams.

## Add Remotes

    $ conan remote add camposs "https://conan.campar.in.tum.de/api/conan/conan-camposs"
    $ conan remote add ubitrack "https://conan.campar.in.tum.de/api/conan/conan-ubitrack"

## For Users: Use this package

There are a number of options:

 * with_nvenc: Enables NVidia NVEnc Videostreaming using hw-acceleration and a simple UDP transport (unstable). Pulls NvPipe dependency and CUDA
 * with_ndi: Enables Newtek NDI Videostreaming. Requires a location of the sdk (config variable: ndisdk_root)
 * ndisdk_root: root of Newtek NDI SDK root directory
### Project setup

If you handle multiple dependencies in your project is better to add a *conanfile.txt*

    [requires]
    ubitrack_device_comm_videostream/1.3.0@ubitrack/stable

    [options]
    ubitrack_device_comm_videostream:with_nvenc=True
    ubitrack_device_comm_videostream:with_ndi=True
    ubitrack_device_comm_videostream:ndisdk_root=~/path/to/ndisdk

    [generators]
    cmake
    txt

Complete the installation of requirements for your project running:

    $ mkdir build && cd build && conan install ..
    
Note: It is recommended that you run conan install from a build directory and not the root of the project directory.  This is because conan generates *conanbuildinfo* files specific to a single build configuration which by default comes from an autodetected default profile located in ~/.conan/profiles/default .  If you pass different build configuration options to conan install, it will generate different *conanbuildinfo* files.  Thus, they shoudl not be added to the root of the project, nor committed to git. 

## For Packagers: Publish this Package

The example below shows the commands used to publish to campar conan repository. To publish to your own conan respository (for example, after forking this git repository), you will need to change the commands below accordingly. 

## Build and package 

The following command both runs all the steps of the conan file, and publishes the package to the local system cache.  This includes downloading dependencies from "build_requires" and "requires" , and then running the build() method. 

    $ conan create . ubitrack/stable

## Upload

    $ conan upload -r ubitrack ubitrack_device_comm_videostream/1.3.0@ubitrack/stable
