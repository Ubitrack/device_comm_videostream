from conans import ConanFile, CMake, tools


class UbitrackCommVideostreamConan(ConanFile):
    name = "ubitrack_device_comm_videostream"
    version = "1.3.0"

    description = "Ubitrack Device Communication Videostream"
    url = "https://github.com/Ubitrack/device_comm_videostream.git"
    license = "GPL"

    short_paths = True
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "with_nvenc": [True, False],
        "with_ndi": [True, False],
        "with_nvenc_rtsp": [True, False],
        "workspaceBuild" : [True,False],
    }
    generators = "cmake"

    default_options = {
        "ubitrack_core:shared": True,
        "ubitrack_vision:shared":True,
        "ubitrack_dataflow:shared":True,
        "with_nvenc":False,
        "with_nvenc_rtsp":True,
        "with_ndi":False,
        "workspaceBuild" : True,
        }

    # all sources are deployed with the package
    exports_sources = "cmake/*", "doc/*", "src/*", "CMakeLists.txt"

    def configure(self):
        # no CUDA/NVenc on Macos 
        if self.settings.os == "Macos":
            self.options.with_nvenc = False
            self.options.with_nvenc_rtsp = False

    def requirements(self):

        userChannel = "ubitrack/stable"
        if self.options.workspaceBuild:
            userChannel = "user/testing"

        self.requires("ubitrack_core/%s@%s" % (self.version, userChannel))
        self.requires("ubitrack_vision/%s@%s" % (self.version, userChannel))
        self.requires("ubitrack_dataflow/%s@%s" % (self.version, userChannel))

        # if not (self.options.with_nvenc or self.options.with_nvenc_rtsp or self.options.with_ndi):
        #     raise ValueError("No Videostream supplier activated.")
        if self.options.with_nvenc:
            self.requires("nvpipe/[>=0.1]@camposs/testing")
        if self.options.with_nvenc_rtsp:
            self.requires("nvenc_rtsp/0.1@artekmed/testing")
            self.requires("cuda_dev_config/[>=1.0]@camposs/stable")

        if self.options.with_ndi:
            self.requires("newtekndi/[>=3.0]@vendor/stable")

    def imports(self):
        self.copy(pattern="*.dll", dst="bin", src="bin") # From bin to bin
        self.copy(pattern="*.dylib*", dst="lib", src="lib") 
        self.copy(pattern="*.so*", dst="lib", src="lib") 
       
    def build(self):
        if not (self.options.with_nvenc or self.options.with_nvenc_rtsp or self.options.with_ndi):
            self.output.warn("No videostream driver selected!!!")
            return

        cmake = CMake(self)
        if self.options.with_nvenc:
            cmake.definitions["WITH_NVENC"] = "ON"
        if self.options.with_nvenc_rtsp:
            cmake.definitions["WITH_NVENC_RTSP"] = "ON"
        if self.options.with_ndi:
            cmake.definitions["WITH_NDI"] = "ON"
        cmake.configure()
        cmake.build()
        cmake.install()

    def package(self):
        pass

    def package_info(self):
        pass

    def package_id(self):
        self.info.requires["ubitrack_vision"].full_package_mode()
