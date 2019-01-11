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
        "with_msgpack": [True, False],
    }
    generators = "cmake"

    requires = (
        "ubitrack_core/%s@ubitrack/stable" % version,
        "ubitrack_vision/%s@ubitrack/stable" % version,
        "ubitrack_dataflow/%s@ubitrack/stable" % version,
       )

    default_options = (
        "ubitrack_core:shared=True",
        "ubitrack_vision:shared=True",
        "ubitrack_dataflow:shared=True",
        "with_nvenc=True",
        "with_msgpack=True",
        )

    # all sources are deployed with the package
    exports_sources = "doc/*", "src/*", "CMakeLists.txt"

    def requirements(self):
        if self.options.with_msgpack:
            self.requires("msgpack/[>=2.1.5]@camposs/stable")
        if self.options.with_nvenc:
            self.requires("nvpipe/[>=0.1]@camposs/testing")

    def imports(self):
        self.copy(pattern="*.dll", dst="bin", src="bin") # From bin to bin
        self.copy(pattern="*.dylib*", dst="lib", src="lib") 
        self.copy(pattern="*.so*", dst="lib", src="lib") 
       
    def build(self):
        cmake = CMake(self)
        cmake.definitions["WITH_MSGPACK"] = self.options.with_msgpack
        cmake.definitions["WITH_NVENC"] = self.options.with_nvenc
        cmake.configure()
        cmake.build()
        cmake.install()

    def package(self):
        pass

    def package_info(self):
        pass

    def package_id(self):
        self.info.requires["ubitrack_vision"].full_package_mode()
