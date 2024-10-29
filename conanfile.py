from conan import ConanFile
from conan.tools.cmake import cmake_layout

class InstinctRecipe(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"

    def layout(self):
        cmake_layout(self)

    def requirements(self):
        self.requires("spdlog/1.14.1")
        self.requires("fmt/10.2.1")
        self.requires("boost/1.85.0")
        self.requires("eigen/3.4.0")
        self.requires("catch2/3.6.0")
        self.requires("nlohmann_json/3.11.3")
        self.requires("unordered_dense/4.4.0")
        self.requires("muparser/2.3.4")
        self.requires("libssh/0.10.6")

    def configure(self):
        self.options["boost*"].without_atomic = True
        self.options["boost*"].without_chrono = True
        self.options["boost*"].without_cobalt = True
        self.options["boost*"].without_container = True
        self.options["boost*"].without_context = True
        self.options["boost*"].without_contract = True
        self.options["boost*"].without_coroutine = True
        self.options["boost*"].without_date_time = False
        self.options["boost*"].without_exception = True
        self.options["boost*"].without_fiber = True
        self.options["boost*"].without_filesystem = True
        self.options["boost*"].without_graph = True
        self.options["boost*"].without_graph_parallel = True
        self.options["boost*"].without_iostreams = True
        self.options["boost*"].without_json = True
        self.options["boost*"].without_locale = True
        self.options["boost*"].without_log = True
        self.options["boost*"].without_math = False
        self.options["boost*"].without_mpi = True
        self.options["boost*"].without_nowide = True
        self.options["boost*"].without_program_options = False
        self.options["boost*"].without_python = True
        self.options["boost*"].without_random = True
        self.options["boost*"].without_regex = True
        self.options["boost*"].without_serialization = True
        self.options["boost*"].without_stacktrace = True
        self.options["boost*"].without_system = True
        self.options["boost*"].without_test = True
        self.options["boost*"].without_thread = True
        self.options["boost*"].without_timer = True
        self.options["boost*"].without_type_erasure = True
        self.options["boost*"].without_url = True
        self.options["boost*"].without_wave = True