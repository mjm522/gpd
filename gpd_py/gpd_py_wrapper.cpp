#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
namespace py = pybind11;
#include "gpd_py_lib.cpp"


PYBIND11_MODULE(gpd_py, m) {
    m.doc() = "pybind11 gpd plugin"; // optional module docstring

    // bindings to GPDPy class
    py::class_<GPDPy>(m, "GPDPy")
        .def(py::init<const std::string>())
        .def("detect_grasps", &GPDPy::detectGrasps, "A function which generates grasps");
}
