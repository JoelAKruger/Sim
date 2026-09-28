# Project Chrono, just the parts Regolith uses: the core multibody engine and
# Chrono::Vehicle, whose SCM terrain is the deformable soil. The source is a sparse
# checkout: the repository's 1.2 GB of demo data is never fetched.
{ lib
, stdenv
, fetchFromGitHub
, cmake
, ninja
, eigen
}:

stdenv.mkDerivation {
  pname = "chrono";
  version = "10.0.0";

  src = fetchFromGitHub {
    owner = "projectchrono";
    repo = "chrono";
    rev = "9faf13dd8f1128dd75ed233a9627027b0422c3f7"; # tag 10.0.0
    sparseCheckout = [ "cmake" "src" ];
    hash = "sha256-q7+/g3UI1DZDBCUB7bFiF/KXgQRQAlg3ai78PLsayDU=";
  };

  # SCM tests each soil sample's short upward ray against the bounding box of every body it
  # watches, but as if the ray never ended, so every body above the soil (the chassis, the
  # legs) passed and cost a real ray cast. This makes the test stop where the ray does.
  patches = [ ./scm-ray-obb.patch ];

  # The build copies and installs data/ and template_project/, which the sparse
  # checkout leaves out; empty ones will do.
  postPatch = ''
    mkdir -p data template_project
  '';

  nativeBuildInputs = [ cmake ninja ];
  # Chrono's installed headers include Eigen's.
  propagatedBuildInputs = [ eigen ];

  cmakeFlags = [
    "-DBUILD_DEMOS=OFF"
    "-DBUILD_TESTING=OFF"
    "-DBUILD_BENCHMARKING=OFF"
    "-DCH_ENABLE_MODULE_VEHICLE=ON"
    "-DCH_ENABLE_MODULE_VEHICLE_COSIM=OFF"
    "-DCH_ENABLE_MODULE_VEHICLE_MODELS=OFF"
    # SIMD detection tunes the build to this machine's CPU; the package must run anywhere.
    "-DCH_USE_SIMD=OFF"
  ];

  meta = {
    description = "Project Chrono multibody dynamics (core and Chrono::Vehicle)";
    homepage = "https://projectchrono.org";
    license = lib.licenses.bsd3;
    platforms = lib.platforms.linux;
  };
}
