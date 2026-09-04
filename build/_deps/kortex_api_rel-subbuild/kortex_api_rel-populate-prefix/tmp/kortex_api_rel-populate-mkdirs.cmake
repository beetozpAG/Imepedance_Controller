# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/beetozp/Escritorio/MODULAR/Controlador_Impedancia/build/_deps/kortex_api_rel-src"
  "/home/beetozp/Escritorio/MODULAR/Controlador_Impedancia/build/_deps/kortex_api_rel-build"
  "/home/beetozp/Escritorio/MODULAR/Controlador_Impedancia/build/_deps/kortex_api_rel-subbuild/kortex_api_rel-populate-prefix"
  "/home/beetozp/Escritorio/MODULAR/Controlador_Impedancia/build/_deps/kortex_api_rel-subbuild/kortex_api_rel-populate-prefix/tmp"
  "/home/beetozp/Escritorio/MODULAR/Controlador_Impedancia/build/_deps/kortex_api_rel-subbuild/kortex_api_rel-populate-prefix/src/kortex_api_rel-populate-stamp"
  "/home/beetozp/Escritorio/MODULAR/Controlador_Impedancia/build/_deps/kortex_api_rel-subbuild/kortex_api_rel-populate-prefix/src"
  "/home/beetozp/Escritorio/MODULAR/Controlador_Impedancia/build/_deps/kortex_api_rel-subbuild/kortex_api_rel-populate-prefix/src/kortex_api_rel-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/beetozp/Escritorio/MODULAR/Controlador_Impedancia/build/_deps/kortex_api_rel-subbuild/kortex_api_rel-populate-prefix/src/kortex_api_rel-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/beetozp/Escritorio/MODULAR/Controlador_Impedancia/build/_deps/kortex_api_rel-subbuild/kortex_api_rel-populate-prefix/src/kortex_api_rel-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
