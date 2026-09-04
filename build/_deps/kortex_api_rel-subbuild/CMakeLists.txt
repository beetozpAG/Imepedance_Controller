# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.28.3)

# Reject any attempt to use a toolchain file. We must not use one because
# we could be downloading it here. If the CMAKE_TOOLCHAIN_FILE environment
# variable is set, the cache variable will have been initialized from it.
unset(CMAKE_TOOLCHAIN_FILE CACHE)
unset(ENV{CMAKE_TOOLCHAIN_FILE})

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(kortex_api_rel-populate NONE)



include(ExternalProject)
ExternalProject_Add(kortex_api_rel-populate
                     "UPDATE_DISCONNECTED" "False" "DOWNLOAD_EXTRACT_TIMESTAMP" "TRUE" "EXTERNALPROJECT_INTERNAL_ARGUMENT_SEPARATOR" "URL" "https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.7.0/linux_x86-64_gcc_5.4.zip" "URL_HASH" "SHA256=e522fde1424103050c3512f7989c2e7be74353fc754a17d162bb33ad365fb7ae"
                    SOURCE_DIR          "/home/beetozp/Escritorio/MODULAR/Controlador_Impedancia/build/_deps/kortex_api_rel-src"
                    BINARY_DIR          "/home/beetozp/Escritorio/MODULAR/Controlador_Impedancia/build/_deps/kortex_api_rel-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
                    USES_TERMINAL_PATCH     YES
)


