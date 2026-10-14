@echo off
setlocal EnableExtensions
pushd "%~dp0" >nul || exit /b 1

set "BUILD_DIR=build"
set "GENERATOR=MinGW Makefiles"
set "CMAKE_PREFIX_PATH="

where cl >nul 2>&1
if not errorlevel 1 (
    set "GENERATOR=NMake Makefiles"
    goto :configure
)

if exist "C:\msys64\ucrt64\bin\cmake.exe" (
    set "PATH=C:\msys64\ucrt64\bin;%PATH%"
    set "GENERATOR=MinGW Makefiles"
    goto :configure
)

where cmake >nul 2>&1
if errorlevel 1 (
    echo ERROR: CMake not found.
    echo Install Visual Studio C++ Build Tools or MSYS2 UCRT64 CMake/GCC.
    goto :failed
)

:configure
cmake -S . -B "%BUILD_DIR%" -G "%GENERATOR%" -DCMAKE_BUILD_TYPE=Release -DBOIDS_SCREENSAVER_ENABLE_OPENMP=OFF
if errorlevel 1 goto :failed

cmake --build "%BUILD_DIR%" --config Release --parallel
if errorlevel 1 goto :failed

ctest --test-dir "%BUILD_DIR%" -C Release --output-on-failure
if errorlevel 1 goto :failed

if not exist "bin\boids.scr" (
    echo ERROR: Build completed without bin\boids.scr.
    goto :failed
)

echo.
echo Build successful: bin\boids.scr
popd
endlocal
exit /b 0

:failed
echo.
echo ERROR: Build failed.
popd
endlocal
exit /b 1
