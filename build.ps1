# PowerShell script to clean, configure, and build a CMake project using MinGW

# Remove the existing build directory if it exists
if (Test-Path "build") {
    Remove-Item -Recurse -Force "build"
    Write-Host "Old build directory removed."
}

# Create a new build directory
New-Item -ItemType Directory -Path "build" | Out-Null
Set-Location "build"

# Run CMake to configure the project with MinGW
Write-Host "Configuring project with CMake..."
cmake -G "MinGW Makefiles" ..

# Check if CMake succeeded
if ($LASTEXITCODE -ne 0) {
    Write-Host "CMake configuration failed!" -ForegroundColor Red
    exit $LASTEXITCODE
}

# Build the project using MinGW Make
Write-Host "Building project with MinGW32-make..."
mingw32-make

# Check if the build succeeded
if ($LASTEXITCODE -ne 0) {
    Write-Host "Build failed!" -ForegroundColor Red
    exit $LASTEXITCODE
}

Set-Location ..

Write-Host "Build completed successfully!" -ForegroundColor Green
