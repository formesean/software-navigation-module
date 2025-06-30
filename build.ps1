# PowerShell script to configure and build a CMake project using MinGW

# Create build directory if it doesn't exist
if (-Not (Test-Path "build")) {
    New-Item -ItemType Directory -Path "build" | Out-Null
    Write-Host "Created build directory."
}

Set-Location "build"

# Configure only if Makefile doesn't exist (i.e., first time or manually deleted)
if (-Not (Test-Path "Makefile")) {
    Write-Host "Running CMake to configure project..."
    cmake -G "MinGW Makefiles" ..
    if ($LASTEXITCODE -ne 0) {
        Write-Host "CMake configuration failed!" -ForegroundColor Red
        exit $LASTEXITCODE
    }
}
else {
    Write-Host "Makefile found — skipping CMake configuration."
}

# Build using MinGW Make
Write-Host "Building project with MinGW32-make..."
mingw32-make

if ($LASTEXITCODE -ne 0) {
    Write-Host "Build failed!" -ForegroundColor Red
    exit $LASTEXITCODE
}

Set-Location ..

Write-Host "✅ Build completed successfully!" -ForegroundColor Green
