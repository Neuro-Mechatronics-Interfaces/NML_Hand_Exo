# Build Jekyll Documentation (Static HTML)
# This builds the site without running a server

Write-Host "=== Jekyll Documentation Builder ===" -ForegroundColor Cyan
Write-Host ""

# Check if we're in the right directory
if (-not (Test-Path "_config.yml")) {
    Write-Host "ERROR: Must run from website_jekyll/ directory" -ForegroundColor Red
    Write-Host "Usage: cd exo_docs_scaffold\website_jekyll; .\build_jekyll.ps1" -ForegroundColor Yellow
    exit 1
}

# Check Ruby/Bundler
Write-Host "Checking dependencies..." -ForegroundColor Yellow
try {
    $rubyVersion = & ruby --version 2>&1
    $bundlerVersion = & bundle --version 2>&1
    Write-Host "  Ruby: OK" -ForegroundColor Green
    Write-Host "  Bundler: OK" -ForegroundColor Green
} catch {
    Write-Host "  ERROR: Ruby or Bundler not installed!" -ForegroundColor Red
    Write-Host "  Run serve_jekyll.ps1 for installation instructions" -ForegroundColor Yellow
    exit 1
}

# Install dependencies if needed
if (-not (Test-Path "Gemfile.lock")) {
    Write-Host ""
    Write-Host "Installing dependencies..." -ForegroundColor Yellow
    bundle install
}

# Clean previous build
Write-Host ""
Write-Host "Cleaning previous build..." -ForegroundColor Yellow
if (Test-Path "_site") {
    Remove-Item -Path "_site" -Recurse -Force
}

# Build the site
Write-Host ""
Write-Host "Building Jekyll site..." -ForegroundColor Yellow
Write-Host ""

bundle exec jekyll build

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "=== Build Successful! ===" -ForegroundColor Green
    Write-Host ""
    Write-Host "Static site generated at:" -ForegroundColor Cyan
    Write-Host "  $(Resolve-Path '_site')" -ForegroundColor White
    Write-Host ""
    Write-Host "To view locally:" -ForegroundColor Cyan
    Write-Host "  .\serve_jekyll.ps1" -ForegroundColor White
    Write-Host ""
    Write-Host "Or open _site\index.html in a browser" -ForegroundColor Cyan
    
} else {
    Write-Host ""
    Write-Host "=== Build Failed ===" -ForegroundColor Red
    Write-Host "Check the errors above for details." -ForegroundColor Yellow
    exit 1
}
