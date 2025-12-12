# Build and Serve Jekyll Documentation Locally
# Prerequisites: Ruby and Bundler must be installed

Write-Host "=== Jekyll Documentation Server ===" -ForegroundColor Cyan
Write-Host ""

# Check if we're in the right directory
if (-not (Test-Path "_config.yml")) {
    Write-Host "ERROR: Must run from website_jekyll/ directory" -ForegroundColor Red
    Write-Host "Usage: cd exo_docs_scaffold\website_jekyll; .\serve_jekyll.ps1" -ForegroundColor Yellow
    exit 1
}

# Check if Ruby is installed
Write-Host "Checking Ruby installation..." -ForegroundColor Yellow
try {
    $rubyVersion = & ruby --version 2>&1
    Write-Host "  Found: $rubyVersion" -ForegroundColor Green
} catch {
    Write-Host "  ERROR: Ruby not installed!" -ForegroundColor Red
    Write-Host ""
    Write-Host "Install Ruby from:" -ForegroundColor Cyan
    Write-Host "  https://rubyinstaller.org/ (Windows)" -ForegroundColor White
    Write-Host "  or use: choco install ruby" -ForegroundColor White
    Write-Host ""
    Write-Host "After installing Ruby, restart terminal and run this script again." -ForegroundColor Yellow
    exit 1
}

# Check if Bundler is installed
Write-Host ""
Write-Host "Checking Bundler installation..." -ForegroundColor Yellow
try {
    $bundlerVersion = & bundle --version 2>&1
    Write-Host "  Found: $bundlerVersion" -ForegroundColor Green
} catch {
    Write-Host "  Bundler not installed. Installing..." -ForegroundColor Yellow
    gem install bundler
}

# Install dependencies
Write-Host ""
Write-Host "Installing Jekyll and dependencies..." -ForegroundColor Yellow
Write-Host "(This may take a few minutes on first run)" -ForegroundColor Cyan
Write-Host ""

bundle install

if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "ERROR: Failed to install dependencies" -ForegroundColor Red
    exit 1
}

# Start Jekyll server
Write-Host ""
Write-Host "=== Starting Jekyll Server ===" -ForegroundColor Green
Write-Host ""
Write-Host "  Local URL: http://localhost:4000" -ForegroundColor Cyan
Write-Host "  Server will auto-rebuild on file changes" -ForegroundColor Cyan
Write-Host ""
Write-Host "Press Ctrl+C to stop" -ForegroundColor Yellow
Write-Host ""

# Serve the site with live reload
bundle exec jekyll serve --livereload --open-url

# Alternative without opening browser automatically:
# bundle exec jekyll serve --livereload
