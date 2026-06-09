# run_ablation.ps1
# Runs all ablation experiments for Section 3.3.3 using conda train-gpu env.
# Usage: powershell -ExecutionPolicy Bypass -File scripts\isaac_rl\run_ablation.ps1

$ErrorActionPreference = "Stop"

$ScriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = (Resolve-Path (Join-Path $ScriptRoot "..\..")).Path
$IsaacRoot = Join-Path $RepoRoot "RL_training\Isaac_RL"
$env:PYTHONPATH = "$IsaacRoot;$env:PYTHONPATH"
Set-Location $RepoRoot

$CONDA_PYTHON = "D:\Programs\Anaconda3\envs\train-gpu\python.exe"
$TRAIN_SCRIPT = Join-Path $IsaacRoot "pushcube_isaac_v0\train_pushcube_bc.py"
$DATASET      = "data\simulation\isaac_rl\datasets\pushcube_manual_v1_aug100.hdf5"
$EPOCHS       = 300
$SEED         = 42

# Verify python exists
if (-not (Test-Path $CONDA_PYTHON)) {
    Write-Error "Python not found at: $CONDA_PYTHON"
    exit 1
}

# Verify dataset exists
if (-not (Test-Path $DATASET)) {
    Write-Error "Dataset not found at: $DATASET"
    exit 1
}

Write-Host "============================================" -ForegroundColor Cyan
Write-Host " PushCube BC Ablation Study" -ForegroundColor Cyan
Write-Host " Python : $CONDA_PYTHON" -ForegroundColor Cyan
Write-Host " Dataset: $DATASET" -ForegroundColor Cyan
Write-Host " Epochs : $EPOCHS" -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan

# ── helper function ──────────────────────────────────────────────────────────
function Run-Experiment {
    param(
        [string]$Tag,
        [string]$OutputDir,
        [string]$ExtraArgs
    )
    Write-Host ""
    Write-Host ">>> [$Tag] starting ..." -ForegroundColor Yellow
    $cmd = "$CONDA_PYTHON $TRAIN_SCRIPT --dataset $DATASET --output $OutputDir --epochs $EPOCHS --seed $SEED $ExtraArgs"
    Write-Host "    $cmd" -ForegroundColor DarkGray
    $start = Get-Date
    Invoke-Expression $cmd
    if ($LASTEXITCODE -ne 0) {
        Write-Error "[$Tag] FAILED with exit code $LASTEXITCODE"
        exit $LASTEXITCODE
    }
    $elapsed = (Get-Date) - $start
    Write-Host ">>> [$Tag] done in $($elapsed.ToString('mm\:ss'))" -ForegroundColor Green
}

# ── Learning Rate Ablation ───────────────────────────────────────────────────
Write-Host ""
Write-Host "=== Learning Rate Ablation ===" -ForegroundColor Magenta

Run-Experiment `
    -Tag       "lr=1e-2" `
    -OutputDir "data\simulation\isaac_rl\runs\ablation_lr_1e2" `
    -ExtraArgs "--lr 1e-2 --batch-size 256"

Run-Experiment `
    -Tag       "lr=1e-3 (baseline)" `
    -OutputDir "data\simulation\isaac_rl\runs\ablation_lr_1e3" `
    -ExtraArgs "--lr 1e-3 --batch-size 256"

Run-Experiment `
    -Tag       "lr=1e-4" `
    -OutputDir "data\simulation\isaac_rl\runs\ablation_lr_1e4" `
    -ExtraArgs "--lr 1e-4 --batch-size 256"

# ── Batch Size Ablation ──────────────────────────────────────────────────────
Write-Host ""
Write-Host "=== Batch Size Ablation ===" -ForegroundColor Magenta

Run-Experiment `
    -Tag       "batch=64" `
    -OutputDir "data\simulation\isaac_rl\runs\ablation_bs64" `
    -ExtraArgs "--lr 1e-3 --batch-size 64"

Run-Experiment `
    -Tag       "batch=128" `
    -OutputDir "data\simulation\isaac_rl\runs\ablation_bs128" `
    -ExtraArgs "--lr 1e-3 --batch-size 128"

# batch=256 already done above (lr=1e-3 baseline), reuse it

Run-Experiment `
    -Tag       "batch=512" `
    -OutputDir "data\simulation\isaac_rl\runs\ablation_bs512" `
    -ExtraArgs "--lr 1e-3 --batch-size 512"

# ── Summary ──────────────────────────────────────────────────────────────────
Write-Host ""
Write-Host "============================================" -ForegroundColor Cyan
Write-Host " All ablation runs complete!" -ForegroundColor Cyan
Write-Host " Results in: data\simulation\isaac_rl\runs\ablation_*" -ForegroundColor Cyan
Write-Host " Next: python scripts/generate_paper_plots.py" -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan
