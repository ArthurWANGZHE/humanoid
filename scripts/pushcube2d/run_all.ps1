# Full PushCube2D experiment runner.
#
# Pipeline:
#   1. Recollect aligned / near / random scripted datasets.
#   2. Train two BC variants for each dataset:
#      - all_data: uses every valid transition, including failed episodes
#      - success_only: uses only successful episodes
#   3. Evaluate every trained model on aligned / near / random reset modes.
#   4. Generate per-run training plots plus aggregate comparison tables/figures.
#
# Examples:
#   .\run_all.ps1
#   .\run_all.ps1 -CollectEpisodes 500 -Epochs 500 -HiddenDim 256 -NumLayers 5
#   .\run_all.ps1 -CollectMaxSteps 160   # clean expert collection
#   .\run_all.ps1 -Only demo_random -CollectEpisodes 500 -Epochs 300
#   .\run_all.ps1 -SkipRecollect -SkipVideos

param(
    [int]$CollectEpisodes = 300,
    [int]$CollectMaxSteps = 25,
    [int]$Epochs = 300,
    [int]$BatchSize = 256,
    [double]$Lr = 1e-3,
    [double]$MinLrRatio = 0.05,
    [ValidateSet("constant", "cosine", "linear")]
    [string]$LrSchedule = "cosine",
    [int]$WarmupEpochs = 5,
    [int]$HiddenDim = 128,
    [int]$NumLayers = 4,
    [ValidateSet("tanh", "relu", "silu")]
    [string]$Activation = "silu",
    [double]$WeightDecay = 1e-5,
    [double]$GradClip = 5.0,
    [double]$ValSplit = 0.1,
    [int]$Seed = 0,
    [int]$EvalSeed = 1000,
    [int]$EvalEpisodes = 100,
    [int]$DatasetEpisode = 0,
    [string[]]$Only = @(),
    [switch]$SkipRecollect,
    [switch]$CollectSuccessOnly,
    [switch]$NoTensorBoard,
    [switch]$SkipTraining,
    [switch]$SkipEvaluation,
    [switch]$SkipPlots,
    [switch]$SkipVideos
)

$ErrorActionPreference = "Stop"

$ScriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = (Resolve-Path (Join-Path $ScriptRoot "..\..")).Path
$PushcubeRoot = Join-Path $RepoRoot "psuhcube"
$env:PYTHONPATH = "$PushcubeRoot;$env:PYTHONPATH"
Set-Location $RepoRoot

$TensorBoard = -not $NoTensorBoard.IsPresent
$ShouldRecollect = -not $SkipRecollect.IsPresent
$MakePlots = -not $SkipPlots.IsPresent
$MakeVideos = -not $SkipVideos.IsPresent
$CollectSuccessOnlyValue = $CollectSuccessOnly.IsPresent

$datasets = @(
    @{
        name = "demo_aligned"
        reset = "aligned"
        dataset = "data/pushcube2d/demo_aligned.npz"
    },
    @{
        name = "demo_near"
        reset = "near"
        dataset = "data/pushcube2d/demo_near.npz"
    },
    @{
        name = "demo_random"
        reset = "random"
        dataset = "data/pushcube2d/demo_random.npz"
    }
)

$trainVariants = @(
    @{
        name = "all_data"
        successOnlyData = $false
        label = "All data"
    },
    @{
        name = "success_only"
        successOnlyData = $true
        label = "Success only"
    }
)

$evalModes = @("aligned", "near", "random")
$logRoot = "data/pushcube2d/logs"
$modelRoot = "data/pushcube2d/models"
$outputRoot = "figure/pushcube2d"
$experimentRoot = Join-Path $outputRoot "experiment_compare"
$evalRoot = Join-Path $experimentRoot "eval_json"

New-Item -ItemType Directory -Force $modelRoot | Out-Null
New-Item -ItemType Directory -Force $logRoot | Out-Null
New-Item -ItemType Directory -Force $outputRoot | Out-Null
New-Item -ItemType Directory -Force $experimentRoot | Out-Null
New-Item -ItemType Directory -Force $evalRoot | Out-Null

if ($CollectSuccessOnlyValue) {
    Write-Host "WARNING: -CollectSuccessOnly makes all_data and success_only training nearly identical." -ForegroundColor DarkYellow
    Write-Host "For the requested comparison, leave -CollectSuccessOnly off." -ForegroundColor DarkYellow
}

function Invoke-Step {
    param(
        [string]$Title,
        [scriptblock]$Command
    )

    Write-Host ""
    Write-Host $Title -ForegroundColor Yellow
    & $Command
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed: $Title"
    }
}

function Should-RunDataset {
    param([string]$Name)
    return ($Only.Count -eq 0 -or $Only -contains $Name)
}

$selectedDatasets = @($datasets | Where-Object { Should-RunDataset $_.name })
if ($selectedDatasets.Count -eq 0) {
    throw "No datasets selected. Valid names: demo_aligned, demo_near, demo_random"
}

Write-Host "============================================" -ForegroundColor Cyan

if (-not $SkipEvaluation.IsPresent) {
    Get-ChildItem -LiteralPath $evalRoot -Filter "*.json" -ErrorAction SilentlyContinue | Remove-Item -Force
}
Write-Host " PushCube2D Full Experiment" -ForegroundColor Cyan
Write-Host " Datasets:        $($selectedDatasets.name -join ', ')" -ForegroundColor Cyan
Write-Host " Train variants:  all_data, success_only" -ForegroundColor Cyan
Write-Host " Eval modes:      $($evalModes -join ', ')" -ForegroundColor Cyan
Write-Host " Recollect:       $ShouldRecollect" -ForegroundColor Cyan
Write-Host " CollectEpisodes: $CollectEpisodes" -ForegroundColor Cyan
Write-Host " CollectMaxSteps: $CollectMaxSteps" -ForegroundColor Cyan
Write-Host " Epochs:          $Epochs" -ForegroundColor Cyan
Write-Host " TensorBoard:     $TensorBoard" -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan

foreach ($d in $selectedDatasets) {
    $name = $d.name
    $dataset = $d.dataset
    $resetMode = $d.reset

    Write-Host ""
    Write-Host "--------------------------------------------" -ForegroundColor Cyan
    Write-Host " Dataset stage: $name ($resetMode)" -ForegroundColor Cyan
    Write-Host "--------------------------------------------" -ForegroundColor Cyan

    if ($ShouldRecollect) {
        Invoke-Step "[Collect] Recollecting $name" {
            python -m pushcube2d.collect_data `
                --episodes $CollectEpisodes `
                --out $dataset `
                --render false `
                --seed $Seed `
                --noise-std 0.02 `
                --max-steps $CollectMaxSteps `
                --reset-mode $resetMode `
                --success-only $CollectSuccessOnlyValue
        }
    } elseif (-not (Test-Path -LiteralPath $dataset)) {
        throw "Dataset not found and -SkipRecollect was used: $dataset"
    }

    Invoke-Step "[Check] Validating $name" {
        python -m pushcube2d.check_dataset $dataset
    }

    if ($MakePlots) {
        Invoke-Step "[Dataset Plot] Visualizing $name" {
            python -m pushcube2d.visualize_dataset `
                $dataset `
                --out "$outputRoot/${name}_dataset_summary.png" `
                --max-episodes 80
        }
    }

    foreach ($variant in $trainVariants) {
        $variantName = $variant.name
        $successOnlyData = $variant.successOnlyData
        $runName = "bc_${resetMode}_${variantName}"
        $model = "$modelRoot/${runName}.npz"
        $runDir = "$logRoot/$runName"
        $plotPrefix = "$outputRoot/$runName"

        Write-Host ""
        Write-Host "============================================" -ForegroundColor Cyan
        Write-Host " Training model: $runName" -ForegroundColor Cyan
        Write-Host " Dataset:        $dataset" -ForegroundColor Cyan
        Write-Host " Success filter: $successOnlyData" -ForegroundColor Cyan
        Write-Host " Model:          $model" -ForegroundColor Cyan
        Write-Host " Run dir:        $runDir" -ForegroundColor Cyan
        Write-Host "============================================" -ForegroundColor Cyan

        if (-not $SkipTraining.IsPresent) {
            if (Test-Path -LiteralPath $runDir) {
                Remove-Item -LiteralPath $runDir -Recurse -Force
            }
            Invoke-Step "[Train] $runName" {
                python -m pushcube2d.train_bc `
                    --dataset $dataset `
                    --out $model `
                    --log-dir $logRoot `
                    --run-name $runName `
                    --epochs $Epochs `
                    --batch-size $BatchSize `
                    --lr $Lr `
                    --min-lr-ratio $MinLrRatio `
                    --lr-schedule $LrSchedule `
                    --warmup-epochs $WarmupEpochs `
                    --hidden-dim $HiddenDim `
                    --num-layers $NumLayers `
                    --activation $Activation `
                    --weight-decay $WeightDecay `
                    --grad-clip $GradClip `
                    --val-split $ValSplit `
                    --seed $Seed `
                    --log-every 10 `
                    --save-best true `
                    --tensorboard $TensorBoard `
                    --success-only-data $successOnlyData
            }
        } else {
            Write-Host "[Train] Skipped: $runName" -ForegroundColor DarkYellow
        }

        if ($MakePlots) {
            Invoke-Step "[Training Plot] $runName" {
                python -m pushcube2d.visualize_training `
                    --log-dir $runDir `
                    --out-prefix $plotPrefix `
                    --formats png,svg `
                    --hparams true
            }
        }

        if (-not $SkipEvaluation.IsPresent) {
            foreach ($evalMode in $evalModes) {
                $evalJson = Join-Path $evalRoot "${runName}_on_${evalMode}.json"
                Invoke-Step "[Eval] $runName on $evalMode" {
                    python -m pushcube2d.eval_policy `
                        --model $model `
                        --episodes $EvalEpisodes `
                        --seed $EvalSeed `
                        --render false `
                        --reset-mode $evalMode `
                        --out $evalJson `
                        --label $runName `
                        --train-reset-mode $resetMode `
                        --train-variant $variantName
                }
            }
        } else {
            Write-Host "[Eval] Skipped: $runName" -ForegroundColor DarkYellow
        }

        if ($MakePlots) {
            Invoke-Step "[Model Trajectory] $runName" {
                python -m pushcube2d.visualize_trajectory `
                    --source model `
                    --model $model `
                    --seed $EvalSeed `
                    --reset-mode $resetMode `
                    --out "$outputRoot/${runName}_model_${resetMode}.png"
            }
        }

        if ($MakeVideos) {
            Invoke-Step "[Model Video] $runName" {
                python -m pushcube2d.visualize_video `
                    --source model `
                    --model $model `
                    --seed $EvalSeed `
                    --reset-mode $resetMode `
                    --out "$outputRoot/${runName}_model_${resetMode}.mp4" `
                    --fps 20 `
                    --render-size 512
            }
        }
    }

    if ($MakePlots) {
        Invoke-Step "[Dataset Trajectory] $name episode $DatasetEpisode" {
            python -m pushcube2d.visualize_trajectory `
                --source dataset `
                --dataset $dataset `
                --episode $DatasetEpisode `
                --out "$outputRoot/${name}_dataset_ep${DatasetEpisode}.png"
        }
    }

    if ($MakeVideos) {
        Invoke-Step "[Dataset Video] $name episode $DatasetEpisode" {
            python -m pushcube2d.visualize_video `
                --source dataset `
                --dataset $dataset `
                --episode $DatasetEpisode `
                --out "$outputRoot/${name}_dataset_ep${DatasetEpisode}.mp4" `
                --fps 20 `
                --render-size 512
        }
    }
}

if (-not $SkipEvaluation.IsPresent) {
    Invoke-Step "[Compare] Aggregating evaluation results" {
        python -m pushcube2d.compare_experiments `
            --eval-dir $evalRoot `
            --out-prefix "$experimentRoot/pushcube2d_bc" `
            --formats png,svg
    }
}

if ($MakePlots) {
    Invoke-Step "[Statistics] Generating statistical comparison charts" {
        python -m pushcube2d.plot_statistics
    }
}

Write-Host ""
Write-Host "All experiments complete." -ForegroundColor Cyan
Write-Host "TensorBoard: tensorboard --logdir $logRoot" -ForegroundColor Cyan
Write-Host "Eval JSON:   $evalRoot" -ForegroundColor Cyan
Write-Host "Comparison:  $experimentRoot" -ForegroundColor Cyan
