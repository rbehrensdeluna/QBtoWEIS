@echo off
REM ===============================================================
REM  QBtoWEIS Installer for Windows (Batch / CMD version)
REM  -----------------------------------------------------------
REM  Assumes:
REM    - Conda (Miniforge/Miniconda/Anaconda) installed & on PATH
REM    - Git installed & on PATH
REM    - No admin required
REM
REM  What it does:
REM    1) Clone or update QBtoWEIS (smart detection of repo location)
REM    2) Create or update the conda env from environment.yml
REM    3) Install mpi4py
REM    4) Install QBtoWEIS in editable mode (-e)
REM ===============================================================

setlocal ENABLEDELAYEDEXPANSION

REM --- Config ---
set "SCRIPT_DIR=%~dp0"
set "REPO_OWNER=rbehrensdeluna"
set "REPO_NAME=QBtoWEIS"
set "REPO_URL=https://github.com/%REPO_OWNER%/%REPO_NAME%.git"

REM Detect repo root if the script is inside the repo (root or /install)
if exist "%SCRIPT_DIR%\.git" (
  set "REPO_DIR=%SCRIPT_DIR%"
) else if exist "%SCRIPT_DIR%..\.git" (
  set "REPO_DIR=%SCRIPT_DIR%.."
) else (
  REM Fall back: clone next to this script
  set "REPO_DIR=%SCRIPT_DIR%%REPO_NAME%"
)

set "ENV_NAME=qbweis-env"

echo.
echo ===============================================================
echo   QBtoWEIS Installer (CMD)
echo   Target folder: "%REPO_DIR%"
echo   Conda env:     %ENV_NAME%
echo ===============================================================
echo.

REM --- Check conda and git availability ---
where conda >nul 2>nul
if errorlevel 1 (
  echo [ERROR] "conda" not found on PATH. Please install/initialize Miniforge/Conda.
  goto :fail
)

where git >nul 2>nul
if errorlevel 1 (
  echo [ERROR] "git" not found on PATH. Please install Git and add it to PATH.
  goto :fail
)

REM --- Clone or update repository ---
if exist "%REPO_DIR%\.git" (
  echo ==^> Repository found at "%REPO_DIR%". Pulling latest changes...
  pushd "%REPO_DIR%"
  git pull --rebase --autostash
  if errorlevel 1 (
    echo [ERROR] git pull failed.
    popd
    goto :fail
  )
  popd
) else if exist "%REPO_DIR%" (
  echo [ERROR] Folder "%REPO_DIR%" exists but is NOT a git repo. Move/rename it or delete it.
  goto :fail
) else (
  echo ==^> Cloning repository to "%REPO_DIR%" ...
  git clone "%REPO_URL%" "%REPO_DIR%"
  if errorlevel 1 (
    echo [ERROR] git clone failed.
    goto :fail
  )
)

echo.

REM --- Conda: ensure conda-forge channel present (best effort, ignore errors) ---
call conda config --add channels conda-forge >nul 2>nul

REM --- Create or update environment from environment.yml ---
echo ==^> Resolving environment "%ENV_NAME%"...
call conda env list | findstr /C:" %ENV_NAME% " >nul
if errorlevel 1 (
  echo ==^> Creating environment from environment.yml ...
  call conda env create --name %ENV_NAME% -f "%REPO_DIR%\environment.yml"
  if errorlevel 1 (
    echo [ERROR] conda env create failed.
    goto :fail
  )
) else (
  echo ==^> Updating environment from environment.yml ...
  call conda env update --name %ENV_NAME% -f "%REPO_DIR%\environment.yml" --prune
  if errorlevel 1 (
    echo [WARN] conda env update reported an issue; continuing...
  )
)

echo.

REM --- Install Windows-specific dependencies ---
echo ==^> Installing mpi4py ...
call conda install -n %ENV_NAME% -y mpi4py

echo.

REM --- Install QBtoWEIS in editable mode ---
echo ==^> Installing QBtoWEIS (-e) ...
call conda run -n %ENV_NAME% python -m pip install -e "%REPO_DIR%"
if errorlevel 1 (
  echo [ERROR] pip editable install failed.
  goto :fail
)

echo.
echo ===============================================================
echo  QBtoWEIS installation complete.
echo.
echo  Next steps:
echo    1) Activate the environment:
echo         conda activate %ENV_NAME%
echo.
echo    2) Download QBladeCE (v2.0.9+):
echo         https://qblade.org/downloads/
echo.
echo    3) Set your QBlade DLL path in:
echo         QBtoWEIS\weis\inputs\modeling_schema.yaml   at "path2qb_dll"
echo.
echo    4) Try the example:
echo         cd "%REPO_DIR%\qb_examples\00_run_test"
echo         python weis_driver_oc3.py
echo ===============================================================
echo.
goto :eof

:fail
echo.
echo Installation did not complete successfully.
echo Press any key to exit...
pause >nul
exit /b 1
