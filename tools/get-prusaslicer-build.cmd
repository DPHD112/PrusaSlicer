@echo off
rem Opens the newest PrusaSlicer test build from github.com/DPHD112/PrusaSlicer,
rem downloading it first if this PC doesn't have it yet.
rem
rem Why this stops the "Windows protected your PC" warning: a web browser tags
rem every file it downloads as coming from the internet, and unzipping in
rem Explorer copies that tag onto each file inside. Windows only runs its
rem SmartScreen check on programs that carry the tag. GitHub CLI doesn't add
rem it, so builds fetched here open straight away.
rem
rem Needs GitHub CLI. Install it once from a terminal:  winget install --id GitHub.cli
rem The first run then asks you to sign in to GitHub in your browser.
rem
rem Each build gets its own folder in %USERPROFILE%\PrusaSlicer-builds, named after
rem the number in the build's link, with a build-info.txt saying which build it is.
rem Old builds stay until you delete their folders.
rem
rem To open one particular build, run this with its link (or just the number):
rem   get-prusaslicer-build.cmd https://github.com/DPHD112/PrusaSlicer/actions/runs/36351239938

setlocal
title PrusaSlicer build
set "REPO=DPHD112/PrusaSlicer"
set "ARTIFACT=PrusaSlicer-organic-windows"
set "APP=slic3r-app-launcher.exe"
set "BUILDS=%USERPROFILE%\PrusaSlicer-builds"
set "DEST="
rem Stops GitHub CLI from pausing to ask questions.
set "GH_PROMPT_DISABLED=1"

where gh >nul 2>&1 || goto no_gh

gh auth token >nul 2>&1 && goto signed_in
echo Sign in to GitHub once: type the code shown below into the GitHub page
echo that opens in your browser, then allow GitHub CLI.
echo.
start "" "https://github.com/login/device"
gh auth login --hostname github.com --web || goto failed
echo.
:signed_in

set "RUN=%~1"
if not defined RUN goto find_newest
rem A build link works too: keep the number that follows /runs/
set "RUN=%RUN:*/runs/=%"
for /f "delims=/" %%i in ("%RUN%") do set "RUN=%%i"
goto have_run

:find_newest
rem The newest artifact with this name comes from the newest finished build.
for /f %%i in ('gh api -X GET "repos/%REPO%/actions/artifacts" -f "name=%ARTIFACT%" -f "per_page=1" -q ".artifacts[0].workflow_run.id"') do set "RUN=%%i"
if "%RUN%"=="null" set "RUN="
if not defined RUN goto no_build

:have_run
set "DEST=%BUILDS%\%RUN%"
rem build-info.txt is written last, so a folder without it is an unfinished download.
if exist "%DEST%\build-info.txt" goto launch
if exist "%DEST%" rmdir /s /q "%DEST%"
mkdir "%DEST%" || goto failed
echo Downloading build %RUN% (about 100 MB) into %DEST%
gh run download %RUN% --repo %REPO% --name %ARTIFACT% --dir "%DEST%" || goto failed
gh run view %RUN% --repo %REPO% --json displayTitle,createdAt,url -q ".displayTitle, .createdAt, .url" > "%DEST%\build-info.tmp" || goto failed
ren "%DEST%\build-info.tmp" build-info.txt || goto failed

:launch
if not exist "%DEST%\%APP%" goto no_app
start "" /d "%DEST%" "%DEST%\%APP%"
exit /b 0

:no_gh
echo GitHub CLI isn't installed. Install it once from a terminal with:
echo     winget install --id GitHub.cli
echo then run this again.
goto failed

:no_build
echo Couldn't find any %ARTIFACT% build on GitHub.
goto failed

:no_app
echo %DEST% has no %APP% in it.
goto failed

:failed
if defined DEST if not exist "%DEST%\build-info.txt" if exist "%DEST%" rmdir /s /q "%DEST%"
echo.
echo Nothing was opened. The message above says what went wrong.
pause
exit /b 1
