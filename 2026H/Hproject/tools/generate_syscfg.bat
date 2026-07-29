@echo off
setlocal

set "SYSCONFIG_CLI=E:\ti\sysconfig-1.20.0_3587\sysconfig_cli.bat"
set "SDK_PRODUCT=E:\ti\mspm0_sdk_2_01_00_03\.metadata\product.json"
set "PROJECT_ROOT=%~dp0.."
set "CONFIG_FILE=%PROJECT_ROOT%\config\board.syscfg"
set "GENERATED_DIR=%PROJECT_ROOT%\generated"

if not exist "%SYSCONFIG_CLI%" (
    echo SysConfig CLI not found: %SYSCONFIG_CLI%
    exit /b 1
)
if not exist "%CONFIG_FILE%" (
    echo SysConfig file not found: %CONFIG_FILE%
    exit /b 1
)
if not exist "%GENERATED_DIR%" mkdir "%GENERATED_DIR%"

call "%SYSCONFIG_CLI%" -o "%GENERATED_DIR%" -s "%SDK_PRODUCT%" --compiler keil "%CONFIG_FILE%"
exit /b %ERRORLEVEL%
