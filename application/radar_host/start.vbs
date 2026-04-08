Set WshShell = CreateObject("WScript.Shell")
cmd = "powershell.exe -NoProfile -ExecutionPolicy Bypass -File ""D:\STM32Project\Radar\application\radar_host\start.ps1"""
WshShell.Run cmd, 0, False
Set WshShell = Nothing
