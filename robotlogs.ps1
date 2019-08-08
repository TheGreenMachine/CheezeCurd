Write-Host "Copying logs from roboRio"
& scp rio:/home/lvuser/*.bag C:\util\logs\ *> $null
Write-Host "Removing logs from roboRio"
& ssh rio "rm /home/lvuser/*.bag" *> $null
Write-Host "Creating html files"
Get-ChildItem C:\util\logs\*.bag | ForEach-Object  { & C:\util\badlogvis.exe $_ }
Remove-Item C:\util\logs\*.bag *> $null
& $(Get-ChildItem C:\util\logs\*.html | Sort-Object LastWriteTime | Select-Object -last 1)
