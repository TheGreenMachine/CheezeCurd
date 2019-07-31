Write-Host "Copying logs from roboRio"
& scp rio:/home/lvuser/*.bag D:\logs *> $null
Write-Host "Removing logs from roboRio"
& ssh rio rm /home/lvuser/*.bag *> $null
Write-Host "Creating html files"
Get-ChildItem D:\logs\*.bag | ForEach-Object  { & D:\util\badlogvis.exe $_ }
rm D:\logs\*.bag *> $nullgci
& $(gci D:\logs\*.html | sort LastWriteTime | select -last 1)
