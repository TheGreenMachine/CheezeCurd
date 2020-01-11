$rio = 'rio'
Write-Host "Copying logs from roboRio"
& scp $rio`:/home/lvuser/*.bag d:\util\logs\
Write-Host "Removing logs from roboRio"
#& ssh $rio "rm /home/lvuser/*.bag" *> $null
Write-Host "Creating html files"
Get-ChildItem d:\util\logs\*.bag | ForEach-Object { & d:\util\badlogvis.exe $_ }
Remove-Item d:\util\logs\*.bag *> $null
& $(Get-ChildItem d:\util\logs\*.html | Sort-Object LastWriteTime | Select-Object -last 1)
