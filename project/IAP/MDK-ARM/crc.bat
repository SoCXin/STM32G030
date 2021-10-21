SET SREC_PATH=C:\SREC

for /f %%i in ('dir /b .\gps_bass.bin') do (
set indexdx=%%~zi
)

ECHO %indexdx%

ECHO %SREC_PATH%\srec_cat.exe gps_bass.bin -Binary -crop 0 %indexdx%  -crc32-b-e %indexdx%  -o gps_bass_crc.bin -Binary
%SREC_PATH%\srec_cat.exe gps_bass.bin -Binary -crop 0 %indexdx%  -crc32-l-e %indexdx%  -o gps_bass_crc.bin -Binary
