@echo off
setlocal
if not exist bin mkdir bin

cl /nologo /O2 /LD native\src\k4wcll_stub.c /link /NOENTRY /NODEFAULTLIB /OUT:bin\k4wcll.dll || exit /b 1
cl /nologo /O2 /I native\include native\src\vvtechs_worker.c ws2_32.lib user32.lib /Fe:bin\vvtechs_worker.exe || exit /b 1
echo Built bin\k4wcll.dll and bin\vvtechs_worker.exe
