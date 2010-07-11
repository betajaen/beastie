xcopy "build\%2\*.dll" "%OGRE_HOME%\bin\%3\" /Q /R /Y

IF %3==debug GOTO CopyDebug

:CopyRelease
findstr /i /m "SamplePlugin=%1" "%OGRE_HOME%\bin\%3\samples.cfg" >NUL 2>&1
IF ERRORLEVEL 1 GOTO WriteReleaseSample
GOTO Finished

:WriteReleaseSample
ECHO. >> "%OGRE_HOME%\bin\%3\samples.cfg"
ECHO SamplePlugin=%1 >> "%OGRE_HOME%\bin\%3\samples.cfg"
GOTO Finished



:CopyDebug
findstr /i /m "SamplePlugin=%1Debug" "%OGRE_HOME%\bin\%3\samples_d.cfg" >NUL 2>&1
IF ERRORLEVEL 1 GOTO WriteDebugSample
GOTO Finished

:WriteDebugSample
ECHO. >> "%OGRE_HOME%\bin\%3\samples_d.cfg"
ECHO SamplePlugin=%1Debug >> "%OGRE_HOME%\bin\%3\samples_d.cfg"
GOTO Finished


:Finished
