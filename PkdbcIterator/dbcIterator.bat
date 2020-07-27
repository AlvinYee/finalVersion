@echo off
echo ************************************************************************
echo [INFO]: dbcModel.dbcIterator starts
echo [version]: 1.0
echo ************************************************************************

set dbc=..\dbc\test.dbc
set flattenDbc=..\output\flattenDbc.c
if not exist %dbc% goto ErrorWrongPath
if not exist %flattenDbc% cd.> %flattenDbc%


python dbcIterator.py %dbc%  %flattenDbc%

goto End

:ErrorWrongPath
echo ************************************************************************
echo [ERROR]: dbc file does not exist
echo ************************************************************************

:End
echo ************************************************************************
echo [INFO]: dbcModel.dbcIterator ends
echo ************************************************************************
pause