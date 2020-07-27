@echo off
echo ************************************************************************
echo [INFO]: dbcModel.layoutGenerator starts
echo [version]: 1.0
echo ************************************************************************

@echo [HINT] must change the name of dbc to ESCL.dbc
set dbc=..\dbc\ESCL.dbc

@echo [HINT] must change the name to ESCL/ESL...
set ESCL_NAME=ESCL

@echo [HINT] must change the msg list to Coco
set RxMsgList=ESC8_20,ESC9_200,PKE1_100

set I2CCAN_par_h=..\output\I2CCAN_Par.h
set I2CCAN_par_c=..\output\I2CCAN_Par.c

if not exist %dbc% goto ErrorWrongPath
if not exist %I2CCAN_par_h% cd.> %I2CCAN_par_h%
if not exist %I2CCAN_par_c% cd.> %I2CCAN_par_c%

python MdlayoutGenerator.py %dbc% %I2CCAN_par_h% %I2CCAN_par_c% %ESCL_NAME% %RxMsgList%

goto End

:ErrorWrongPath
echo ************************************************************************
echo [ERROR]: dbc file does not exist
echo ************************************************************************

:End
echo ************************************************************************
echo [INFO]: dbcModel.layoutGenerator ends
echo ************************************************************************
pause