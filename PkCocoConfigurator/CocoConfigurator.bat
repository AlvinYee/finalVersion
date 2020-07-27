@echo off
echo ************************************************************************
echo [INFO]: CocoNut.CocoConfigurator starts
echo [version]: 1.0
echo ************************************************************************

set MqXml=..\description\Coco.xml
set _HCM_LCD=..\template\_HCM_LCD.c
set HCM_LCD=..\output\HCM_LCD.c
set CocoCfg_h=..\output\CocoCfg.h
set CocoCfg_c=..\output\CocoCfg.c

if not exist %MqXml% goto ErrorWrongPath
if not exist %_HCM_LCD% goto ErrorWrongPath

if not exist %HCM_LCD% cd.> %HCM_LCD%
if not exist %CocoCfg_h% cd.> %CocoCfg_h%
if not exist %CocoCfg_c% cd.> %CocoCfg_c%

python -m cogapp -d -o %HCM_LCD% %_HCM_LCD%

goto End
:ErrorWrongPath
echo ************************************************************************
echo [ERROR]: template file does not exist
echo ************************************************************************

:End
echo ************************************************************************
echo [INFO]: CocoNut.CocoConfigurator ends
echo ************************************************************************
pause