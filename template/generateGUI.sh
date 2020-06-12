#!/bin/bash
cd resources 
pyrcc5 resources.qrc -o resources_rc.py
mv resources_rc.py ..

cd ../gui
pyuic5 mainwindow.ui > form.py
cd ..
	
	
