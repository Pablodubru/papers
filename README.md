# papers

Executable naming convention:
Name the binaries with as:  NameOfTheJournal-PaperTitleFirstWord 



# dependencies
Install fftw, eigen and qt5 serial port libraries in debian-based with:

``
sudo apt install libfftw3-dev libeigen3-dev libqt5serialport5-dev libplot-dev
``

Add user to dialup group with:

``
sudo usermod -a -G dialout <user>
``

and reboot.
