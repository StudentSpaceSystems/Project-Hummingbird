import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.console
import numpy as np
from pyqtgraph.dockarea import *

# Instantiate window and dockable region
app = QtGui.QApplication([])
win = QtGui.QMainWindow()
pg.setConfigOption('background', (240, 240, 240))
area = DockArea()
win.setCentralWidget(area)
window_size = (1366, 768)
window_block = (int(window_size[0] / 32), int(window_size[1]) / 40)
win.resize(window_size[0], window_size[1])
win.setWindowTitle('Real-time Data Visualization and Control')

# Create docks and place about screen.  size parameter is not constraining and docks re-shape to fill space and
# conform to internal specifications
d1 = Dock("Altitude Orientation Indicator", size=(window_block[0] * 24, window_block[1] * 13), hideTitle=True)
d2 = Dock("Console", size=(window_block[0] * 24, window_block[1] * 5), hideTitle=True)
d3 = Dock("Data Stream", size=(24 * window_block[0], 24 * window_block[1]), hideTitle=True)
d4 = Dock("Menu", size=(8 * window_block[0], 27 * window_block[1]), hideTitle=True)
d6 = Dock("Stream Options", size=(8 * window_block[0], 13 * window_block[1]), hideTitle=True)
area.addDock(d1, 'left')
area.addDock(d6, 'right', d1)
area.addDock(d3, 'bottom', d1)
area.addDock(d2, 'bottom', d3)
area.addDock(d4, 'bottom', d6)


# Add widgets into each dock

# first dock gets save/restore buttons
w4 = pg.LayoutWidget()
label = QtGui.QLabel("""Will contain either 3D plot or 3 cross-sectional plots for orientation viewing""")
saveBtn = QtGui.QPushButton('Save dock state')
restoreBtn = QtGui.QPushButton('Restore dock state')
restoreBtn.setEnabled(False)
w4.addWidget(label, row=0, col=0)
w4.addWidget(saveBtn, row=1, col=0)
w4.addWidget(restoreBtn, row=2, col=0)
d1.addWidget(w4)
# Comments added
state = None
estate = True


def save(*args):
    global state
    print(args)
    state = area.saveState()
    restoreBtn.setEnabled(True)


def load():
    global state
    area.restoreState(state)


saveBtn.clicked.connect(save, 1, 2)
restoreBtn.clicked.connect(load)

w2 = pg.console.ConsoleWidget(namespace={"np":np})
d2.addWidget(w2)

## Hide title bar on dock 3
d3.hideTitleBar()
w3 = pg.PlotWidget(title="Live Raw Data Stream (right click for options)")
w3.plot(np.random.normal(size=100))
d3.addWidget(w3)

# w4 = pg.PlotWidget(title="Dock 4 plot")
# w4.plot(np.random.normal(size=100))
# d4.addWidget(w4)

# w5 = pg.ImageView()
# w5.setImage(np.random.normal(size=(100,100)))
# d5.addWidget(w5)

# w6 = pg.PlotWidget(title="Dock 6 plot")
# w6.plot(np.random.normal(size=100))
# d6.addWidget(w6)

win.show()

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
