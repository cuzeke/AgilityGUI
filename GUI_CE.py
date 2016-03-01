"""GUI.py Creates a GUI to display and analyse data"""

# Author:            Andrea Guatemala <andreasg@mit.edu>
# Created:           06/2015
# Last Modified:     2/12/2016 by Chika Eke <ceke@mit.edu>
# Modifications from original: Commented out pymatbrige import & all mlab portions of code since Matlab not on Mac!

import sys
from PyQt4 import QtGui, QtCore, uic # PyQt4 does not run in new OSX 'El Capitan' yet
#from pymatbridge import Matlab #enable matlab code to run
#mlab = Matlab()
#mlab = Matlab('C:/Program` Files/MATLAB/R2015b/bin/matlab')

# Import plotting and maths libraries
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QT as NavigationToolbar
import numpy as np

# Load UI file (Created with QT Designer)
form_class = uic.loadUiType("GUI_CE.ui")[0]


# Window class
class MyWindowClass(QtGui.QMainWindow, form_class):
    def __init__(self, parent=None):
        
        # Setup main window
        QtGui.QMainWindow.__init__(self, parent)
        
        # Load UI from QT Developer
        self.setupUi(self)
            
        # Set StartPage
        self.Pages.setCurrentWidget(self.StartPage)
        
        # Connect buttons to functions
        self.EvaluationModeButton.clicked.connect(self.EvaluationModeButton_clicked)
        self.BackButton1.clicked.connect(self.BackButton1_clicked)
        self.ContinueButton1.clicked.connect(self.ContinueButton1_clicked)
        self.AddButton.clicked.connect(self.AddButton_clicked)
        self.RemoveButton.clicked.connect(self.RemoveButton_clicked)
        self.EditObsButton.clicked.connect(self.EditObsButton_clicked)
        self.DetailButton.clicked.connect(self.DetailButton_clicked)
        self.LoadObsButton.clicked.connect(self.LoadObsButton_clicked)
        #self.NewButton.clicked.connect()
        #self.ResApplyBox.clicked.connect()
        #self.XSpinBox.clicked.connect()
        #self.YSpinBox.clicked.connect()
        #self.ZSpinBox.clicked.connect()
        #self.RevCoordButton.clicked.connect()
        self.PlotButton.clicked.connect(self.PlotButton_clicked)
        self.DoneButton1.clicked.connect(self.DoneButton1_clicked)
        self.DoneButton2.clicked.connect(self.DoneButton1_clicked)
        self.LoadRawButton.clicked.connect(self.LoadButton_clicked)
        self.LoadPreButton.clicked.connect(self.LoadPreButton_clicked)
        self.BackButton2.clicked.connect(self.BackButton2_clicked)
        self.ContinueButton2.clicked.connect(self.ContinueButton2_clicked)
        self.BackButton3.clicked.connect(self.BackButton4_clicked)
        self.BackButton4.clicked.connect(self.BackButton4_clicked)
        self.ContinueButton3.clicked.connect(self.ContinueButton3_clicked)
        self.BackButton5.clicked.connect(self.BackButton5_clicked)
        #self.PlotButton1.clicked.connect()
        #self.PlotButton2.clicked.connect()
        #self.PlotButton3.clicked.connect()
        #self.DVcomboBox.clicked.connect()
        #self.IVcomboBox.clicked.connect()
##        self.SummaryMetricsButton.clicked.connect(self.SummaryMetricsButton_clicked)
##        self.ObstaclesButton.clicked.connect(self.ObstaclesButton_clicked)
##        self.MotionPrimitiveButton.clicked.connect(self.MotionPrimitiveButton_clicked)
        
#Button functions
        #Start Page Buttons
    def EvaluationModeButton_clicked(self):
        self.Pages.setCurrentWidget(self.DefCoursePage)

    def DevelopmentModeButton_clicked(self):
        pass
    
        #Def Course Page Buttons
    def LoadObsButton_clicked(self):
        fileNames = QtGui.QFileDialog.getOpenFileName(self, "Select one or more files to open")
        #Create a list of available files (TEMPORARY)
        self.avfiles_dict = {'Calibration': fig1, "Planned agility run 1": fig2}
        # Add files to available list
        for i in self.avfiles_dict:
            self.AddFilesList(self.listWidget2, i)

    def EditObsButton_clicked(self):
        self.Pages.setCurrentWidget(self.DimCoursePage)

    def DetailButton_clicked(self):
        self.Pages.setCurrentWidget(self.ViewObsDetPage)
            
    def ContinueButton1_clicked(self):
        if self.listWidget2.count() == 0:
            message = QtGui.QMessageBox.information(self, "No Files Selected",  "No files have been selected. Please select at least one to continue")
        else:
            self.Pages.setCurrentWidget(self.LoadDataPage)

    def BackButton1_clicked(self):
        self.Pages.setCurrentWidget(self.StartPage)

    def AddButton_clicked(self):
        for i in self.listWidget1.selectedItems(): 
            self.AddFilesList(self.listWidget2, i.text())
            self.RemoveFilesList(self.listWidget1, self.listWidget1.row(i))
 #           self.AddFilesList(self.GraphsList, i.text())

    def RemoveButton_clicked(self):
        for i in self.listWidget2.selectedItems():
            self.AddFilesList(self.listWidget1, i.text())
            self.RemoveFilesList(self.listWidget2, self.listWidget2.row(i))
            #self.RemoveFilesList(self.GraphsList, self.GraphsList.row(i))

        #Edit Obstacle Buttons

    def PlotButton_clicked(self):
        #add image to graphicsview
        pass
        
    def DoneButton1_clicked(self):
        self.Pages.setCurrentWidget(self.DefCoursePage)  
    
        #Obstacle Detail Buttons

        #Load Data Page Buttons

##    def selectFile():
##        lineEdit1.setText(QFileDialog.getOpenFileName())
##        pushButton.clicked.connect(selectFile)
    def LoadButton_clicked(self):
        fileNames = QtGui.QFileDialog.getOpenFileName(self, "Select one or more files to open")
        self.AddFilesList(self.listWidget7, fileNames)
##        mlab.start()
##        mlab.run('C:/Users/leia/Dropbox (MIT)/GUI/AgilityGUI/UM_agility.m')
##        mlab.stop()

    def LoadPreButton_clicked(self):
        pass
              
    def ContinueButton2_clicked(self):
        if self.listWidget2.count() == 0:
           message = QtGui.QMessageBox.information(self, "No Files Selected",  "No files have been selected. Please select at least one to continue")
        else:
            self.Pages.setCurrentWidget(self.ProcessDataPage)
            self.download()

    def BackButton2_clicked(self):
        self.Pages.setCurrentWidget(self.DefCoursePage)

        #Process Page Buttons
        
        #Conflict Page Buttons
    def BackButton4_clicked(self):
        self.Pages.setCurrentWidget(self.LoadDataPage)

    def ContinueButton3_clicked(self):
        self.Pages.setCurrentWidget(self.ScoreReportPage)

        #Score Report Page Buttons
    def BackButton5_clicked(self):
        self.Pages.setCurrentWidget(self.ConflictPage)
            
##        #Plot page Selection
##    def SummaryMetricsButton_clicked(self):
##        self.Lists.setCurrentWidget(self.SummaryMetricsPage)
##        self.ObstaclesButton.setChecked(False)
##        self.MotionPrimitiveButton.setChecked(False)
##
##    def ObstaclesButton_clicked(self):
##        self.Lists.setCurrentWidget(self.ObstaclesPage)
##        self.SummaryMetricsButton.setChecked(False)
##        self.MotionPrimitiveButton.setChecked(False)
##
##    def MotionPrimitiveButton_clicked(self):
##        self.Lists.setCurrentWidget(self.MotionPrimitivePage)
##        self.SummaryMetricsButton.setChecked(False)
##        self.ObstaclesButton.setChecked(False)
##
##    def Plot(self):
##        self.AddGraph(fig0)
##        self.show()
##        self.canvas.draw()
##
##    def BackButton5_clicked(self):
##        self.Pages.setCurrentWidget(self.ConflictPage)

#List functions

    def AddFilesList(self, lst, name):
        lst.addItem(name)

    def RemoveFilesList(self, lst, name):
        lst.takeItem(name)       

    def AddGraph(self, fig):
        # Create a canvas object
        self.canvas = FigureCanvas(fig)
        # Add canvas object to vertical layout widget
        self.mplvl.addWidget(self.canvas)
        # Draw actual canvas
        self.canvas.draw()

    def removegraph(self):
        # Remove canvas from vertical layout widget
        self.mplvl.removeWidget(self.canvas)
        self.canvas.close()

# Progress Bar Function
# Currently is not connected to file download

    def download(self):
        self.completed = 0
        while self.completed < 100:
            self.completed += 0.0001 # Add small increments to simulate a dowload
            self.ProgressBar.setValue(self.completed)
        self.Pages.setCurrentWidget(self.ConflictPage)
##        if self.mplvl.count() == 0: # Plots only if there is one canvas
##            self.Plot()             # Should probably make a different function for this
##        self.Lists.setCurrentWidget(self.SummaryMetricsPage)
        
## File opening and reading

file1 = open("plot.txt", "r") # Open sample data file
file1_string = file1.read() # Read file and create a straing variable
file1.close()

# Create a list of only numbers (convert strings to integers for plotting)
lst = []
i = 0
while i < len(file1_string):
    if file1_string[i] != " ":
        lst.append(int(file1_string[i]))
    i+= 1

#Axes figure
fig0 = Figure()
ax1f0 = fig0.add_subplot(111)

# Figure that can be plotted
fig1 = Figure()
ax1f1 = fig1.add_subplot(111)
ax1f1.plot(lst)
#ax1f1.plot(np.random.rand(5))

fig2 = Figure()
ax1f2 = fig2.add_subplot(121)
ax1f2.plot(np.random.rand(5))
ax2f2 = fig2.add_subplot(122)
ax2f2.plot(np.random.rand(10))

fig3 = Figure()
ax1f3 = fig3.add_subplot(111)
ax1f3.pcolormesh(np.random.rand(20,20))


# Execute functions
app = QtGui.QApplication(sys.argv)
myWindow = MyWindowClass(None)
myWindow.show()
app.exec_()

