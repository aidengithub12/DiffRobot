import tkinter as tk
from tkinter import ttk # For themed widgets
import DataRead as dr

root = tk.Tk()
root.title("Differential Robot Debug Console!")
root.geometry("1080x720")

#Constant Widgets
label_title = ttk.Label(root, text="Diff Robot Debug Console",font=("Arial",30))
seperator_title = ttk.Frame(root,height=5,border=1,relief=tk.SUNKEN)
GyroWidget = ttk.Label(root,text="GYRO/ACCEL",font=("Arial",20))
DistanceSensorWidget = ttk.Label(root, text="Distance Sensor",font=("Arial",20))
SpeedWidget = ttk.Label(root, text="Speed",font = ("Arial",20))
VoltageWidget = ttk.Label(root, text="Battery Voltage", font = ("Arial", 20))
CurrentWidget = ttk.Label(root,text="Battery Current",font=("Arial",20))
IsRunningWidget = ttk.Label(root, text="IsRunning?", font=("Arial", 20))
seperator_middle = ttk.Separator(root,orient="horizontal")

#list of seperated data to change every upadte
dataList = dr.parseData(dr.getData())

#function - ignore
def updateData(data:list):
    data = dr.parseData(dr.getData())
    GyroTV.set(data[0])
    DistanceSensorTV.set(data[1])
    SpeedTV.set(data[2])
    BattVoltageTV.set(data[3])
    BattCurrentTV.set(data[4])
    IsRunningTV.set(data[5])
    root.after(1000,updateData,dataList)
    


#Changing Widgets
GyroTV = tk.StringVar(root,dataList[0])
GyroData = ttk.Label(root, textvariable=GyroTV,font=("Arial",15))
DistanceSensorTV = tk.StringVar(root,dataList[1])
DistanceSensorData = ttk.Label(root,textvariable=DistanceSensorTV,font=("Arial",15))
SpeedTV = tk.StringVar(root,dataList[2])
SpeedData = ttk.Label(root,textvariable=SpeedTV,font=("Arial",15))
BattVoltageTV = tk.StringVar(root,dataList[3])
BattVoltageData = ttk.Label(root, textvariable=BattVoltageTV,font=("Arial",15))
BattCurrentTV = tk.StringVar(root,dataList[4])
BattCurrentData = ttk.Label(root, textvariable=BattCurrentTV,font=("Arial",15))
IsRunningTV = tk.StringVar(root,dataList[5])
IsRunningData = ttk.Label(root, textvariable=IsRunningTV,font=("Arial",15))

#Buttons
# UpdateButton = ttk.Button(root, command= lambda:updateData(dataList))

#Draw widgets
#cont widgets
label_title.pack(pady=10)
seperator_title.pack(pady=15,fill='x')
seperator_middle.place(rely=0.50,width=1920)
GyroWidget.place(relx=0,rely=0.15)
DistanceSensorWidget.place(relx=0.4,rely=0.15)
SpeedWidget.place(relx=0.9, rely=0.15)
VoltageWidget.place(relx=0,rely=0.55)
CurrentWidget.place(relx=0.4,rely=0.55)
IsRunningWidget.place(relx=0.9,rely=0.55)
#changing widgets
GyroData.place(relx=0.05,rely=0.3)
DistanceSensorData.place(relx=0.4,rely=0.3)
SpeedData.place(relx=0.9,rely=0.3)
BattVoltageData.place(relx=0.05,rely=0.75)
BattCurrentData.place(relx=0.5,rely=0.75)
IsRunningData.place(relx=0.9,rely=0.75)

#place button
# UpdateButton.place(relx=0.45,rely=0.9)

#main loop
updateData(dataList)
root.mainloop()



    